// SPDX-License-Identifier: MPL-2.0

#import "HevcEncoderHelperClient.h"

#define OXRSYS_ENC_IPC_WANT_MACH 1
#import "../encoder_helper/EncoderHelperIpc.h"

#import <IOSurface/IOSurface.h>

#include <mach/mach.h>
#include <servers/bootstrap.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <random>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <pthread/qos.h>
#include <spdlog/spdlog.h>

extern char** environ;

using namespace oxrsys::enc_ipc;

namespace
{

constexpr int kChildSocketFd = 3; // fd the helper reads its control socket on

bool WriteAllFd(int fd, const uint8_t* data, size_t len)
{
    size_t off = 0;
    while (off < len)
    {
        ssize_t n = ::write(fd, data + off, len - off);
        if (n > 0) { off += (size_t)n; continue; }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool ReadAllFd(int fd, uint8_t* data, size_t len)
{
    size_t off = 0;
    while (off < len)
    {
        ssize_t n = ::read(fd, data + off, len - off);
        if (n > 0) { off += (size_t)n; continue; }
        if (n == 0) return false;
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool RecvFramed(int fd, MsgType& type, std::vector<uint8_t>& payload)
{
    uint8_t header[kHeaderBytes];
    if (!ReadAllFd(fd, header, kHeaderBytes)) return false;
    Reader r(header, kHeaderBytes);
    uint32_t magic = r.U32();
    uint16_t t = r.U16();
    r.U16();
    uint32_t len = r.U32();
    if (magic != kMagic || len > kMaxPayloadBytes) return false;
    type = (MsgType)t;
    payload.resize(len);
    if (len > 0 && !ReadAllFd(fd, payload.data(), len)) return false;
    return true;
}

std::string MakeRendezvousName()
{
    std::random_device rd;
    uint64_t nonce = ((uint64_t)rd() << 32) ^ ((uint64_t)rd() ^ ((uint64_t)getpid() << 16));
    char buf[128];
    snprintf(buf, sizeof(buf), "org.oxrsys.enc.%d.%016llx", (int)getpid(),
             (unsigned long long)nonce);
    return buf;
}

} // namespace

HevcEncoderHelperClient::~HevcEncoderHelperClient()
{
    Stop();
}

bool HevcEncoderHelperClient::SendFramed(uint16_t type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> framed = Frame((MsgType)type, payload);
    std::lock_guard<std::mutex> lock(writeMutex_);
    if (sockFd_ < 0) return false;
    return WriteAllFd(sockFd_, framed.data(), framed.size());
}

bool HevcEncoderHelperClient::Start(const Config& config, void* const* iosurfaces, size_t count,
                                    OnNal onNal, OnFrameDone onFrameDone)
{
    if (count == 0 || count > kMaxSlots || config.helperPath.empty())
    {
        spdlog::error("EncoderHelper: invalid start args (count={}, path='{}')", count,
                      config.helperPath);
        return false;
    }
    if (access(config.helperPath.c_str(), X_OK) != 0)
    {
        spdlog::warn("EncoderHelper: helper binary not found/executable at '{}' — using in-process "
                     "software encoder",
                     config.helperPath);
        return false;
    }

    onNal_ = std::move(onNal);
    onFrameDone_ = std::move(onFrameDone);
    slotCount_ = (uint32_t)count;

    // --- Control socket (parent <-> child) ---
    int sv[2] = {-1, -1};
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0)
    {
        spdlog::error("EncoderHelper: socketpair failed: {}", strerror(errno));
        return false;
    }
    // --- stderr capture pipe (child stderr -> parent log) ---
    int errpipe[2] = {-1, -1};
    if (pipe(errpipe) != 0)
    {
        spdlog::error("EncoderHelper: pipe failed: {}", strerror(errno));
        ::close(sv[0]); ::close(sv[1]);
        return false;
    }

    // --- Mach rendezvous: check in a per-spawn name BEFORE spawning. The
    // parent owns the receive right; the child looks the name up and sends us a
    // send right to its own port so we can push surface ports to it. We do NOT
    // use posix_spawnattr_setspecialport (libxpc latches the bootstrap port
    // before main() and the child would hang). ---
    const std::string rendezvous = MakeRendezvousName();
    mach_port_t bootstrapPort = MACH_PORT_NULL;
    task_get_bootstrap_port(mach_task_self(), &bootstrapPort);
    mach_port_t parentRx = MACH_PORT_NULL;
    kern_return_t kr = bootstrap_check_in(bootstrapPort, rendezvous.c_str(), &parentRx);
    if (kr != KERN_SUCCESS || parentRx == MACH_PORT_NULL)
    {
        spdlog::error("EncoderHelper: bootstrap_check_in('{}') failed: {} ({})", rendezvous, kr,
                      bootstrap_strerror(kr));
        ::close(sv[0]); ::close(sv[1]); ::close(errpipe[0]); ::close(errpipe[1]);
        return false;
    }

    // --- Spawn the helper ---
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, sv[1], kChildSocketFd);
    posix_spawn_file_actions_adddup2(&fa, errpipe[1], STDERR_FILENO);
    // Close every inherited fd we don't want the child to keep, but NEVER an fd
    // number that is also a dup2 target: dup2 already closed the target's old
    // occupant, and an explicit close of that number would kill the descriptor
    // we just installed (socketpair/pipe can hand back fd 3 or 2 themselves).
    auto closeIfNotTarget = [&](int fd) {
        if (fd != kChildSocketFd && fd != STDERR_FILENO)
            posix_spawn_file_actions_addclose(&fa, fd);
    };
    closeIfNotTarget(sv[0]);
    closeIfNotTarget(errpipe[0]);
    closeIfNotTarget(sv[1]);
    closeIfNotTarget(errpipe[1]);

    char fdArg[16];
    snprintf(fdArg, sizeof(fdArg), "%d", kChildSocketFd);
    std::string pathCopy = config.helperPath;
    std::string rzCopy = rendezvous;
    char* argv[] = {
        pathCopy.data(),
        (char*)"--socket-fd", fdArg,
        (char*)"--rendezvous", rzCopy.data(),
        nullptr,
    };

    pid_t pid = -1;
    int spawnRc = posix_spawn(&pid, config.helperPath.c_str(), &fa, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    // Parent no longer needs the child ends.
    ::close(sv[1]);
    ::close(errpipe[1]);

    if (spawnRc != 0)
    {
        spdlog::error("EncoderHelper: posix_spawn('{}') failed: {}", config.helperPath,
                      strerror(spawnRc));
        ::close(sv[0]); ::close(errpipe[0]);
        mach_port_mod_refs(mach_task_self(), parentRx, MACH_PORT_RIGHT_RECEIVE, -1);
        return false;
    }
    sockFd_ = sv[0];
    stderrReadFd_ = errpipe[0];
    childPid_ = pid;
    spdlog::info("EncoderHelper: spawned pid={} path={} rendezvous={}", pid, config.helperPath,
                 rendezvous);

    // Forward the helper's stderr into the runtime log.
    stderrThread_ = std::thread([this]() {
        char line[1024];
        std::string acc;
        for (;;)
        {
            ssize_t n = ::read(stderrReadFd_, line, sizeof(line));
            if (n <= 0) break;
            acc.append(line, (size_t)n);
            size_t nl;
            while ((nl = acc.find('\n')) != std::string::npos)
            {
                spdlog::info("EncoderHelper[child]: {}", acc.substr(0, nl));
                acc.erase(0, nl + 1);
            }
        }
    });

    // 1. Send Init config.
    {
        std::vector<uint8_t> p;
        PutU32(p, config.width);
        PutU32(p, config.height);
        PutU32(p, config.fps);
        PutU32(p, config.bitrateMbps);
        PutU32(p, config.keyframeIntervalSec);
        PutU32(p, (uint32_t)count);
        PutU32(p, config.preset);
        if (!SendFramed((uint16_t)MsgType::Init, p))
        {
            spdlog::error("EncoderHelper: failed to send Init");
            mach_port_mod_refs(mach_task_self(), parentRx, MACH_PORT_RIGHT_RECEIVE, -1);
            MarkDead("init send failed");
            return false;
        }
    }

    // 2. Receive the child's port (send right) via mach, with a bounded wait.
    mach_port_t childPort = MACH_PORT_NULL;
    {
        PortMsgRecv rmsg = {};
        kr = mach_msg(&rmsg.msg.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rmsg), parentRx,
                      5000 /*ms*/, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS || rmsg.msg.header.msgh_id != kMsgIdChildPort)
        {
            spdlog::error("EncoderHelper: did not receive child port (kr={}, id=0x{:x})", kr,
                          rmsg.msg.header.msgh_id);
            mach_port_mod_refs(mach_task_self(), parentRx, MACH_PORT_RIGHT_RECEIVE, -1);
            MarkDead("mach rendezvous timeout");
            return false;
        }
        childPort = rmsg.msg.port.name;
    }
    // Done with our rendezvous receive right (and the bootstrap name).
    mach_port_mod_refs(mach_task_self(), parentRx, MACH_PORT_RIGHT_RECEIVE, -1);

    // 3. Transfer one IOSurface send right per slot to the child.
    bool transferOk = true;
    for (size_t i = 0; i < count; ++i)
    {
        IOSurfaceRef surf = (IOSurfaceRef)iosurfaces[i];
        if (surf == nullptr)
        {
            spdlog::error("EncoderHelper: slot {} has no IOSurface", i);
            transferOk = false;
            break;
        }
        mach_port_t surfPort = IOSurfaceCreateMachPort(surf);
        if (surfPort == MACH_PORT_NULL)
        {
            spdlog::error("EncoderHelper: IOSurfaceCreateMachPort(slot {}) failed", i);
            transferOk = false;
            break;
        }

        PortMsg msg = {};
        msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
        msg.header.msgh_size = sizeof(msg);
        msg.header.msgh_remote_port = childPort;
        msg.header.msgh_local_port = MACH_PORT_NULL;
        msg.header.msgh_id = kMsgIdSurface;
        msg.body.msgh_descriptor_count = 1;
        msg.port.name = surfPort;
        msg.port.disposition = MACH_MSG_TYPE_MOVE_SEND; // move into message
        msg.port.type = MACH_MSG_PORT_DESCRIPTOR;
        msg.slot = (uint32_t)i;
        msg.width = (uint32_t)IOSurfaceGetWidth(surf);
        msg.height = (uint32_t)IOSurfaceGetHeight(surf);
        msg.pixelFormat = kPixelFormatBGRA;

        kr = mach_msg(&msg.header, MACH_SEND_MSG, sizeof(msg), 0, MACH_PORT_NULL,
                      MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS)
        {
            spdlog::error("EncoderHelper: mach_msg(send surface slot {}) failed: {}", i, kr);
            // MOVE_SEND did not consume on failure — drop the orphaned right.
            mach_port_deallocate(mach_task_self(), surfPort);
            transferOk = false;
            break;
        }
    }
    mach_port_deallocate(mach_task_self(), childPort);

    if (!transferOk)
    {
        MarkDead("surface transfer failed");
        return false;
    }

    // 4. Await InitAck.
    {
        MsgType type;
        std::vector<uint8_t> payload;
        if (!RecvFramed(sockFd_, type, payload) || type != MsgType::InitAck)
        {
            spdlog::error("EncoderHelper: no InitAck (helper likely died during init)");
            MarkDead("no init ack");
            return false;
        }
        Reader r(payload.data(), payload.size());
        uint32_t status = r.U32();
        const uint8_t* hw = r.Bytes(1);
        bool hardware = hw && *hw;
        if (!r.ok() || status != (uint32_t)InitStatus::Ok || !hardware)
        {
            spdlog::warn("EncoderHelper: init not OK (status={}, hardware={}) — falling back to "
                         "in-process software encoder",
                         status, hardware);
            MarkDead("init not ok");
            return false;
        }
        usingHardware_.store(true);
    }

    alive_.store(true);
    readerThread_ = std::thread([this]() { ReaderLoop(); });
    spdlog::info("EncoderHelper: ready — hardware HEVC encoder live in native-arm64 helper (pid={})",
                 childPid_);
    return true;
}

void HevcEncoderHelperClient::ReaderLoop()
{
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
    for (;;)
    {
        MsgType type;
        std::vector<uint8_t> payload;
        if (!RecvFramed(sockFd_, type, payload))
        {
            if (!stopping_.load())
                MarkDead("helper socket closed (crash or exit)");
            return;
        }
        Reader r(payload.data(), payload.size());
        if (type == MsgType::Nal)
        {
            uint64_t cookie = r.U64();
            int64_t pts = r.I64();
            const uint8_t* keyp = r.Bytes(1);
            uint32_t dataLen = r.U32();
            const uint8_t* data = r.Bytes(dataLen);
            if (r.ok() && data != nullptr && onNal_)
                onNal_(cookie, data, dataLen, keyp && *keyp, pts);
        }
        else if (type == MsgType::FrameDone)
        {
            uint64_t cookie = r.U64();
            const uint8_t* dropped = r.Bytes(1);
            double encodeMs = r.F64();
            const uint8_t* key = r.Bytes(1);
            if (r.ok() && onFrameDone_)
                onFrameDone_(cookie, dropped && *dropped, encodeMs, key && *key);
        }
    }
}

void HevcEncoderHelperClient::MarkDead(const char* reason)
{
    bool was = alive_.exchange(false);
    if (was || sockFd_ >= 0)
    {
        spdlog::warn("EncoderHelper: marking helper dead: {}", reason);
    }
    // Closing the socket unblocks the reader thread and signals the child EOF.
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        if (sockFd_ >= 0)
        {
            ::close(sockFd_);
            sockFd_ = -1;
        }
    }
    // Only notify a genuine mid-session death (was alive), exactly once, so the
    // owner can reclaim in-flight frames. Init-time failures (never alive) and
    // orderly Stop() do not fire this.
    if (was && !diedNotified_.exchange(true) && onDied_)
    {
        onDied_();
    }
}

void HevcEncoderHelperClient::SubmitFrame(uint64_t cookie, uint32_t slot, int64_t ptsNs,
                                          bool forceKeyframe)
{
    if (!alive_.load()) return;
    std::vector<uint8_t> p;
    PutU64(p, cookie);
    PutU32(p, slot);
    PutI64(p, ptsNs);
    p.push_back(forceKeyframe ? 1 : 0);
    if (!SendFramed((uint16_t)MsgType::Encode, p))
        MarkDead("encode submit write failed");
}

void HevcEncoderHelperClient::SetBitrate(uint32_t bitrateMbps)
{
    if (!alive_.load()) return;
    std::vector<uint8_t> p;
    PutU32(p, bitrateMbps);
    SendFramed((uint16_t)MsgType::SetBitrate, p);
}

void HevcEncoderHelperClient::Stop()
{
    if (stopping_.exchange(true))
    {
        // Already stopping; still join if threads are around.
    }
    else if (alive_.load())
    {
        std::vector<uint8_t> empty;
        SendFramed((uint16_t)MsgType::Shutdown, empty);
    }

    // Close the socket to unblock the reader (also EOFs the child).
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        if (sockFd_ >= 0) { ::close(sockFd_); sockFd_ = -1; }
    }
    alive_.store(false);

    if (readerThread_.joinable()) readerThread_.join();

    if (stderrReadFd_ >= 0) { ::close(stderrReadFd_); stderrReadFd_ = -1; }
    if (stderrThread_.joinable()) stderrThread_.join();

    if (childPid_ > 0)
    {
        // Give the child a moment to exit cleanly, then reap (kill if needed).
        for (int i = 0; i < 100; ++i)
        {
            int st = 0;
            pid_t r = waitpid(childPid_, &st, WNOHANG);
            if (r == childPid_ || (r < 0 && errno == ECHILD)) { childPid_ = -1; break; }
            usleep(2000);
        }
        if (childPid_ > 0)
        {
            kill(childPid_, SIGKILL);
            waitpid(childPid_, nullptr, 0);
            childPid_ = -1;
        }
    }
}
