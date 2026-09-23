// SPDX-License-Identifier: MPL-2.0
//
// Offline end-to-end smoke test for the native-arm64 encoder helper.
//
// Mimics the exact runtime scenario WITHOUT Wine/the game: a parent process
// (built x86_64 so it runs under Rosetta, just like the runtime dylib's host)
// creates IOSurface-backed BGRA buffers via a CVPixelBufferPool (as the runtime
// does), spawns the arm64 helper, performs the mach rendezvous + IOSurface
// transfer, submits a handful of frames, and prints the hardware flag, the NAL
// sizes, and the per-frame hardware encode time.
//
// PASS criteria: "InitAck hardware=YES" and frames returning NAL units with
// single-digit-ms encode times. That proves surface sharing + IPC + hardware
// VT session across the x86_64->arm64 boundary — the whole mechanism.
//
// Build (parent as x86_64 to mimic Rosetta):
//   xcrun clang++ -arch x86_64 -std=c++17 -O2 runtime/encoder_helper/smoke_test.mm \
//     -o build/helper/smoke_test_x64 \
//     -framework Foundation -framework CoreFoundation -framework CoreVideo \
//     -framework CoreMedia -framework IOSurface
//   build/helper/smoke_test_x64 build/helper/oxrsys-encoder-helper

#define OXRSYS_ENC_IPC_WANT_MACH 1
#include "EncoderHelperIpc.h"

#import <CoreVideo/CoreVideo.h>
#import <IOSurface/IOSurface.h>

#include <mach/mach.h>
#include <servers/bootstrap.h>

#include <cstdio>
#include <cstring>
#include <random>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
using namespace oxrsys::enc_ipc;

static const int kChildSocketFd = 3;
static const uint32_t kW = 2272, kH = 1264, kSlots = 3, kFps = 72, kMbps = 50;

static bool WriteAll(int fd, const uint8_t* d, size_t n)
{
    size_t o = 0;
    while (o < n) { ssize_t r = write(fd, d + o, n - o); if (r <= 0) { if (r < 0 && errno == EINTR) continue; return false; } o += r; }
    return true;
}
static bool ReadAll(int fd, uint8_t* d, size_t n)
{
    size_t o = 0;
    while (o < n) { ssize_t r = read(fd, d + o, n - o); if (r == 0) return false; if (r < 0) { if (errno == EINTR) continue; return false; } o += r; }
    return true;
}
static bool Send(int fd, MsgType t, const std::vector<uint8_t>& p)
{
    auto f = Frame(t, p);
    return WriteAll(fd, f.data(), f.size());
}
static bool Recv(int fd, MsgType& t, std::vector<uint8_t>& p)
{
    uint8_t h[kHeaderBytes];
    if (!ReadAll(fd, h, kHeaderBytes)) return false;
    Reader r(h, kHeaderBytes);
    if (r.U32() != kMagic) return false;
    t = (MsgType)r.U16();
    r.U16();
    uint32_t len = r.U32();
    p.resize(len);
    return len == 0 || ReadAll(fd, p.data(), len);
}

int main(int argc, char** argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <helper-path>\n", argv[0]); return 2; }
    const char* helperPath = argv[1];

    // 1. IOSurface-backed BGRA pool, exactly like VideoEncoder::Initialize.
    NSDictionary* attrs = @{
        (NSString*)kCVPixelBufferWidthKey : @(kW),
        (NSString*)kCVPixelBufferHeightKey : @(kH),
        (NSString*)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
        (NSString*)kCVPixelBufferIOSurfacePropertiesKey : @{},
        (NSString*)kCVPixelBufferMetalCompatibilityKey : @YES,
    };
    CVPixelBufferPoolRef pool = nullptr;
    if (CVPixelBufferPoolCreate(kCFAllocatorDefault, nullptr, (__bridge CFDictionaryRef)attrs,
                                &pool) != kCVReturnSuccess) { fprintf(stderr, "pool create failed\n"); return 3; }
    CVPixelBufferRef pbs[kSlots] = {};
    IOSurfaceRef surfs[kSlots] = {};
    for (uint32_t i = 0; i < kSlots; ++i)
    {
        if (CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pbs[i]) != kCVReturnSuccess)
        { fprintf(stderr, "pb create %u failed\n", i); return 3; }
        surfs[i] = CVPixelBufferGetIOSurface(pbs[i]);
        // Fill with a nonzero pattern so the encoder has real content.
        IOSurfaceLock(surfs[i], 0, nullptr);
        memset(IOSurfaceGetBaseAddress(surfs[i]), 0x40 + i * 0x20,
               IOSurfaceGetHeight(surfs[i]) * IOSurfaceGetBytesPerRow(surfs[i]));
        IOSurfaceUnlock(surfs[i], 0, nullptr);
    }
    printf("[smoke] created %u IOSurface-backed %ux%u BGRA buffers\n", kSlots, kW, kH);

    // 2. socketpair + mach rendezvous check-in.
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    std::random_device rd;
    char name[128];
    snprintf(name, sizeof(name), "org.oxrsys.enc.smoke.%d.%08x", getpid(), rd());
    mach_port_t bp = MACH_PORT_NULL;
    task_get_bootstrap_port(mach_task_self(), &bp);
    mach_port_t parentRx = MACH_PORT_NULL;
    kern_return_t kr = bootstrap_check_in(bp, name, &parentRx);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "[smoke] bootstrap_check_in failed: %d (%s)\n", kr, bootstrap_strerror(kr)); return 4; }

    // 3. Spawn arm64 helper.
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, sv[1], kChildSocketFd);
    // Only close the parent end if it is not the dup2 target (dup2 already
    // closed the target's old occupant). Closing the target would kill the
    // socket we just installed.
    if (sv[0] != kChildSocketFd) posix_spawn_file_actions_addclose(&fa, sv[0]);
    if (sv[1] != kChildSocketFd) posix_spawn_file_actions_addclose(&fa, sv[1]);
    char fdArg[16]; snprintf(fdArg, sizeof(fdArg), "%d", kChildSocketFd);
    char* av[] = { (char*)helperPath, (char*)"--socket-fd", fdArg, (char*)"--rendezvous", name, nullptr };
    pid_t pid = -1;
    int rc = posix_spawn(&pid, helperPath, &fa, nullptr, av, environ);
    posix_spawn_file_actions_destroy(&fa);
    close(sv[1]);
    if (rc != 0) { fprintf(stderr, "[smoke] posix_spawn failed: %s\n", strerror(rc)); return 5; }
    int sock = sv[0];
    printf("[smoke] spawned helper pid=%d\n", pid);

    // 4. Init.
    { std::vector<uint8_t> p; PutU32(p, kW); PutU32(p, kH); PutU32(p, kFps); PutU32(p, kMbps);
      PutU32(p, 2); PutU32(p, kSlots); PutU32(p, 0); Send(sock, MsgType::Init, p); }

    // 5. Receive child port, send surfaces.
    PortMsgRecv rmsg = {};
    kr = mach_msg(&rmsg.msg.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rmsg), parentRx, 5000, MACH_PORT_NULL);
    if (kr != KERN_SUCCESS || rmsg.msg.header.msgh_id != kMsgIdChildPort)
    { fprintf(stderr, "[smoke] no child port: kr=%d id=0x%x\n", kr, rmsg.msg.header.msgh_id); return 6; }
    mach_port_t childPort = rmsg.msg.port.name;
    printf("[smoke] received child mach port\n");
    for (uint32_t i = 0; i < kSlots; ++i)
    {
        mach_port_t sp = IOSurfaceCreateMachPort(surfs[i]);
        PortMsg m = {};
        m.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
        m.header.msgh_size = sizeof(m);
        m.header.msgh_remote_port = childPort;
        m.header.msgh_id = kMsgIdSurface;
        m.body.msgh_descriptor_count = 1;
        m.port.name = sp; m.port.disposition = MACH_MSG_TYPE_MOVE_SEND; m.port.type = MACH_MSG_PORT_DESCRIPTOR;
        m.slot = i; m.width = kW; m.height = kH; m.pixelFormat = kPixelFormatBGRA;
        kr = mach_msg(&m.header, MACH_SEND_MSG, sizeof(m), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS) { fprintf(stderr, "[smoke] send surface %u failed: %d\n", i, kr); return 7; }
    }
    printf("[smoke] transferred %u surfaces via mach\n", kSlots);

    // 6. InitAck.
    MsgType t; std::vector<uint8_t> p;
    if (!Recv(sock, t, p) || t != MsgType::InitAck) { fprintf(stderr, "[smoke] no InitAck\n"); return 8; }
    { Reader r(p.data(), p.size()); uint32_t st = r.U32(); const uint8_t* hw = r.Bytes(1);
      printf("[smoke] InitAck status=%u hardware=%s\n", st, (hw && *hw) ? "YES" : "NO");
      if (st != 0 || !(hw && *hw)) { fprintf(stderr, "[smoke] FAIL: helper did not get hardware encoder\n"); }
    }

    // 7. Encode one frame at a time (submit, wait for its FrameDone), exactly as
    // the runtime gates slot reuse on encode completion. Rewrite the slot's
    // surface each iteration so the encoder does real work on fresh content.
    // This yields the TRUE isolated per-frame hardware encode cost.
    const int kFrames = 40;
    size_t nalBytes = 0; int nalCount = 0;
    double sumMs = 0; double maxMs = 0; int measured = 0;
    for (int f = 0; f < kFrames; ++f)
    {
        uint32_t slot = (uint32_t)(f % kSlots);
        // Fresh, non-degenerate content (a moving gradient) so the encoder does
        // representative work rather than re-emitting an identical frame.
        IOSurfaceLock(surfs[slot], 0, nullptr);
        uint8_t* base = (uint8_t*)IOSurfaceGetBaseAddress(surfs[slot]);
        size_t bpr = IOSurfaceGetBytesPerRow(surfs[slot]);
        for (uint32_t y = 0; y < kH; ++y)
            memset(base + y * bpr, (uint8_t)((f * 7 + y) & 0xFF), bpr);
        IOSurfaceUnlock(surfs[slot], 0, nullptr);

        std::vector<uint8_t> e;
        PutU64(e, (uint64_t)(f + 1));
        PutU32(e, slot);
        PutI64(e, (int64_t)f * 13888889);
        e.push_back(f == 0 ? 1 : 0);
        Send(sock, MsgType::Encode, e);

        // Wait for this frame's terminal result before moving on.
        bool frameDone = false;
        while (!frameDone)
        {
            if (!Recv(sock, t, p)) { fprintf(stderr, "[smoke] socket closed early\n"); return 9; }
            Reader r(p.data(), p.size());
            if (t == MsgType::Nal) { r.U64(); r.I64(); r.Bytes(1); uint32_t len = r.U32(); nalBytes += len; nalCount++; }
            else if (t == MsgType::FrameDone)
            {
                uint64_t cookie = r.U64(); const uint8_t* dr = r.Bytes(1); double ms = r.F64(); const uint8_t* key = r.Bytes(1);
                printf("[smoke] frame cookie=%llu dropped=%d encodeMs=%.2f key=%d\n",
                       (unsigned long long)cookie, dr && *dr, ms, key && *key);
                if (cookie > 5) { sumMs += ms; maxMs = ms > maxMs ? ms : maxMs; measured++; } // skip warmup
                frameDone = true;
            }
        }
        usleep(2000); // small gap between frames
    }
    printf("[smoke] total NAL units=%d bytes=%zu\n", nalCount, nalBytes);
    if (measured > 0)
        printf("[smoke] isolated per-frame HARDWARE HEVC encode (frames 6..%d): avg=%.2fms max=%.2fms\n",
               kFrames, sumMs / measured, maxMs);

    Send(sock, MsgType::Shutdown, {});
    close(sock);
    int status = 0; waitpid(pid, &status, 0);
    for (uint32_t i = 0; i < kSlots; ++i) CVPixelBufferRelease(pbs[i]);
    CVPixelBufferPoolRelease(pool);
    printf("[smoke] done (helper exit status=%d)\n", WEXITSTATUS(status));
    return 0;
}
