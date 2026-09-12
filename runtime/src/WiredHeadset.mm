// SPDX-License-Identifier: MPL-2.0
//
// WiredHeadset: client of oxrsys-headset-helper (runtime/headset_helper).
//
// The helper owns the headset, its display, the driver and tracking, and
// shows a lobby between sessions. This side, living inside the game process
// (possibly x86_64 under Rosetta), connects to it over a Unix socket, spawns
// it if it is not running, receives tracking packets and feeds them into a
// TrackingReceiver as if they came from a streaming client, and hands the
// helper each frame through shared IOSurfaces: the session's eye snapshots
// are composed side by side into a slot surface on the GPU and the slot
// number is sent over the socket. Nothing here touches USB or AppKit, which
// is what makes it work inside Wine.

#include "WiredHeadset.h"

#include "Config.h"
#include "StreamingFrameQueue.h"
#include "TrackingReceiver.h"

#define OXRSYS_ENC_IPC_WANT_MACH 1
#include "../headset_helper/HeadsetHelperIpc.h"

#include <oxrsys/protocol/Protocol.h>

#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>

#include <spdlog/spdlog.h>

#include <mach/mach.h>
#include <servers/bootstrap.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

extern char** environ;

using namespace oxrsys::headset_ipc;
using oxrsys::enc_ipc::PortMsg;
using oxrsys::enc_ipc::PortMsgRecv;
using oxrsys::enc_ipc::kMsgIdChildPort;
using oxrsys::enc_ipc::kMsgIdSurface;
using oxrsys::enc_ipc::Reader;

namespace
{

// Four: one being shown, one queued, one being composed, one spare so a slot
// released after the next present does not starve a 90 Hz producer.
constexpr uint32_t kSlots = 4;

int64_t SteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool WriteAll(int fd, const uint8_t* data, size_t len)
{
    while (len > 0)
    {
        ssize_t n = ::send(fd, data, len, MSG_NOSIGNAL);
        if (n <= 0)
        {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        data += n;
        len -= (size_t)n;
    }
    return true;
}

bool ReadAll(int fd, uint8_t* data, size_t len)
{
    while (len > 0)
    {
        ssize_t n = ::recv(fd, data, len, 0);
        if (n <= 0)
        {
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        data += n;
        len -= (size_t)n;
    }
    return true;
}

bool RecvFramed(int fd, MsgType& type, std::vector<uint8_t>& payload)
{
    uint8_t hdr[oxrsys::enc_ipc::kHeaderBytes];
    if (!ReadAll(fd, hdr, sizeof(hdr))) return false;
    Reader r(hdr, sizeof(hdr));
    const uint32_t magic = r.U32();
    const uint16_t t = r.U16();
    const uint16_t version = r.U16();
    const uint32_t len = r.U32();
    if (magic != oxrsys::enc_ipc::kMagic || version != kProtocolVersion ||
        len > oxrsys::enc_ipc::kMaxPayloadBytes)
    {
        return false;
    }
    payload.resize(len);
    if (len > 0 && !ReadAll(fd, payload.data(), len)) return false;
    type = (MsgType)t;
    return true;
}

// Compose: draw one eye snapshot into half of the shared slot surface.
const char* kComposeShader = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct Out { float4 position [[position]]; float2 uv; };
vertex Out compose_vertex(uint vid [[vertex_id]]) {
    float2 pos = float2((vid == 1) ? 3.0 : -1.0, (vid == 2) ? -3.0 : 1.0);
    Out o; o.position = float4(pos, 0.0, 1.0); o.uv = float2((pos.x + 1.0) * 0.5, (1.0 - pos.y) * 0.5); return o;
}
fragment float4 compose_fragment(Out in [[stage_in]], texture2d<float> eye [[texture(0)]]) {
    constexpr sampler s(address::clamp_to_edge, filter::linear);
    return float4(eye.sample(s, in.uv).rgb, 1.0);
}
)MSL";

std::string HelperPath(const ConfigValues& config)
{
    if (!config.wiredHelperPath.empty()) return config.wiredHelperPath;
    if (const char* env = getenv("OXRSYS_HEADSET_HELPER_PATH"); env != nullptr && env[0] != '\0') return env;
    return Config::Get().dylibDir + "/oxrsys-headset-helper";
}

} // namespace

struct WiredHeadset::Impl
{
    std::mutex openMutex;
    std::atomic<bool> open{false};
    int fd = -1;
    std::mutex writeMutex;
    HeadsetInfo info;
    std::string rendezvous;
    mach_port_t rendezvousRx = MACH_PORT_NULL;

    std::unique_ptr<TrackingReceiver> tracking;
    std::thread readerThread;
    std::atomic<bool> readerRunning{false};

    // Surfaces handed to the helper.
    std::mutex slotMutex;
    std::condition_variable slotCv;
    IOSurfaceRef surfaces[kSlots] = {};
    id<MTLTexture> slotTextures[kSlots] = {};
    bool slotInFlight[kSlots] = {};
    bool surfacesAcked = false;
    SurfacesStatus surfacesStatus = SurfacesStatus::TransferFailed;

    // Presentation.
    std::mutex presentMutex;
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLRenderPipelineState> composePipeline = nil;
    StreamingFrameQueue frameQueue;
    std::thread presentThread;
    std::atomic<bool> presenting{false};
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint32_t> dropped{0};

    bool Send(MsgType type, const std::vector<uint8_t>& payload);
    void ReaderLoop();
    void PresentLoop();
    void ReleaseSurfaces();
};

bool WiredHeadset::Impl::Send(MsgType type, const std::vector<uint8_t>& payload)
{
    if (fd < 0) return false;
    const std::vector<uint8_t> bytes = Frame(type, payload);
    std::lock_guard<std::mutex> lock(writeMutex);
    return WriteAll(fd, bytes.data(), bytes.size());
}

WiredHeadset& WiredHeadset::Shared()
{
    static WiredHeadset shared;
    return shared;
}

WiredHeadset::WiredHeadset() : impl_(std::make_unique<Impl>())
{
    impl_->frameQueue.SetReleaseFrameCallback([](StreamingFrame& frame) { frame.source.Reset(); });
}

WiredHeadset::~WiredHeadset()
{
    Close();
}

bool WiredHeadset::IsSupported()
{
    return true;
}

namespace
{

int ConnectSocket(const std::string& path)
{
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr = {};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0)
    {
        ::close(fd);
        return -1;
    }
    const int bufSize = 1 << 20;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
    return fd;
}

bool SpawnHelper(const std::string& path, const std::string& socketPath)
{
    if (access(path.c_str(), X_OK) != 0)
    {
        spdlog::error("WiredHeadset: helper not found at '{}' (set wired_helper_path or "
                      "$OXRSYS_HEADSET_HELPER_PATH)",
                      path);
        return false;
    }
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    // Its own session: it outlives the game and keeps the lobby up.
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
    std::string pathCopy = path;
    std::string sockCopy = socketPath;
    char* argv[] = {pathCopy.data(), (char*)"--socket", sockCopy.data(), nullptr};
    pid_t pid = -1;
    const int rc = posix_spawn(&pid, path.c_str(), nullptr, &attr, argv, environ);
    posix_spawnattr_destroy(&attr);
    if (rc != 0)
    {
        spdlog::error("WiredHeadset: posix_spawn('{}') failed: {}", path, strerror(rc));
        return false;
    }
    spdlog::info("WiredHeadset: spawned helper pid {} ({})", pid, path);
    return true;
}

} // namespace

bool WiredHeadset::EnsureOpen(const ConfigValues& config)
{
    std::lock_guard<std::mutex> lock(impl_->openMutex);
    if (impl_->open.load()) return true;
    if (!config.wiredHeadset) return false;

    const std::string socketPath = DefaultSocketPath();
    int fd = ConnectSocket(socketPath);
    if (fd < 0)
    {
        if (!SpawnHelper(HelperPath(config), socketPath)) return false;
        // Opening the headset and finding its display takes a few seconds.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (fd < 0 && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            fd = ConnectSocket(socketPath);
        }
        if (fd < 0)
        {
            spdlog::error("WiredHeadset: helper did not come up on {}", socketPath);
            return false;
        }
    }
    impl_->fd = fd;

    // Mach rendezvous name for surface transfer, checked in once per process.
    if (impl_->rendezvousRx == MACH_PORT_NULL)
    {
        impl_->rendezvous = "com.oxrsys.headset." + std::to_string(getpid());
        mach_port_t bootstrapPort = MACH_PORT_NULL;
        task_get_bootstrap_port(mach_task_self(), &bootstrapPort);
        const kern_return_t kr = bootstrap_check_in(bootstrapPort, impl_->rendezvous.c_str(), &impl_->rendezvousRx);
        if (kr != KERN_SUCCESS)
        {
            spdlog::error("WiredHeadset: bootstrap_check_in('{}') failed: {} ({})", impl_->rendezvous, kr,
                          bootstrap_strerror(kr));
            impl_->rendezvousRx = MACH_PORT_NULL;
        }
    }

    // Hello, then wait for the headset description.
    {
        std::vector<uint8_t> p;
        oxrsys::enc_ipc::PutU16(p, kProtocolVersion);
        oxrsys::enc_ipc::PutU32(p, (uint32_t)getpid());
        PutString(p, impl_->rendezvous);
        if (!impl_->Send(MsgType::Hello, p))
        {
            spdlog::error("WiredHeadset: hello failed");
            ::close(impl_->fd);
            impl_->fd = -1;
            return false;
        }
    }
    {
        struct timeval tv = {5, 0};
        setsockopt(impl_->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        MsgType type;
        std::vector<uint8_t> payload;
        bool got = false;
        while (RecvFramed(impl_->fd, type, payload))
        {
            if (type == MsgType::HeadsetInfo && DecodeHeadsetInfo(payload, impl_->info))
            {
                got = true;
                break;
            }
            // Tracking may already be flowing; skip until the info arrives.
        }
        struct timeval none = {0, 0};
        setsockopt(impl_->fd, SOL_SOCKET, SO_RCVTIMEO, &none, sizeof(none));
        if (!got)
        {
            spdlog::error("WiredHeadset: no HeadsetInfo from the helper");
            ::close(impl_->fd);
            impl_->fd = -1;
            return false;
        }
    }

    spdlog::info("WiredHeadset: helper serves {}: panel {}x{} @ {} Hz, eye {}x{}, controllers {}, display {}",
                 impl_->info.name, impl_->info.panelW, impl_->info.panelH, impl_->info.refreshHz,
                 impl_->info.eyeW, impl_->info.eyeH, impl_->info.controllers,
                 impl_->info.displayReady ? "ready" : "missing");

    impl_->tracking = std::make_unique<TrackingReceiver>();
    impl_->readerRunning.store(true);
    impl_->readerThread = std::thread([this] { impl_->ReaderLoop(); });
    impl_->open.store(true);
    return true;
}

void WiredHeadset::Impl::ReaderLoop()
{
    pthread_setname_np("oxrsys-wired-reader");
    MsgType type;
    std::vector<uint8_t> payload;
    while (readerRunning.load() && RecvFramed(fd, type, payload))
    {
        switch (type)
        {
            case MsgType::Tracking:
                if (payload.size() >= sizeof(oxr::protocol::TrackingPacket) && tracking)
                {
                    tracking->InjectPacket(payload.data(), payload.size());
                }
                break;
            case MsgType::FrameReleased:
            {
                Reader r(payload.data(), payload.size());
                const uint32_t slot = r.U32();
                std::lock_guard<std::mutex> lock(slotMutex);
                if (slot < kSlots) slotInFlight[slot] = false;
                break;
            }
            case MsgType::SurfacesAck:
            {
                Reader r(payload.data(), payload.size());
                std::lock_guard<std::mutex> lock(slotMutex);
                surfacesStatus = (SurfacesStatus)r.U32();
                surfacesAcked = true;
                slotCv.notify_all();
                break;
            }
            case MsgType::HeadsetInfo:
                DecodeHeadsetInfo(payload, info);
                break;
            default:
                break;
        }
    }
    if (readerRunning.load())
    {
        spdlog::warn("WiredHeadset: connection to the helper closed");
    }
    // Unblock anyone waiting for an ack.
    std::lock_guard<std::mutex> lock(slotMutex);
    surfacesAcked = true;
    slotCv.notify_all();
}

bool WiredHeadset::IsOpen() const
{
    return impl_->open.load();
}

void WiredHeadset::Close()
{
    DetachGraphics();
    std::lock_guard<std::mutex> lock(impl_->openMutex);
    if (!impl_->open.load()) return;
    impl_->open.store(false);
    impl_->Send(MsgType::Bye, {});
    impl_->readerRunning.store(false);
    if (impl_->fd >= 0)
    {
        ::shutdown(impl_->fd, SHUT_RDWR);
        ::close(impl_->fd);
        impl_->fd = -1;
    }
    if (impl_->readerThread.joinable()) impl_->readerThread.join();
    impl_->tracking.reset();
}

uint32_t WiredHeadset::GetEyeWidth() const
{
    return impl_->info.eyeW;
}

uint32_t WiredHeadset::GetEyeHeight() const
{
    return impl_->info.eyeH;
}

uint32_t WiredHeadset::GetRefreshRateHz() const
{
    return impl_->info.refreshHz > 0 ? impl_->info.refreshHz : 90;
}

std::string WiredHeadset::GetName() const
{
    return impl_->info.name;
}

TrackingReceiver* WiredHeadset::GetTrackingReceiver()
{
    return impl_->tracking.get();
}

uint64_t WiredHeadset::GetPresentedFrameCount() const
{
    return impl_->submitted.load();
}

void WiredHeadset::Impl::ReleaseSurfaces()
{
    std::lock_guard<std::mutex> lock(slotMutex);
    for (uint32_t i = 0; i < kSlots; i++)
    {
        slotTextures[i] = nil;
        if (surfaces[i] != nullptr)
        {
            CFRelease(surfaces[i]);
            surfaces[i] = nullptr;
        }
        slotInFlight[i] = false;
    }
    surfacesAcked = false;
}

bool WiredHeadset::AttachGraphics(const GraphicsContext& graphicsContext)
{
    if (!impl_->open.load() || graphicsContext.api != GraphicsApi::Metal || graphicsContext.metalDevice == nullptr)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl_->presentMutex);
    if (impl_->presenting.load()) return true;
    if (impl_->info.panelW == 0 || impl_->info.panelH == 0 || impl_->rendezvousRx == MACH_PORT_NULL)
    {
        return false;
    }

    id<MTLDevice> device = (__bridge id<MTLDevice>)graphicsContext.metalDevice;
    impl_->device = device;
    impl_->queue = [device newCommandQueue];
    impl_->queue.label = @"OXRSys wired headset compose";

    NSError* error = nil;
    id<MTLLibrary> lib = [device newLibraryWithSource:@(kComposeShader) options:nil error:&error];
    if (lib == nil)
    {
        spdlog::error("WiredHeadset: compose shader failed: {}", error.localizedDescription.UTF8String);
        return false;
    }
    MTLRenderPipelineDescriptor* desc = [MTLRenderPipelineDescriptor new];
    desc.vertexFunction = [lib newFunctionWithName:@"compose_vertex"];
    desc.fragmentFunction = [lib newFunctionWithName:@"compose_fragment"];
    desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
    impl_->composePipeline = [device newRenderPipelineStateWithDescriptor:desc error:&error];
    if (impl_->composePipeline == nil)
    {
        spdlog::error("WiredHeadset: compose pipeline failed: {}", error.localizedDescription.UTF8String);
        return false;
    }

    // Slot surfaces: side-by-side stereo at the panel's eye size.
    const uint32_t width = impl_->info.eyeW * 2;
    const uint32_t height = impl_->info.eyeH;
    impl_->ReleaseSurfaces();
    for (uint32_t i = 0; i < kSlots; i++)
    {
        const size_t bytesPerRow = IOSurfaceAlignProperty(kIOSurfaceBytesPerRow, (size_t)width * 4);
        NSDictionary* props = @{
            (id)kIOSurfaceWidth : @(width),
            (id)kIOSurfaceHeight : @(height),
            (id)kIOSurfaceBytesPerElement : @4,
            (id)kIOSurfaceBytesPerRow : @(bytesPerRow),
            (id)kIOSurfacePixelFormat : @((uint32_t)'BGRA'),
        };
        IOSurfaceRef surf = IOSurfaceCreate((__bridge CFDictionaryRef)props);
        if (surf == nullptr)
        {
            spdlog::error("WiredHeadset: IOSurfaceCreate failed for slot {}", i);
            impl_->ReleaseSurfaces();
            return false;
        }
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                     width:width
                                                                                    height:height
                                                                                 mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        id<MTLTexture> tex = [device newTextureWithDescriptor:td iosurface:surf plane:0];
        if (tex == nil)
        {
            spdlog::error("WiredHeadset: IOSurface texture failed for slot {}", i);
            CFRelease(surf);
            impl_->ReleaseSurfaces();
            return false;
        }
        impl_->surfaces[i] = surf;
        impl_->slotTextures[i] = tex;
    }

    // Tell the helper, then push the surfaces over Mach once it answers.
    {
        std::vector<uint8_t> p;
        oxrsys::enc_ipc::PutU32(p, kSlots);
        if (!impl_->Send(MsgType::Surfaces, p))
        {
            impl_->ReleaseSurfaces();
            return false;
        }
    }
    mach_port_t helperPort = MACH_PORT_NULL;
    {
        PortMsgRecv rmsg = {};
        const kern_return_t kr = mach_msg(&rmsg.msg.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rmsg),
                                          impl_->rendezvousRx, 5000, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS || rmsg.msg.header.msgh_id != kMsgIdChildPort)
        {
            spdlog::error("WiredHeadset: helper did not answer the surface rendezvous (kr={})", kr);
            impl_->ReleaseSurfaces();
            return false;
        }
        helperPort = rmsg.msg.port.name;
    }
    for (uint32_t i = 0; i < kSlots; i++)
    {
        mach_port_t surfPort = IOSurfaceCreateMachPort(impl_->surfaces[i]);
        PortMsg msg = {};
        msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
        msg.header.msgh_size = sizeof(msg);
        msg.header.msgh_remote_port = helperPort;
        msg.header.msgh_id = kMsgIdSurface;
        msg.body.msgh_descriptor_count = 1;
        msg.port.name = surfPort;
        msg.port.disposition = MACH_MSG_TYPE_MOVE_SEND;
        msg.port.type = MACH_MSG_PORT_DESCRIPTOR;
        msg.slot = i;
        msg.width = width;
        msg.height = height;
        msg.pixelFormat = oxrsys::enc_ipc::kPixelFormatBGRA;
        const kern_return_t kr =
            mach_msg(&msg.header, MACH_SEND_MSG, sizeof(msg), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (kr != KERN_SUCCESS)
        {
            spdlog::error("WiredHeadset: sending surface {} failed: {}", i, kr);
            mach_port_deallocate(mach_task_self(), surfPort);
            mach_port_deallocate(mach_task_self(), helperPort);
            impl_->ReleaseSurfaces();
            return false;
        }
    }
    mach_port_deallocate(mach_task_self(), helperPort);
    {
        std::unique_lock<std::mutex> slotLock(impl_->slotMutex);
        impl_->slotCv.wait_for(slotLock, std::chrono::seconds(5), [this] { return impl_->surfacesAcked; });
        if (!impl_->surfacesAcked || impl_->surfacesStatus != SurfacesStatus::Ok)
        {
            spdlog::error("WiredHeadset: helper rejected the surfaces (status {})", (uint32_t)impl_->surfacesStatus);
            slotLock.unlock();
            impl_->ReleaseSurfaces();
            return false;
        }
    }

    impl_->frameQueue.Start();
    impl_->presenting.store(true);
    impl_->presentThread = std::thread([this] { impl_->PresentLoop(); });
    spdlog::info("WiredHeadset: {} frame surfaces shared with the helper ({}x{})", kSlots, width, height);
    return true;
}

void WiredHeadset::DetachGraphics()
{
    std::lock_guard<std::mutex> lock(impl_->presentMutex);
    if (!impl_->presenting.load()) return;
    impl_->presenting.store(false);
    impl_->frameQueue.Stop();
    if (impl_->presentThread.joinable()) impl_->presentThread.join();
    impl_->frameQueue.Clear();
    // The helper keeps showing the last frame for half a second, then the
    // lobby; the surfaces stay valid on its side until the next Surfaces.
    impl_->ReleaseSurfaces();
    impl_->composePipeline = nil;
    impl_->queue = nil;
    impl_->device = nil;
    spdlog::info("WiredHeadset: presenter stopped after {} frames ({} dropped)", impl_->submitted.load(),
                 impl_->dropped.load());
}

bool WiredHeadset::IsPresenting() const
{
    return impl_->presenting.load();
}

void WiredHeadset::SendFrame(FrameSource frameSource)
{
    if (!impl_->presenting.load()) return;
    StreamingFrame frame = {};
    frame.source = std::move(frameSource);
    frame.timestampNs = SteadyNowNs();
    frame.valid = frame.source.IsStereoValid();
    if (!frame.valid)
    {
        frame.source.Reset();
        return;
    }
    impl_->frameQueue.PushLatest(std::move(frame));
}

void WiredHeadset::Impl::PresentLoop()
{
    pthread_setname_np("oxrsys-wired-compose");
    while (presenting.load())
    {
        StreamingFrame frame = {};
        if (!frameQueue.WaitPop(presenting, frame)) continue;
        if (!frame.valid)
        {
            frame.source.Reset();
            continue;
        }

        // A free slot: not in flight at the helper.
        int slot = -1;
        {
            std::lock_guard<std::mutex> lock(slotMutex);
            for (uint32_t i = 0; i < kSlots; i++)
            {
                if (!slotInFlight[i] && slotTextures[i] != nil)
                {
                    slot = (int)i;
                    break;
                }
            }
            if (slot >= 0) slotInFlight[slot] = true;
        }
        if (slot < 0)
        {
            frame.source.Reset();
            dropped.fetch_add(1);
            continue;
        }

        @autoreleasepool
        {
            id<MTLTexture> target = slotTextures[slot];
            id<MTLCommandBuffer> cmd = [queue commandBuffer];
            const FrameImageSource* sources[2] = {&frame.source.left, &frame.source.right};
            for (int i = 0; i < 2; i++)
            {
                if (sources[i]->sync.IsValid())
                {
                    [cmd encodeWaitForEvent:(__bridge id<MTLSharedEvent>)sources[i]->sync.waitObject.get()
                                      value:sources[i]->sync.waitValue];
                }
            }
            MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
            rp.colorAttachments[0].texture = target;
            rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
            rp.colorAttachments[0].storeAction = MTLStoreActionStore;
            id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
            [enc setRenderPipelineState:composePipeline];
            const double half = (double)target.width / 2.0;
            for (int i = 0; i < 2; i++)
            {
                MTLViewport vp = {i * half, 0.0, half, (double)target.height, 0.0, 1.0};
                [enc setViewport:vp];
                MTLScissorRect sc = {(NSUInteger)(i * half), 0, (NSUInteger)half, target.height};
                [enc setScissorRect:sc];
                [enc setFragmentTexture:(__bridge id<MTLTexture>)sources[i]->image.get() atIndex:0];
                [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
            }
            [enc endEncoding];
            [cmd commit];
            // Cross-process GPU sync is not available; wait here (off the app
            // thread) so the helper never reads a half-written surface.
            [cmd waitUntilCompleted];
        }
        frame.source.Reset();

        SubmitFrame sf;
        sf.slot = (uint32_t)slot;
        sf.timestampNs = frame.timestampNs;
        sf.hasPose = frame.hasPose;
        for (int i = 0; i < 4; i++) sf.orientation[i] = frame.headOrientation[i];
        for (int i = 0; i < 3; i++) sf.position[i] = frame.headPosition[i];
        if (!Send(MsgType::SubmitFrame, EncodeSubmitFrame(sf)))
        {
            std::lock_guard<std::mutex> lock(slotMutex);
            slotInFlight[slot] = false;
            dropped.fetch_add(1);
            continue;
        }
        submitted.fetch_add(1);
    }
}
