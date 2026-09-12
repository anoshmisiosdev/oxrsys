// SPDX-License-Identifier: MPL-2.0
//
// WiredHeadset backed by the Monado Windows Mixed Reality driver (drivers/).
//
// Tracking: a thread polls the driver's head pose, predicted ahead by about
// two frames, and injects it into a TrackingReceiver as if it had arrived from
// a streaming client, so InputManager, spaces, and prediction see no
// difference. Position is orientation-only for now (a fixed eye height).
//
// Display: the session's frames go through a latest-frame-only queue to a
// presenter thread that waits on the swapchain snapshot's shared event and
// draws both eyes through the lens distortion mesh onto the panel.

#include "WiredHeadset.h"

#include "Config.h"
#include "StreamingFrameQueue.h"
#include "TrackingReceiver.h"

#include "wmr_macos.h"
#include "wmr_panel.h"

#include "os/os_time.h"
#include "xrt/xrt_device.h"

#include <oxrsys/protocol/Protocol.h>

#import <Metal/Metal.h>

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

namespace
{

int64_t SteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

constexpr float kDefaultIpdM = 0.063f;
constexpr int kTrackingRateHz = 250;

} // namespace

struct WiredHeadset::Impl
{
    std::mutex openMutex;
    std::atomic<bool> open{false};
    struct oxrsys_wmr_headset* headset = nullptr;
    std::unique_ptr<oxrsys::WmrPanel> panel;
    bool panelOpen = false;
    std::string name;
    uint32_t refreshHz = 90;
    float eyeHeightM = 1.6f;

    // Tracking.
    std::unique_ptr<TrackingReceiver> tracking;
    std::thread trackingThread;
    std::atomic<bool> trackingRunning{false};

    // Presentation.
    std::mutex presentMutex;
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> presentQueue = nil;
    StreamingFrameQueue frameQueue;
    std::thread presentThread;
    std::atomic<bool> presenting{false};
    std::atomic<uint64_t> presentedFrames{0};
    std::atomic<uint32_t> droppedFrames{0};

    void TrackingLoop();
    void PresentLoop();
};

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

bool WiredHeadset::EnsureOpen(const ConfigValues& config)
{
    std::lock_guard<std::mutex> lock(impl_->openMutex);
    if (impl_->open.load())
    {
        return true;
    }
    if (!config.wiredHeadset)
    {
        return false;
    }

    // Displays before the panel switches on, so a hot-plugged display is
    // recognised even without a mode of exactly the panel's size.
    std::vector<CGDirectDisplayID> displaysBefore = oxrsys::WmrPanel::OnlineDisplays();

    struct oxrsys_wmr_headset* headset = nullptr;
    const enum oxrsys_wmr_open_result result = oxrsys_wmr_headset_open(U_LOGGING_INFO, &headset);
    if (result != OXRSYS_WMR_OPEN_OK)
    {
        if (result == OXRSYS_WMR_OPEN_NO_HEADSET)
        {
            spdlog::info("WiredHeadset: wired_headset is enabled but no Windows Mixed Reality headset is connected");
        }
        else
        {
            spdlog::error("WiredHeadset: failed to open the headset: {}", oxrsys_wmr_open_result_str(result));
        }
        return false;
    }
    impl_->headset = headset;
    impl_->name = std::string("Windows Mixed Reality ") + oxrsys_wmr_headset_type_str(headset->type);
    impl_->eyeHeightM = config.wiredEyeHeightM;

    impl_->panel = std::make_unique<oxrsys::WmrPanel>(headset->hmd->hmd);
    oxrsys::WmrPanelOptions options;
    options.display_id = config.wiredDisplayId;
    options.display_timeout_s = 20.0;
    options.displays_before = std::move(displaysBefore);
    impl_->panelOpen = impl_->panel->Open(options);
    if (impl_->panelOpen)
    {
        const double hz = impl_->panel->RefreshRateHz();
        impl_->refreshHz = hz > 1.0 ? static_cast<uint32_t>(std::lround(hz)) : 90u;
    }
    else
    {
        spdlog::error("WiredHeadset: the panel did not appear as a display; tracking works but nothing "
                      "will be shown. See docs/platforms/wmr.md (EDID override).");
    }

    const oxrsys::WmrPanelGeometry& geometry = impl_->panel->Geometry();
    spdlog::info("WiredHeadset: {} open, panel {}x{} @ {} Hz, eye {}x{}, display {}",
                 impl_->name, geometry.panel_w, geometry.panel_h, impl_->refreshHz,
                 geometry.views[0].render_w, geometry.views[0].render_h,
                 impl_->panelOpen ? "ready" : "missing");

    impl_->tracking = std::make_unique<TrackingReceiver>();
    impl_->trackingRunning.store(true);
    impl_->trackingThread = std::thread([this] { impl_->TrackingLoop(); });

    impl_->open.store(true);
    return true;
}

bool WiredHeadset::IsOpen() const
{
    return impl_->open.load();
}

void WiredHeadset::Close()
{
    DetachGraphics();

    std::lock_guard<std::mutex> lock(impl_->openMutex);
    if (!impl_->open.load())
    {
        return;
    }
    impl_->open.store(false);

    impl_->trackingRunning.store(false);
    if (impl_->trackingThread.joinable())
    {
        impl_->trackingThread.join();
    }
    impl_->tracking.reset();

    if (impl_->panel)
    {
        impl_->panel->Close();
        impl_->panel.reset();
    }
    if (impl_->headset != nullptr)
    {
        oxrsys_wmr_headset_close(&impl_->headset);
    }
    impl_->panelOpen = false;
}

uint32_t WiredHeadset::GetEyeWidth() const
{
    return impl_->panel ? impl_->panel->Geometry().views[0].render_w : 0;
}

uint32_t WiredHeadset::GetEyeHeight() const
{
    return impl_->panel ? impl_->panel->Geometry().views[0].render_h : 0;
}

uint32_t WiredHeadset::GetRefreshRateHz() const
{
    return impl_->refreshHz;
}

std::string WiredHeadset::GetName() const
{
    return impl_->name;
}

TrackingReceiver* WiredHeadset::GetTrackingReceiver()
{
    return impl_->tracking.get();
}

uint64_t WiredHeadset::GetPresentedFrameCount() const
{
    return impl_->presentedFrames.load();
}

bool WiredHeadset::AttachGraphics(const GraphicsContext& graphicsContext)
{
    if (!impl_->open.load() || graphicsContext.api != GraphicsApi::Metal || graphicsContext.metalDevice == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->presentMutex);
    if (impl_->presenting.load())
    {
        return true;
    }

    id<MTLDevice> device = (__bridge id<MTLDevice>)graphicsContext.metalDevice;
    if (!impl_->panel->EnsureRenderer(device))
    {
        spdlog::error("WiredHeadset: could not create the panel renderer on the session's Metal device");
        return false;
    }
    impl_->device = device;
    // Our own queue: the swapchain snapshot signals a shared event that the
    // warp waits on, so no ordering with the app's queue is needed.
    impl_->presentQueue = [device newCommandQueue];
    impl_->presentQueue.label = @"OXRSys wired headset presenter";

    impl_->frameQueue.Start();
    impl_->presenting.store(true);
    impl_->presentThread = std::thread([this] { impl_->PresentLoop(); });
    spdlog::info("WiredHeadset: presenter started");
    return true;
}

void WiredHeadset::DetachGraphics()
{
    std::lock_guard<std::mutex> lock(impl_->presentMutex);
    if (!impl_->presenting.load())
    {
        return;
    }
    impl_->presenting.store(false);
    impl_->frameQueue.Stop();
    if (impl_->presentThread.joinable())
    {
        impl_->presentThread.join();
    }
    impl_->frameQueue.Clear();
    impl_->presentQueue = nil;
    impl_->device = nil;
    spdlog::info("WiredHeadset: presenter stopped after {} frames ({} dropped)",
                 impl_->presentedFrames.load(), impl_->droppedFrames.load());
}

bool WiredHeadset::IsPresenting() const
{
    return impl_->presenting.load() && impl_->panelOpen;
}

void WiredHeadset::SendFrame(FrameSource frameSource)
{
    if (!impl_->presenting.load())
    {
        return;
    }
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

/*
 *
 * Threads.
 *
 */

void WiredHeadset::Impl::TrackingLoop()
{
    pthread_setname_np("oxrsys-wired-tracking");

    const oxrsys::WmrPanelGeometry& geometry = panel->Geometry();
    const int64_t periodNs = 1000000000LL / kTrackingRateHz;
    // About two frames: xrWaitFrame samples the pose, the app renders one
    // frame, and the presenter shows it at the next refresh.
    const int64_t horizonNs = 2 * (1000000000LL / std::max<uint32_t>(refreshHz, 1));

    struct xrt_device* hmd = headset->hmd;
    uint64_t samples = 0;
    int64_t lastLogNs = SteadyNowNs();
    bool wasValid = false;

    while (trackingRunning.load())
    {
        const int64_t loopStartNs = SteadyNowNs();

        struct xrt_space_relation relation = {};
        const int64_t monadoNowNs = os_monotonic_get_ns();
        const xrt_result_t xret = xrt_device_get_tracked_pose(hmd, XRT_INPUT_GENERIC_HEAD_POSE,
                                                              monadoNowNs + horizonNs, &relation);
        const bool valid = xret == XRT_SUCCESS &&
                           (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;

        oxr::protocol::TrackingPacket packet = {};
        packet.timestampNs = loopStartNs;
        packet.trackingFlags = 0;
        packet.headPosition[0] = 0.0f;
        packet.headPosition[1] = eyeHeightM;
        packet.headPosition[2] = 0.0f;
        if (valid)
        {
            packet.headOrientation[0] = relation.pose.orientation.x;
            packet.headOrientation[1] = relation.pose.orientation.y;
            packet.headOrientation[2] = relation.pose.orientation.z;
            packet.headOrientation[3] = relation.pose.orientation.w;
        }
        else
        {
            packet.headOrientation[3] = 1.0f;
        }
        if ((relation.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0)
        {
            packet.headAngularVelocity[0] = relation.angular_velocity.x;
            packet.headAngularVelocity[1] = relation.angular_velocity.y;
            packet.headAngularVelocity[2] = relation.angular_velocity.z;
        }
        packet.ipd = kDefaultIpdM;
        packet.eyeFov[0] = geometry.views[0].fov[0];
        packet.eyeFov[1] = geometry.views[0].fov[1];
        packet.eyeFov[2] = geometry.views[0].fov[2];
        packet.eyeFov[3] = geometry.views[0].fov[3];

        tracking->InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        samples++;

        if (valid != wasValid)
        {
            spdlog::info("WiredHeadset: head orientation {}", valid ? "valid" : "INVALID");
            wasValid = valid;
        }
        if (loopStartNs - lastLogNs >= 10000000000LL)
        {
            spdlog::debug("WiredHeadset: {} tracking samples, {} frames presented, {} dropped", samples,
                          presentedFrames.load(), droppedFrames.load());
            lastLogNs = loopStartNs;
        }

        const int64_t elapsedNs = SteadyNowNs() - loopStartNs;
        if (elapsedNs < periodNs)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(periodNs - elapsedNs));
        }
    }
}

void WiredHeadset::Impl::PresentLoop()
{
    pthread_setname_np("oxrsys-wired-present");

    while (presenting.load())
    {
        StreamingFrame frame = {};
        if (!frameQueue.WaitPop(presenting, frame))
        {
            continue;
        }
        if (!frame.valid || !panelOpen || !panel->IsReady())
        {
            frame.source.Reset();
            droppedFrames.fetch_add(1);
            continue;
        }

        @autoreleasepool
        {
            oxrsys::WmrPanelEyeInput eyes[2];
            const FrameImageSource* sources[2] = {&frame.source.left, &frame.source.right};
            for (int i = 0; i < 2; i++)
            {
                eyes[i].texture = (__bridge id<MTLTexture>)sources[i]->image.get();
                if (sources[i]->sync.IsValid())
                {
                    eyes[i].waitEvent = (__bridge id<MTLSharedEvent>)sources[i]->sync.waitObject.get();
                    eyes[i].waitValue = sources[i]->sync.waitValue;
                }
            }

            // The staging lease lives in the FrameSource; hand it to the GPU
            // completion so the slot is not reused while the warp reads it.
            auto keepAlive = std::make_shared<FrameSource>(std::move(frame.source));
            const bool presented = panel->Present(presentQueue, eyes[0], eyes[1], ^{
                keepAlive->Reset();
            });
            if (presented)
            {
                presentedFrames.fetch_add(1);
            }
            else
            {
                keepAlive->Reset();
                droppedFrames.fetch_add(1);
            }
        }
    }
}
