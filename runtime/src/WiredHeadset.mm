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

#include "psmv_macos.h"
#include "wmr_macos.h"
#include "wmr_panel.h"
#include "wmr_psmv_tracking.h"

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
constexpr size_t kMaxPsMove = 4;

// Orientation-only controllers get a fixed "arm model" position relative to
// the head: hands slightly in front, below, and to either side, following
// the head's yaw so they stay in view when the user turns.
constexpr float kArmOffsetX = 0.18f;
constexpr float kArmOffsetY = -0.35f;
constexpr float kArmOffsetZ = -0.40f;

enum class Hand
{
    Left = 0,
    Right = 1,
};

// The first pose-typed input is the grip pose on every driver we use.
enum xrt_input_name GripPoseInput(const struct xrt_device* xdev)
{
    for (size_t i = 0; i < xdev->input_count; i++)
    {
        if (XRT_GET_INPUT_TYPE(xdev->inputs[i].name) == XRT_INPUT_TYPE_POSE)
        {
            return xdev->inputs[i].name;
        }
    }
    return XRT_INPUT_GENERIC_HEAD_POSE;
}

// Map one controller's inputs onto the packet's Touch-style fields. WMR
// controllers have a thumbstick, trackpad, menu, squeeze and trigger; PS
// Moves have face buttons, Move (used as grip), Start (menu) and a trigger.
void MapControllerInputs(const struct xrt_device* xdev, Hand hand, oxr::protocol::TrackingPacket& packet)
{
    using namespace oxr::protocol;
    const bool left = hand == Hand::Left;
    float& trigger = left ? packet.leftTrigger : packet.rightTrigger;
    float& grip = left ? packet.leftGrip : packet.rightGrip;
    float* stick = left ? packet.leftThumbstick : packet.rightThumbstick;
    const uint32_t primaryClick = left ? BUTTON_X : BUTTON_A;    // lower face button
    const uint32_t secondaryClick = left ? BUTTON_Y : BUTTON_B;  // upper face button
    const uint32_t stickClick = left ? BUTTON_LEFT_THUMBSTICK : BUTTON_RIGHT_THUMBSTICK;
    const uint32_t triggerClick = left ? BUTTON_LEFT_TRIGGER : BUTTON_RIGHT_TRIGGER;
    const uint32_t gripClick = left ? BUTTON_LEFT_GRIP : BUTTON_RIGHT_GRIP;

    for (size_t i = 0; i < xdev->input_count; i++)
    {
        const struct xrt_input& in = xdev->inputs[i];
        const bool b = in.value.boolean;
        switch (in.name)
        {
            // Windows Mixed Reality (original, Odyssey, Reverb G2).
            case XRT_INPUT_WMR_TRIGGER_VALUE:
            case XRT_INPUT_ODYSSEY_CONTROLLER_TRIGGER_VALUE:
            case XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE:
            case XRT_INPUT_PSMV_TRIGGER_VALUE:
                trigger = in.value.vec1.x;
                if (trigger > 0.5f) packet.buttonState |= triggerClick;
                break;
            case XRT_INPUT_WMR_SQUEEZE_CLICK:
            case XRT_INPUT_ODYSSEY_CONTROLLER_SQUEEZE_CLICK:
            case XRT_INPUT_PSMV_MOVE_CLICK:
                if (b) { grip = 1.0f; packet.buttonState |= gripClick; }
                break;
            case XRT_INPUT_G2_CONTROLLER_SQUEEZE_VALUE:
                grip = in.value.vec1.x;
                if (grip > 0.5f) packet.buttonState |= gripClick;
                break;
            case XRT_INPUT_WMR_MENU_CLICK:
            case XRT_INPUT_ODYSSEY_CONTROLLER_MENU_CLICK:
            case XRT_INPUT_G2_CONTROLLER_MENU_CLICK:
            case XRT_INPUT_PSMV_START_CLICK:
                if (b) packet.buttonState |= BUTTON_MENU;
                break;
            case XRT_INPUT_WMR_THUMBSTICK:
            case XRT_INPUT_ODYSSEY_CONTROLLER_THUMBSTICK:
            case XRT_INPUT_G2_CONTROLLER_THUMBSTICK:
                stick[0] = in.value.vec2.x;
                stick[1] = in.value.vec2.y;
                break;
            case XRT_INPUT_WMR_THUMBSTICK_CLICK:
            case XRT_INPUT_ODYSSEY_CONTROLLER_THUMBSTICK_CLICK:
            case XRT_INPUT_G2_CONTROLLER_THUMBSTICK_CLICK:
                if (b) packet.buttonState |= stickClick;
                break;
            // The WMR trackpad click stands in for the lower face button.
            case XRT_INPUT_WMR_TRACKPAD_CLICK:
            case XRT_INPUT_ODYSSEY_CONTROLLER_TRACKPAD_CLICK:
            case XRT_INPUT_G2_CONTROLLER_A_CLICK:
            case XRT_INPUT_G2_CONTROLLER_X_CLICK:
            case XRT_INPUT_PSMV_CROSS_CLICK:
                if (b) packet.buttonState |= primaryClick;
                break;
            case XRT_INPUT_G2_CONTROLLER_B_CLICK:
            case XRT_INPUT_G2_CONTROLLER_Y_CLICK:
            case XRT_INPUT_PSMV_CIRCLE_CLICK:
                if (b) packet.buttonState |= secondaryClick;
                break;
            default:
                break;
        }
    }
}

// Rotate a vector by a quaternion: v' = v + 2w(q x v) + 2 q x (q x v).
struct xrt_vec3 quat_rotate(const struct xrt_quat& q, const struct xrt_vec3& v)
{
    const float cx = q.y * v.z - q.z * v.y;
    const float cy = q.z * v.x - q.x * v.z;
    const float cz = q.x * v.y - q.y * v.x;
    const float ccx = q.y * cz - q.z * cy;
    const float ccy = q.z * cx - q.x * cz;
    const float ccz = q.x * cy - q.y * cx;
    return {v.x + 2.0f * (q.w * cx + ccx), v.y + 2.0f * (q.w * cy + ccy), v.z + 2.0f * (q.w * cz + ccz)};
}

// Rotate a vector by the yaw component of a quaternion only.
struct xrt_vec3 YawRotate(const struct xrt_quat& q, float x, float y, float z)
{
    const float yaw = atan2f(2.0f * (q.w * q.y + q.x * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    const float c = cosf(yaw), s = sinf(yaw);
    return {c * x + s * z, y, -s * x + c * z};
}

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

    // Controllers: the headset's own (WMR over its radio or Bluetooth), or
    // PlayStation Moves paired to the Mac. Indexed by Hand. Not owned here:
    // WMR devices belong to the headset struct, PS Moves to psmv[].
    struct xrt_device* controllers[2] = {nullptr, nullptr};
    struct oxrsys_psmv_controller* psmv[kMaxPsMove] = {};
    size_t psmvCount = 0;
    std::string controllerKind;
    // Sphere tracking on the headset cameras (OpenCV builds only).
    struct oxrsys_wmr_psmv_tracking* sphereTracking = nullptr;

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

    // Controllers: prefer the headset's (radio or Bluetooth WMR controllers),
    // otherwise PlayStation Moves paired to this Mac. With no left/right
    // identity, the first Move is the right hand, the second the left.
    if (headset->left != nullptr || headset->right != nullptr)
    {
        impl_->controllers[static_cast<int>(Hand::Left)] = headset->left;
        impl_->controllers[static_cast<int>(Hand::Right)] = headset->right;
        impl_->controllerKind = headset->controllers_bluetooth ? "WMR controllers (Bluetooth)"
                                                               : "WMR controllers (headset radio)";
    }
    else
    {
        // Sphere position from the headset cameras when the library has
        // OpenCV; the factory must exist before the Moves are created.
        struct xrt_tracking_factory* factory =
            oxrsys_wmr_psmv_tracking_create(headset->hmd, U_LOGGING_INFO, &impl_->sphereTracking);
        oxrsys_psmv_set_tracking_factory(factory);

        const enum oxrsys_psmv_open_result psmvResult =
            oxrsys_psmv_open_all(U_LOGGING_INFO, impl_->psmv, kMaxPsMove, &impl_->psmvCount);
        if (psmvResult == OXRSYS_PSMV_OPEN_OK)
        {
            impl_->controllers[static_cast<int>(Hand::Right)] = impl_->psmv[0]->xdev;
            if (impl_->psmvCount > 1)
            {
                impl_->controllers[static_cast<int>(Hand::Left)] = impl_->psmv[1]->xdev;
            }
            impl_->controllerKind = "PlayStation Move";
        }
        else
        {
            impl_->controllerKind = "none";
            // No Move to track: do not keep the camera pipeline running.
            oxrsys_psmv_set_tracking_factory(nullptr);
            oxrsys_wmr_psmv_tracking_destroy(&impl_->sphereTracking);
        }
    }
    spdlog::info("WiredHeadset: controllers: {} (left {}, right {})", impl_->controllerKind,
                 impl_->controllers[0] != nullptr ? "yes" : "no", impl_->controllers[1] != nullptr ? "yes" : "no");

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
    impl_->controllers[0] = nullptr;
    impl_->controllers[1] = nullptr;
    for (size_t i = 0; i < impl_->psmvCount; i++)
    {
        oxrsys_psmv_close(&impl_->psmv[i]);
    }
    impl_->psmvCount = 0;
    oxrsys_psmv_set_tracking_factory(nullptr);
    oxrsys_wmr_psmv_tracking_destroy(&impl_->sphereTracking);
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

        // Controllers: orientation from their IMU, position from the arm
        // model around the head, inputs mapped onto the Touch-style fields.
        for (int h = 0; h < 2; h++)
        {
            struct xrt_device* ctrl = controllers[h];
            if (ctrl == nullptr)
            {
                continue;
            }
            const Hand hand = static_cast<Hand>(h);
            xrt_device_update_inputs(ctrl);

            struct xrt_space_relation ctrlRelation = {};
            const xrt_result_t cret =
                xrt_device_get_tracked_pose(ctrl, GripPoseInput(ctrl), monadoNowNs + horizonNs, &ctrlRelation);
            const bool ctrlValid = cret == XRT_SUCCESS &&
                                   (ctrlRelation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
            if (!ctrlValid)
            {
                continue;
            }

            float* pos = hand == Hand::Left ? packet.leftControllerPos : packet.rightControllerPos;
            float* rot = hand == Hand::Left ? packet.leftControllerRot : packet.rightControllerRot;
            const bool positionTracked =
                (ctrlRelation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
            struct xrt_vec3 offset;
            if (positionTracked)
            {
                // Sphere position is in the headset camera frame, which rides
                // on the head: rotate it by the full head orientation.
                offset = quat_rotate(relation.pose.orientation, ctrlRelation.pose.position);
            }
            else
            {
                offset = YawRotate(relation.pose.orientation, hand == Hand::Left ? -kArmOffsetX : kArmOffsetX,
                                   kArmOffsetY, kArmOffsetZ);
            }
            pos[0] = packet.headPosition[0] + offset.x;
            pos[1] = packet.headPosition[1] + offset.y;
            pos[2] = packet.headPosition[2] + offset.z;
            rot[0] = ctrlRelation.pose.orientation.x;
            rot[1] = ctrlRelation.pose.orientation.y;
            rot[2] = ctrlRelation.pose.orientation.z;
            rot[3] = ctrlRelation.pose.orientation.w;
            packet.trackingFlags |= hand == Hand::Left ? oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE
                                                       : oxr::protocol::TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE;
            MapControllerInputs(ctrl, hand, packet);
        }

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
