// SPDX-License-Identifier: MPL-2.0

#include "Session.h"
#include "QuadLayerProjection.h"
#include "CompositionLayerAlpha.h"
#include "CompositionLayerQuad.h"
#include "Config.h"
#include "Instance.h"
#include "Runtime.h"
#include "Swapchain.h"
#include "SwapchainCreateValidation.h"
#include "Space.h"
#include "InputManager.h"
#include <mutex>
#include "HeadsetViewStore.h"
#include "StreamingServer.h"
#include "RuntimeStatus.h"
#include "WiredHeadset.h"
#include "Config.h"
#include <spdlog/spdlog.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <ctime>
#include <exception>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

#include <openxr/openxr_platform.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace
{
glm::quat ToGlmQuat(const XrQuaternionf& q) { return glm::quat(q.w, q.x, q.y, q.z); }
glm::vec3 ToGlmVec(const XrVector3f& v) { return glm::vec3(v.x, v.y, v.z); }
XrQuaternionf ToXrQuat(const glm::quat& q) { return {q.x, q.y, q.z, q.w}; }
XrVector3f ToXrVec(const glm::vec3& v) { return {v.x, v.y, v.z}; }

// World-space pose of a reference space's origin (mirrors Space.cpp GetWorldPose
// for the reference-space case, including the space's own poseInSpace offset).
XrPosef ReferenceSpaceWorldPose(Space* space, const InputManager& inputManager)
{
    XrPosef pose{};
    pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    pose.position = {0.0f, 0.0f, 0.0f};

    if (space->GetType() == Space::Type::Reference)
    {
        if (space->GetReferenceSpaceType() == XR_REFERENCE_SPACE_TYPE_VIEW)
        {
            pose = inputManager.GetHeadPose();
        }
        else
        {
            pose = inputManager.GetReferenceSpacePose(space->GetReferenceSpaceType());
        }
    }

    // Apply the space's own offset pose (poseInReferenceSpace).
    const XrPosef& offset = space->GetPoseInSpace();
    glm::quat worldRot = ToGlmQuat(pose.orientation);
    glm::vec3 worldPos = ToGlmVec(pose.position);
    glm::quat offsetRot = ToGlmQuat(offset.orientation);
    glm::vec3 offsetPos = ToGlmVec(offset.position);
    pose.orientation = ToXrQuat(worldRot * offsetRot);
    pose.position = ToXrVec(worldPos + worldRot * offsetPos);
    return pose;
}
} // namespace

namespace
{

using Clock = std::chrono::steady_clock;

// CLOCK_MONOTONIC in nanoseconds — the clock XR_KHR_convert_timespec_time
// converts against.
int64_t MonotonicNowNs()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

FramePose ToFramePose(const XrPosef& pose)
{
    FramePose result = {};
    result.orientation[0] = pose.orientation.x;
    result.orientation[1] = pose.orientation.y;
    result.orientation[2] = pose.orientation.z;
    result.orientation[3] = pose.orientation.w;
    result.position[0] = pose.position.x;
    result.position[1] = pose.position.y;
    result.position[2] = pose.position.z;
    return result;
}

FrameFov ToFrameFov(const XrFovf& fov)
{
    FrameFov result = {};
    result.angleLeft = fov.angleLeft;
    result.angleRight = fov.angleRight;
    result.angleUp = fov.angleUp;
    result.angleDown = fov.angleDown;
    return result;
}

struct SessionMetricSummary
{
    double average = 0.0;
    double p95 = 0.0;
    size_t count = 0;
};

SessionMetricSummary SummarizeSessionSamples(std::vector<double>& samples)
{
    SessionMetricSummary summary = {};
    if (samples.empty())
    {
        return summary;
    }

    std::sort(samples.begin(), samples.end());
    summary.count = samples.size();
    summary.average = std::accumulate(samples.begin(), samples.end(), 0.0) / summary.count;
    size_t p95Index = static_cast<size_t>(0.95 * (samples.size() - 1));
    summary.p95 = samples[p95Index];
    return summary;
}

bool IsSupportedEnvironmentBlendMode(XrEnvironmentBlendMode blendMode, const Instance* instance)
{
    if (blendMode == XR_ENVIRONMENT_BLEND_MODE_OPAQUE)
    {
        return true;
    }
    return blendMode == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND &&
           instance != nullptr &&
           instance->SupportsPassthroughBlendMode();
}

constexpr uint32_t kMaxSupportedCompositionLayers = XR_MIN_COMPOSITION_LAYERS_SUPPORTED;

bool IsFiniteQuaternion(const XrQuaternionf& orientation)
{
    return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

bool IsValidPose(const XrPosef& pose)
{
    if (!IsFiniteQuaternion(pose.orientation) ||
        !std::isfinite(pose.position.x) ||
        !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z))
    {
        return false;
    }

    const float magnitudeSquared =
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w;
    if (!(magnitudeSquared > 0.0f) || !std::isfinite(magnitudeSquared))
    {
        return false;
    }

    const float magnitude = std::sqrt(magnitudeSquared);
    return std::fabs(magnitude - 1.0f) <= 0.01f;
}

bool IsFiniteFov(const XrFovf& fov)
{
    return std::isfinite(fov.angleLeft) && std::isfinite(fov.angleRight) &&
           std::isfinite(fov.angleUp) && std::isfinite(fov.angleDown);
}

// The last headset view (FOV/IPD) seen in this process, else the one saved by an earlier
// run. Sessions are recreated freely (OpenComposite restarts its session to load input
// bindings), so every new session starts from the newest known view.
std::mutex g_headsetViewMutex;
oxrsys::runtime::SavedHeadsetView g_headsetView;
bool g_headsetViewLoaded = false;

oxrsys::runtime::SavedHeadsetView CurrentSavedHeadsetView()
{
    std::scoped_lock lock(g_headsetViewMutex);
    if (!g_headsetViewLoaded)
    {
        g_headsetViewLoaded = true;
        const std::string& path = Config::Get().headsetViewPath;
        if (oxrsys::runtime::LoadHeadsetView(path, g_headsetView))
        {
            spdlog::info("OXRSys: using the saved headset view until a client reports one "
                         "(IPD={:.1f}mm FOV L={:.3f} R={:.3f} U={:.3f} D={:.3f}, from {})",
                         g_headsetView.ipd * 1000.0f, g_headsetView.fov[0], g_headsetView.fov[1],
                         g_headsetView.fov[2], g_headsetView.fov[3], path);
        }
    }
    return g_headsetView;
}

void StoreHeadsetView(const oxrsys::runtime::SavedHeadsetView& view)
{
    std::scoped_lock lock(g_headsetViewMutex);
    g_headsetView = view;
    g_headsetViewLoaded = true;
    const std::string& path = Config::Get().headsetViewPath;
    if (oxrsys::runtime::SaveHeadsetView(path, view))
    {
        spdlog::info("OXRSys: saved headset view IPD={:.1f}mm FOV L={:.3f} R={:.3f} U={:.3f} D={:.3f} to {}",
                     view.ipd * 1000.0f, view.fov[0], view.fov[1], view.fov[2], view.fov[3], path);
    }
    else
    {
        spdlog::warn("OXRSys: could not save the headset view to {}", path);
    }
}

} // namespace

Session::Session(Instance* instance, void* metalDevice, void* metalCommandQueue)
    : instance_(instance), graphicsContext_(GraphicsContext::Metal(metalDevice, metalCommandQueue))
{
    inputManager_ = std::make_unique<InputManager>();
    inputManager_->SetSavedHeadsetView(CurrentSavedHeadsetView());

    startTime_ = std::chrono::steady_clock::now();
    monoStartNs_ = MonotonicNowNs();
    lastFrameTime_ = startTime_;

    Runtime::Get().RegisterHandle(handle_, this);
    instance_->SetSession(this);

    TransitionState(XR_SESSION_STATE_IDLE);
    TransitionState(XR_SESSION_STATE_READY);

    spdlog::info("OXRSys: Metal session created");
}

Session::Session(Instance* instance, const GraphicsContext& graphicsContext)
    : instance_(instance), graphicsContext_(graphicsContext)
{
    inputManager_ = std::make_unique<InputManager>();
    inputManager_->SetSavedHeadsetView(CurrentSavedHeadsetView());

    startTime_ = std::chrono::steady_clock::now();
    monoStartNs_ = MonotonicNowNs();
    lastFrameTime_ = startTime_;

    Runtime::Get().RegisterHandle(handle_, this);
    instance_->SetSession(this);

    TransitionState(XR_SESSION_STATE_IDLE);
    TransitionState(XR_SESSION_STATE_READY);

    const char* apiName = "unknown";
    switch (graphicsContext_.api)
    {
        case GraphicsApi::Metal:
            apiName = "Metal";
            break;
        case GraphicsApi::Vulkan:
            apiName = "Vulkan";
            break;
    }
    spdlog::info("OXRSys: {} session created", apiName);
}

Session::~Session()
{
    if (Shutdown() != XR_SUCCESS)
    {
        // xrDestroySession retains the Session on a bounded drain timeout.
        // Freeing it here would invalidate callbacks and app-device resources.
        std::terminate();
    }
    instance_->RemoveEventsForSession(reinterpret_cast<XrSession>(handle_));
    instance_->SetSession(nullptr);
    Runtime::Get().RemoveHandle(handle_);
    spdlog::info("OXRSys: Session destroyed");
}

XrTime Session::GetCurrentTime() const
{
    return static_cast<XrTime>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - startTime_)
            .count());
}

XrTime Session::TimespecToXrTime(const struct timespec& ts) const
{
    const int64_t monoNs = static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    // XrTime == steady_clock::now() - startTime_, and CLOCK_MONOTONIC advances in
    // lockstep with steady_clock (both real-time ns), so XrTime == monoNs - monoStartNs_.
    return static_cast<XrTime>(monoNs - monoStartNs_);
}

void Session::XrTimeToTimespec(XrTime time, struct timespec& ts) const
{
    const int64_t monoNs = static_cast<int64_t>(time) + monoStartNs_;
    ts.tv_sec = static_cast<time_t>(monoNs / 1000000000LL);
    ts.tv_nsec = static_cast<long>(monoNs % 1000000000LL);
}

void Session::TransitionState(XrSessionState newState)
{
    state_ = newState;

    XrEventDataBuffer event{};
    auto* stateChanged = reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
    stateChanged->type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED;
    stateChanged->next = nullptr;
    stateChanged->session = reinterpret_cast<XrSession>(handle_);
    stateChanged->state = newState;
    stateChanged->time = static_cast<XrTime>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - startTime_)
            .count());

    instance_->PushEvent(event);
    spdlog::info("OXRSys: Session state -> {}", static_cast<int>(newState));
}

void Session::PublishTrackingChanges()
{
    // The first streamed head pose re-anchors LOCAL (and LOCAL_FLOOR) from the provisional
    // default-head anchor to the real head. Tell the application, as OpenXR requires.
    XrPosef poseInPreviousSpace{};
    if (inputManager_->TakeLocalReferenceChange(poseInPreviousSpace))
    {
        const XrTime changeTime = static_cast<XrTime>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - startTime_)
                .count());
        std::vector<XrReferenceSpaceType> changed = {XR_REFERENCE_SPACE_TYPE_LOCAL};
        if (instance_ != nullptr && instance_->SupportsLocalFloor())
        {
            changed.push_back(XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR);
        }
        for (XrReferenceSpaceType type : changed)
        {
            XrEventDataBuffer event{};
            auto* pending = reinterpret_cast<XrEventDataReferenceSpaceChangePending*>(&event);
            pending->type = XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING;
            pending->next = nullptr;
            pending->session = reinterpret_cast<XrSession>(handle_);
            pending->referenceSpaceType = type;
            pending->changeTime = changeTime;
            pending->poseValid = XR_TRUE;
            pending->poseInPreviousSpace = poseInPreviousSpace;
            if (type == XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR)
            {
                // LOCAL_FLOOR keeps its floor height; only LOCAL's x/z and yaw moved.
                pending->poseInPreviousSpace.position.y = 0.0f;
            }
            instance_->PushEvent(event);
        }
        spdlog::info("OXRSys: LOCAL reference re-anchored to the first tracked head pose "
                     "(moved ({:.3f}, {:.3f}, {:.3f}) in the previous LOCAL space); "
                     "queued XrEventDataReferenceSpaceChangePending",
                     poseInPreviousSpace.position.x, poseInPreviousSpace.position.y,
                     poseInPreviousSpace.position.z);
    }

    oxrsys::runtime::SavedHeadsetView view;
    if (inputManager_->TakeHeadsetViewUpdate(view))
    {
        StoreHeadsetView(view);
    }
}

XrResult Session::BeginSession(const XrSessionBeginInfo* beginInfo)
{
    if (beginInfo == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (running_)
    {
        return XR_ERROR_SESSION_RUNNING;
    }
    if (state_ != XR_SESSION_STATE_READY)
    {
        return XR_ERROR_SESSION_NOT_READY;
    }
    if (!instance_->IsViewConfigurationTypeSupported(beginInfo->primaryViewConfigurationType))
    {
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }

    exitRequested_ = false;
    {
        std::scoped_lock lock(frameStateMutex_);
        frameBegun_ = false;
        waitedFrameCount_ = 0;
    }

    running_ = true;

    // Start streaming server (broadcasts on LAN, waits for headset connection)
    StartStreamingIfNeeded();

    spdlog::info("OXRSys: Session begun");
    return XR_SUCCESS;
}

XrResult Session::EndSession()
{
    if (state_ != XR_SESSION_STATE_STOPPING)
    {
        return XR_ERROR_SESSION_NOT_STOPPING;
    }

    running_ = false;

    // Stop streaming so it can be restarted on next BeginSession
    if (!StopStreaming())
    {
        return XR_ERROR_RUNTIME_FAILURE;
    }

    {
        std::scoped_lock lock(frameStateMutex_);
        frameBegun_ = false;
        waitedFrameCount_ = 0;
    }

    TransitionState(XR_SESSION_STATE_IDLE);
    TransitionState(XR_SESSION_STATE_EXITING);
    exitRequested_ = false;

    spdlog::info("OXRSys: Session ended");
    return XR_SUCCESS;
}

XrResult Session::RequestExitSession()
{
    if (!running_)
    {
        return XR_ERROR_SESSION_NOT_RUNNING;
    }

    exitRequested_ = true;
    return XR_SUCCESS;
}

XrResult Session::Shutdown()
{
    std::lock_guard<std::mutex> shutdownLock(shutdownMutex_);
    teardownStarted_.store(true, std::memory_order_release);
    running_ = false;
    exitRequested_ = true;
    state_ = XR_SESSION_STATE_IDLE;

    {
        std::scoped_lock lock(frameStateMutex_);
        frameBegun_ = false;
        waitedFrameCount_ = 0;
    }

    if (!StopStreaming())
    {
        // Keep the server, encoder, swapchains and public session handle
        // alive. xrDestroySession/Instance can retry after callbacks exit.
        return XR_ERROR_RUNTIME_FAILURE;
    }
    if (inputManager_)
    {
        inputManager_->SetTrackingReceiver(nullptr);
    }

    std::lock_guard<std::mutex> swapchainsLock(swapchainsMutex_);
    for (const auto& swapchain : swapchains_)
    {
        const XrResult result = swapchain->PrepareForDestroy();
        if (result != XR_SUCCESS)
        {
            return result;
        }
    }
    spaces_.clear();
    swapchains_.clear();
    return XR_SUCCESS;
}

// Detaches the wired headset or stops the streaming server. Returns false,
// leaving the server in place, only when the server refuses to stop (its
// callbacks are still running); callers surface that as a runtime failure so
// the session can be torn down again later.
bool Session::StopStreaming()
{
    streamingSnapshotDemand_->store(false, std::memory_order_release);
    if (wiredActive_)
    {
        WiredHeadset::Shared().DetachGraphics();
        wiredActive_ = false;
        streamingStarted_ = false;
        if (inputManager_)
        {
            inputManager_->SetTrackingReceiver(nullptr);
        }
        spdlog::info("OXRSys: Wired headset detached for session end");
    }
    if (streamingServer_)
    {
        if (!streamingServer_->Stop())
        {
            return false;
        }
        streamingServer_.reset();
        streamingStarted_ = false;
        if (inputManager_)
        {
            inputManager_->SetTrackingReceiver(nullptr);
        }
        spdlog::info("OXRSys: Streaming server stopped for session end");
    }
    return true;
}

void Session::BeginDebugUtilsLabelRegion(const XrDebugUtilsLabelEXT& labelInfo)
{
    std::scoped_lock lock(debugUtilsMutex_);

    debugUtilsInsertedLabel_.reset();
    DebugUtilsLabelState label = {};
    label.labelName = labelInfo.labelName;
    debugUtilsLabelRegions_.push_back(std::move(label));
}

void Session::EndDebugUtilsLabelRegion()
{
    std::scoped_lock lock(debugUtilsMutex_);

    if (!debugUtilsLabelRegions_.empty())
    {
        debugUtilsLabelRegions_.pop_back();
    }
    debugUtilsInsertedLabel_.reset();
}

void Session::InsertDebugUtilsLabel(const XrDebugUtilsLabelEXT& labelInfo)
{
    std::scoped_lock lock(debugUtilsMutex_);

    DebugUtilsLabelState label = {};
    label.labelName = labelInfo.labelName;
    debugUtilsInsertedLabel_ = std::move(label);
}

void Session::GetDebugUtilsLabels(std::vector<XrDebugUtilsLabelEXT>& labels, std::vector<std::string>& labelNames) const
{
    std::vector<DebugUtilsLabelState> activeLabels;
    {
        std::scoped_lock lock(debugUtilsMutex_);

        if (debugUtilsInsertedLabel_.has_value())
        {
            activeLabels.push_back(*debugUtilsInsertedLabel_);
        }
        for (auto it = debugUtilsLabelRegions_.rbegin(); it != debugUtilsLabelRegions_.rend(); ++it)
        {
            activeLabels.push_back(*it);
        }
    }

    labelNames.clear();
    labels.clear();
    labelNames.reserve(activeLabels.size());
    labels.reserve(activeLabels.size());

    for (const auto& activeLabel : activeLabels)
    {
        labelNames.push_back(activeLabel.labelName);

        XrDebugUtilsLabelEXT label = {XR_TYPE_DEBUG_UTILS_LABEL_EXT};
        label.labelName = labelNames.back().c_str();
        labels.push_back(label);
    }
}

XrResult Session::WaitFrame(const XrFrameWaitInfo* frameWaitInfo, XrFrameState* frameState)
{
    if (frameWaitInfo != nullptr && frameWaitInfo->type != XR_TYPE_FRAME_WAIT_INFO)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (frameState == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (frameState->type != XR_TYPE_FRAME_STATE)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    {
        std::scoped_lock lock(frameStateMutex_);
        if (!IsFrameLoopRunningState())
        {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
    }

    streamingSnapshotDemand_->store(
        streamingServer_ != nullptr && streamingServer_->IsClientConnected(),
        std::memory_order_release);

    uint32_t targetRefreshHz = 90;
    if (wiredActive_)
    {
        targetRefreshHz = std::max(WiredHeadset::Shared().GetRefreshRateHz(), 1u);
    }
    else if (streamingServer_)
    {
        targetRefreshHz = std::max(streamingServer_->GetTargetRefreshRateHz(), 1u);
    }

    // Frame pacing: self-correcting absolute-deadline grid at the negotiated display
    // period. The previous scheme slept for (period - elapsed) and then re-anchored
    // to the wake time, so sleep overshoot was never corrected: periods were always
    // >= the target and the phase drifted continuously against the panel (the app
    // free-ran slightly under refresh, producing a slow beat and frame-duplication
    // judder even at a good average framerate). Here each frame targets a fixed
    // absolute deadline advanced by exactly one period, so overshoot on one frame is
    // absorbed by the next and the cadence locks to the panel period with low
    // variance. targetFrameTime comes from the client-reported (negotiated) refresh.
    auto targetFrameTime = std::chrono::nanoseconds(1000000000ll / targetRefreshHz);
    auto now = std::chrono::steady_clock::now();

    if (nextFrameDeadline_.time_since_epoch().count() == 0 || pacingPeriod_ != targetFrameTime)
    {
        // First frame, or the negotiated refresh changed: (re)seed the grid.
        pacingPeriod_ = targetFrameTime;
        nextFrameDeadline_ = now + targetFrameTime;
    }
    else
    {
        if (now < nextFrameDeadline_)
        {
            std::this_thread::sleep_until(nextFrameDeadline_);
            now = std::chrono::steady_clock::now();
        }
        nextFrameDeadline_ += targetFrameTime;

        // If we fell far behind (a hitch, a stall, or the app pausing), snap the grid
        // back to the present rather than firing a burst of catch-up frames.
        if (now - nextFrameDeadline_ > 2 * targetFrameTime)
        {
            nextFrameDeadline_ = now + targetFrameTime;
        }
    }

    // Frame-pacing diagnostic: actual inter-WaitFrame interval statistics prove the
    // cadence locks to the negotiated period (e.g. ~13.9ms @72Hz, ~11.1ms @90Hz)
    // with low variance instead of free-running / drifting.
    {
        static std::vector<double> intervalSamplesMs;
        static auto lastPacingLog = Clock::now();
        auto intervalMs = std::chrono::duration<double, std::milli>(now - lastFrameTime_).count();
        if (lastFrameTime_ != startTime_)
        {
            intervalSamplesMs.push_back(intervalMs);
        }
        if (now - lastPacingLog >= std::chrono::seconds(1) && !intervalSamplesMs.empty())
        {
            double sum = std::accumulate(intervalSamplesMs.begin(), intervalSamplesMs.end(), 0.0);
            double mean = sum / intervalSamplesMs.size();
            double variance = 0.0;
            for (double s : intervalSamplesMs) { variance += (s - mean) * (s - mean); }
            variance /= intervalSamplesMs.size();
            double stddev = std::sqrt(variance);
            auto mm = std::minmax_element(intervalSamplesMs.begin(), intervalSamplesMs.end());
            spdlog::info("OXRSys: Session::WaitFrame pacing target={}Hz ({:.2f}ms) "
                         "actual mean={:.2f}ms stddev={:.2f}ms min/max={:.2f}/{:.2f}ms (n={})",
                         targetRefreshHz,
                         std::chrono::duration<double, std::milli>(targetFrameTime).count(),
                         mean, stddev, *mm.first, *mm.second, intervalSamplesMs.size());
            intervalSamplesMs.clear();
            lastPacingLog = now;
        }
    }

    auto dt = std::chrono::duration<float>(now - lastFrameTime_).count();
    lastFrameTime_ = now;

    // Update input
    inputManager_->Update(dt);
    PublishTrackingChanges();

    auto displayTime = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startTime_).count();

    frameState->type = XR_TYPE_FRAME_STATE;
    frameState->predictedDisplayTime = static_cast<XrTime>(displayTime);
    frameState->predictedDisplayPeriod = static_cast<XrDuration>(targetFrameTime.count());
    frameState->shouldRender = running_ && !exitRequested_ ? XR_TRUE : XR_FALSE;

    {
        std::scoped_lock lock(frameStateMutex_);
        if (!IsFrameLoopRunningState())
        {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        ++waitedFrameCount_;
    }

    return XR_SUCCESS;
}

XrResult Session::BeginFrame(const XrFrameBeginInfo* frameBeginInfo)
{
    if (frameBeginInfo != nullptr && frameBeginInfo->type != XR_TYPE_FRAME_BEGIN_INFO)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    std::scoped_lock lock(frameStateMutex_);
    if (!IsFrameLoopRunningState())
    {
        return XR_ERROR_SESSION_NOT_RUNNING;
    }
    if (waitedFrameCount_ == 0)
    {
        return XR_ERROR_CALL_ORDER_INVALID;
    }

    --waitedFrameCount_;
    if (frameBegun_)
    {
        return XR_FRAME_DISCARDED;
    }

    frameBegun_ = true;
    return XR_SUCCESS;
}

XrResult Session::EndFrame(const XrFrameEndInfo* frameEndInfo)
{
    {
        std::scoped_lock lock(frameStateMutex_);
        if (!IsFrameLoopRunningState())
        {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        if (!frameBegun_)
        {
            return XR_ERROR_CALL_ORDER_INVALID;
        }
    }

    if (frameEndInfo == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (frameEndInfo->type != XR_TYPE_FRAME_END_INFO)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (frameEndInfo->displayTime <= 0)
    {
        return XR_ERROR_TIME_INVALID;
    }
    if (!IsSupportedEnvironmentBlendMode(frameEndInfo->environmentBlendMode, instance_))
    {
        return XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED;
    }
    if (frameEndInfo->layerCount > kMaxSupportedCompositionLayers)
    {
        return XR_ERROR_LAYER_LIMIT_EXCEEDED;
    }
    if (frameEndInfo->layerCount > 0 && frameEndInfo->layers == nullptr)
    {
        return XR_ERROR_LAYER_INVALID;
    }

    // Extract submitted eye sources for streaming. Missing streaming sources do
    // not invalidate the OpenXR frame; they only skip this frame's video encode.
    const bool passthroughEnabled = instance_ != nullptr &&
                                    instance_->SupportsPassthroughBlendMode();
    const bool environmentAlphaBlend =
        frameEndInfo->environmentBlendMode == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
    bool sourceAlphaProjectionLayer = false;

    FrameSource frameSource = {};
    // The space the surviving projection layer's views are expressed in. Quad
    // poses are relocated into it so the compositor can project them against
    // the very views the application rendered.
    Space* projectionSpace = nullptr;
    // OpenXR composites layers in array order, so a quad submitted *before* the
    // projection layer is drawn underneath it. The projection layer is opaque
    // here, so those quads are fully occluded; collect only the ones that come
    // after it rather than wrongly drawing them on top.
    std::vector<FrameQuadLayer> pendingQuads;
    size_t quadsBeforeProjection = 0;

    for (uint32_t i = 0; i < frameEndInfo->layerCount; i++)
    {
        const XrCompositionLayerBaseHeader* layer = frameEndInfo->layers[i];
        if (layer == nullptr)
        {
            return XR_ERROR_LAYER_INVALID;
        }

        switch (layer->type)
        {
            case XR_TYPE_COMPOSITION_LAYER_PROJECTION:
            {
                const auto& projectionLayer =
                    *reinterpret_cast<const XrCompositionLayerProjection*>(layer);
                if (oxrsys::runtime::IsSourceAlphaProjectionLayerForStreaming(
                        projectionLayer.layerFlags, passthroughEnabled))
                {
                    sourceAlphaProjectionLayer = true;
                }

                XrResult result = ValidateProjectionLayer(projectionLayer, frameSource);
                if (result != XR_SUCCESS)
                {
                    return result;
                }
                projectionSpace = Runtime::Get().FromHandle<Space>(
                    reinterpret_cast<uint64_t>(projectionLayer.space));
                // A second projection layer replaces the first, and anything
                // stacked on the first is now underneath it.
                quadsBeforeProjection += pendingQuads.size();
                pendingQuads.clear();
                break;
            }
            case XR_TYPE_COMPOSITION_LAYER_QUAD:
            {
                FrameQuadLayer quad = {};
                XrResult result = ValidateQuadLayer(
                    *reinterpret_cast<const XrCompositionLayerQuad*>(layer),
                    frameEndInfo->displayTime, projectionSpace, frameSource, quad);
                if (result != XR_SUCCESS)
                {
                    return result;
                }
                if (projectionSpace == nullptr)
                {
                    // Nothing to project against yet, so this quad is either
                    // underneath a projection layer still to come or in a frame
                    // with no projection layer at all. Either way it is not
                    // drawn; ValidateQuadLayer leaves it empty.
                    ++quadsBeforeProjection;
                }
                else if (quad.IsValid())
                {
                    pendingQuads.push_back(std::move(quad));
                }
                break;
            }
            default:
                return XR_ERROR_LAYER_INVALID;
        }
    }

    frameSource.quads = std::move(pendingQuads);
    {
        // Describe the quad layers whenever the set changes (and every ~10 s while any are
        // shown), so a layer that covers the view can be identified from the log.
        static std::atomic<size_t> loggedQuadCount{static_cast<size_t>(-1)};
        static std::atomic<uint32_t> quadLogFrames{0};
        const size_t quadCount = frameSource.quads.size();
        const bool periodic = quadCount > 0 && (quadLogFrames.fetch_add(1) % 900) == 0;
        if (loggedQuadCount.exchange(quadCount) != quadCount || periodic)
        {
            spdlog::info("OXRSys: frame has {} quad layer(s) over the projection layer "
                         "({} submitted beneath it)", quadCount, quadsBeforeProjection);
            for (size_t i = 0; i < quadCount; ++i)
            {
                const FrameQuadLayer& quad = frameSource.quads[i];
                spdlog::info("OXRSys:   quad {}: {:.3f}x{:.3f} m at ({:.3f}, {:.3f}, {:.3f}) "
                             "src rect {}x{}+{}+{} of {}x{} blend={} eyes={}",
                             i, quad.widthMeters, quad.heightMeters, quad.pose.position[0],
                             quad.pose.position[1], quad.pose.position[2], quad.image.sourceWidth,
                             quad.image.sourceHeight, quad.image.sourceX, quad.image.sourceY,
                             quad.image.imageWidth, quad.image.imageHeight,
                             static_cast<int>(quad.blend), static_cast<int>(quad.eyeVisibility));
            }
        }
    }
    if (quadsBeforeProjection > 0)
    {
        static std::atomic_bool loggedOccludedQuads{false};
        if (!loggedOccludedQuads.exchange(true))
        {
            spdlog::info("OXRSys: ignoring {} quad layer(s) not stacked above a projection layer; "
                         "an opaque projection layer submitted after them hides them entirely",
                         quadsBeforeProjection);
        }
    }

    frameSource.alphaBlend = oxrsys::runtime::IsAlphaFrameForStreaming(
        frameEndInfo->environmentBlendMode, sourceAlphaProjectionLayer);

    if (!environmentAlphaBlend && sourceAlphaProjectionLayer)
    {
        static std::atomic_bool loggedSourceAlphaProjection{false};
        if (!loggedSourceAlphaProjection.exchange(true))
        {
            spdlog::info("OXRSys: using projection layer source-alpha flags for passthrough streaming");
        }
    }

    // Send to the wired headset's presenter, or the connected streaming client.
    CheckStreamingConnection();

    // The FOV the headset will display this frame with. The encoder reprojects an eye image
    // submitted with a different FOV (an app that cached an older projection) onto it.
    {
        XrView displayViews[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        inputManager_->GetEyeViews(displayViews, 2);
        for (int eye = 0; eye < 2; ++eye)
        {
            frameSource.displayFov[eye] = ToFrameFov(displayViews[eye].fov);
        }
        frameSource.displayFovValid = true;
    }
    if (wiredActive_)
    {
        WiredHeadset::Shared().SendFrame(std::move(frameSource));
    }
    else if (streamingServer_ && streamingServer_->IsClientConnected())
    {
        auto sendStart = Clock::now();
        if (lastRenderHasPose_)
        {
            const float renderHeadOrientation[4] = {
                lastRenderHeadPose_.orientation.x, lastRenderHeadPose_.orientation.y,
                lastRenderHeadPose_.orientation.z, lastRenderHeadPose_.orientation.w};
            const float renderHeadPosition[3] = {
                lastRenderHeadPose_.position.x, lastRenderHeadPose_.position.y,
                lastRenderHeadPose_.position.z};
            streamingServer_->SendFrame(std::move(frameSource), renderHeadOrientation,
                                        renderHeadPosition);
        }
        else
        {
            streamingServer_->SendFrame(std::move(frameSource));
        }
        double enqueueMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            Clock::now() - sendStart).count();

        static std::vector<double> enqueueSamples;
        static auto lastLogTime = Clock::now();
        enqueueSamples.push_back(enqueueMs);
        if (Clock::now() - lastLogTime >= std::chrono::seconds(1))
        {
            SessionMetricSummary summary = SummarizeSessionSamples(enqueueSamples);
            spdlog::info("OXRSys: Session::EndFrame streaming enqueue avg/p95 = {:.3f}/{:.3f}ms (n={})",
                          summary.average, summary.p95, summary.count);
            enqueueSamples.clear();
            lastLogTime = Clock::now();
        }
    }

    {
        std::scoped_lock lock(frameStateMutex_);
        frameBegun_ = false;
    }

    AdvanceSessionStateAfterFrameSubmission();
    return XR_SUCCESS;
}

bool Session::OwnsSwapchain(const Swapchain* swapchain) const
{
    std::lock_guard<std::mutex> lock(swapchainsMutex_);
    return std::any_of(swapchains_.begin(), swapchains_.end(),
                       [swapchain](const std::unique_ptr<Swapchain>& candidate)
                       {
                           return candidate.get() == swapchain;
                       });
}

XrResult Session::ValidateSwapchainSubImage(const XrSwapchainSubImage& subImage) const
{
    auto* swapchain = Runtime::Get().FromHandle<Swapchain>(reinterpret_cast<uint64_t>(subImage.swapchain));
    if (swapchain == nullptr || !OwnsSwapchain(swapchain))
    {
        return XR_ERROR_HANDLE_INVALID;
    }
    if (!swapchain->HasReleasedImage())
    {
        return XR_ERROR_LAYER_INVALID;
    }
    if (subImage.imageRect.offset.x < 0 || subImage.imageRect.offset.y < 0)
    {
        return XR_ERROR_SWAPCHAIN_RECT_INVALID;
    }
    if (subImage.imageRect.extent.width <= 0 || subImage.imageRect.extent.height <= 0)
    {
        return XR_ERROR_SWAPCHAIN_RECT_INVALID;
    }

    const int64_t imageRectMaxX =
        static_cast<int64_t>(subImage.imageRect.offset.x) + static_cast<int64_t>(subImage.imageRect.extent.width);
    const int64_t imageRectMaxY =
        static_cast<int64_t>(subImage.imageRect.offset.y) + static_cast<int64_t>(subImage.imageRect.extent.height);
    if (imageRectMaxX > static_cast<int64_t>(swapchain->GetWidth()) ||
        imageRectMaxY > static_cast<int64_t>(swapchain->GetHeight()))
    {
        return XR_ERROR_SWAPCHAIN_RECT_INVALID;
    }
    if (subImage.imageArrayIndex >= swapchain->GetArraySize())
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    return XR_SUCCESS;
}

XrResult Session::ValidateProjectionLayer(const XrCompositionLayerProjection& layer,
                                          FrameSource& frameSource) const
{
    auto* space = Runtime::Get().FromHandle<Space>(reinterpret_cast<uint64_t>(layer.space));
    if (space == nullptr || space->GetSession() != this)
    {
        return XR_ERROR_HANDLE_INVALID;
    }
    if (layer.viewCount != 2 || layer.views == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    for (uint32_t viewIndex = 0; viewIndex < layer.viewCount; ++viewIndex)
    {
        const XrCompositionLayerProjectionView& view = layer.views[viewIndex];
        if (!IsValidPose(view.pose))
        {
            return XR_ERROR_POSE_INVALID;
        }
        if (!IsFiniteFov(view.fov))
        {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        XrResult subImageResult = ValidateSwapchainSubImage(view.subImage);
        if (subImageResult != XR_SUCCESS)
        {
            return subImageResult;
        }

        auto* swapchain = Runtime::Get().FromHandle<Swapchain>(reinterpret_cast<uint64_t>(view.subImage.swapchain));
        FrameImageSource imageSource =
            swapchain->GetLastReleasedFrameImageSource(view.subImage.imageArrayIndex);
        // Honor the per-view sub-rectangle: UE packs both eyes into one swapchain side-by-side.
        imageSource.sourceX = static_cast<uint32_t>(view.subImage.imageRect.offset.x);
        imageSource.sourceY = static_cast<uint32_t>(view.subImage.imageRect.offset.y);
        imageSource.sourceWidth = static_cast<uint32_t>(view.subImage.imageRect.extent.width);
        imageSource.sourceHeight = static_cast<uint32_t>(view.subImage.imageRect.extent.height);
        // The compositor needs the exact pose and frustum the eye was rendered
        // with to place quad layers in the same image.
        FrameEyeView& eyeView = frameSource.views[viewIndex == 0 ? 0 : 1];
        eyeView.pose = ToFramePose(view.pose);
        eyeView.fov = ToFrameFov(view.fov);
        eyeView.valid = true;

        if (viewIndex == 0)
        {
            frameSource.left = std::move(imageSource);
        }
        else
        {
            frameSource.right = std::move(imageSource);
        }
    }

    return XR_SUCCESS;
}

bool Session::RelocateQuadPose(Space* quadSpace, const XrPosef& quadPose, Space* projectionSpace,
                               XrTime displayTime, const FrameSource& frameSource,
                               FramePose& outPose)
{
    if (quadSpace == nullptr || projectionSpace == nullptr)
    {
        return false;
    }
    if (quadSpace == projectionSpace)
    {
        outPose = ToFramePose(quadPose);
        return true;
    }

    // Head-locked quads are pinned to the *submitted* view poses rather than
    // located through the tracker, so they share an instant with the eye images
    // they sit on. Going through LocateSpace here would use a head pose one
    // prediction newer than the pixels and the quad would swim.
    if (quadSpace->GetType() == Space::Type::Reference &&
        quadSpace->GetReferenceSpaceType() == XR_REFERENCE_SPACE_TYPE_VIEW)
    {
        FramePose renderViewPose = {};
        if (oxrsys::quad::MakeRenderViewPose(frameSource.views, renderViewPose))
        {
            const FramePose spaceOffset = ToFramePose(quadSpace->GetPoseInSpace());
            outPose = oxrsys::quad::ComposePose(
                oxrsys::quad::ComposePose(renderViewPose, spaceOffset), ToFramePose(quadPose));
            return true;
        }
    }

    XrSpaceLocation location = {};
    location.type = XR_TYPE_SPACE_LOCATION;
    if (quadSpace->LocateSpace(projectionSpace, displayTime, &location) != XR_SUCCESS)
    {
        return false;
    }
    const XrSpaceLocationFlags required =
        XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    if ((location.locationFlags & required) != required)
    {
        return false;
    }

    // Compose the layer's own offset pose on top of where its space sits.
    const glm::quat spaceRotation(location.pose.orientation.w, location.pose.orientation.x,
                                  location.pose.orientation.y, location.pose.orientation.z);
    const glm::vec3 spacePosition(location.pose.position.x, location.pose.position.y,
                                  location.pose.position.z);
    const glm::quat layerRotation(quadPose.orientation.w, quadPose.orientation.x,
                                  quadPose.orientation.y, quadPose.orientation.z);
    const glm::vec3 layerPosition(quadPose.position.x, quadPose.position.y, quadPose.position.z);

    const glm::quat finalRotation = spaceRotation * layerRotation;
    const glm::vec3 finalPosition = spacePosition + spaceRotation * layerPosition;

    outPose.orientation[0] = finalRotation.x;
    outPose.orientation[1] = finalRotation.y;
    outPose.orientation[2] = finalRotation.z;
    outPose.orientation[3] = finalRotation.w;
    outPose.position[0] = finalPosition.x;
    outPose.position[1] = finalPosition.y;
    outPose.position[2] = finalPosition.z;
    return true;
}

XrResult Session::ValidateQuadLayer(const XrCompositionLayerQuad& layer, XrTime displayTime,
                                    Space* projectionSpace, const FrameSource& frameSource,
                                    FrameQuadLayer& outQuad) const
{
    auto* space = Runtime::Get().FromHandle<Space>(reinterpret_cast<uint64_t>(layer.space));
    if (space == nullptr || space->GetSession() != this)
    {
        return XR_ERROR_HANDLE_INVALID;
    }
    if (!IsValidPose(layer.pose))
    {
        return XR_ERROR_POSE_INVALID;
    }
    if (!std::isfinite(layer.size.width) || !std::isfinite(layer.size.height) ||
        layer.size.width < 0.0f || layer.size.height < 0.0f)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    XrResult subImageResult = ValidateSwapchainSubImage(layer.subImage);
    if (subImageResult != XR_SUCCESS)
    {
        return subImageResult;
    }

    // Validation passed; everything below only decides whether this frame can
    // actually draw the quad. A quad that cannot be composited is dropped
    // silently rather than failing the application's xrEndFrame.
    if (projectionSpace == nullptr || layer.size.width <= 0.0f || layer.size.height <= 0.0f)
    {
        return XR_SUCCESS;
    }

    FramePose quadPose = {};
    if (!RelocateQuadPose(space, layer.pose, projectionSpace, displayTime, frameSource, quadPose))
    {
        return XR_SUCCESS;
    }

    auto* swapchain =
        Runtime::Get().FromHandle<Swapchain>(reinterpret_cast<uint64_t>(layer.subImage.swapchain));
    FrameImageSource imageSource =
        swapchain->GetLastReleasedFrameImageSource(layer.subImage.imageArrayIndex);
    if (!imageSource.IsValid())
    {
        return XR_SUCCESS;
    }
    // Sample only the sub-rectangle the application submitted, not the whole
    // swapchain image.
    imageSource.sourceX = static_cast<uint32_t>(layer.subImage.imageRect.offset.x);
    imageSource.sourceY = static_cast<uint32_t>(layer.subImage.imageRect.offset.y);
    imageSource.sourceWidth = static_cast<uint32_t>(layer.subImage.imageRect.extent.width);
    imageSource.sourceHeight = static_cast<uint32_t>(layer.subImage.imageRect.extent.height);
    imageSource.imageWidth = swapchain->GetWidth();
    imageSource.imageHeight = swapchain->GetHeight();

    outQuad.image = std::move(imageSource);
    outQuad.pose = quadPose;
    outQuad.widthMeters = layer.size.width;
    outQuad.heightMeters = layer.size.height;
    outQuad.eyeVisibility = oxrsys::runtime::ToFrameEyeVisibility(layer.eyeVisibility);
    outQuad.blend = oxrsys::runtime::ToFrameQuadBlend(layer.layerFlags);
    return XR_SUCCESS;
}

bool Session::IsFrameLoopRunningState() const
{
    return running_ && state_ != XR_SESSION_STATE_STOPPING;
}

XrResult Session::LocateViews(const XrViewLocateInfo* viewLocateInfo, XrViewState* viewState,
                               uint32_t viewCapacityInput, uint32_t* viewCountOutput, XrView* views)
{
    if (viewLocateInfo == nullptr || viewState == nullptr || viewCountOutput == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (viewLocateInfo->type != XR_TYPE_VIEW_LOCATE_INFO || viewState->type != XR_TYPE_VIEW_STATE)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (viewLocateInfo->displayTime <= 0)
    {
        return XR_ERROR_TIME_INVALID;
    }
    if (viewLocateInfo->viewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
    {
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }
    auto* baseSpace = Runtime::Get().FromHandle<Space>(reinterpret_cast<uint64_t>(viewLocateInfo->space));
    if (baseSpace == nullptr || baseSpace->GetSession() != this)
    {
        return XR_ERROR_HANDLE_INVALID;
    }

    *viewCountOutput = 2;

    // Views follow the head pose, so they are only TRACKED while head tracking is live.
    // The pose stays VALID either way (apps rely on a usable pose existing), matching
    // xrLocateSpace on the VIEW reference space.
    viewState->type = XR_TYPE_VIEW_STATE;
    viewState->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;
    if (inputManager_->IsHeadPoseTracked())
    {
        viewState->viewStateFlags |=
            XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT;
    }

    if (viewCapacityInput == 0)
    {
        return XR_SUCCESS;
    }
    if (views == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (viewCapacityInput < 2)
    {
        return XR_ERROR_SIZE_INSUFFICIENT;
    }

    // GetEyeViews returns the eye poses in absolute (STAGE-floor) world space.
    // Express them relative to the requested base reference space so the game
    // renders at the correct height for its chosen origin. Previously baseSpace
    // was ignored here, so views were always STAGE-absolute regardless of whether
    // the game asked for LOCAL/LOCAL_FLOOR/STAGE, and the STAGE floor-calibration
    // offset never reached the rendered image. For a STAGE base with the default
    // zero offset the transform is identity, so STAGE behaviour is unchanged.
    inputManager_->GetEyeViews(views, 2);

    // Express the STAGE-absolute eye poses relative to the requested base space
    // (identity for a STAGE base at the default zero offset).
    const XrPosef basePose = ReferenceSpaceWorldPose(baseSpace, *inputManager_);
    const glm::quat baseRotInv = glm::inverse(ToGlmQuat(basePose.orientation));
    const glm::vec3 basePos = ToGlmVec(basePose.position);
    for (uint32_t i = 0; i < 2; ++i)
    {
        glm::quat viewRot = ToGlmQuat(views[i].pose.orientation);
        glm::vec3 viewPos = ToGlmVec(views[i].pose.position);
        views[i].pose.orientation = ToXrQuat(baseRotInv * viewRot);
        views[i].pose.position = ToXrVec(baseRotInv * (viewPos - basePos));
    }

    // Remember the exact head pose this frame is being rendered for, so the streamed frame can be
    // tagged with it at submission instead of a later re-prediction.
    lastRenderHeadPose_ = inputManager_->GetHeadPose();
    lastRenderHasPose_ = true;

    return XR_SUCCESS;
}

void Session::AdvanceSessionStateAfterFrameSubmission()
{
    if (!running_)
    {
        return;
    }

    if (exitRequested_)
    {
        switch (state_)
        {
            case XR_SESSION_STATE_FOCUSED:
                TransitionState(XR_SESSION_STATE_VISIBLE);
                break;

            case XR_SESSION_STATE_VISIBLE:
                TransitionState(XR_SESSION_STATE_SYNCHRONIZED);
                break;

            case XR_SESSION_STATE_READY:
                TransitionState(XR_SESSION_STATE_SYNCHRONIZED);
                break;

            case XR_SESSION_STATE_SYNCHRONIZED:
                TransitionState(XR_SESSION_STATE_STOPPING);
                break;

            default:
                break;
        }
        return;
    }

    switch (state_)
    {
        case XR_SESSION_STATE_READY:
            TransitionState(XR_SESSION_STATE_SYNCHRONIZED);
            break;

        case XR_SESSION_STATE_SYNCHRONIZED:
            TransitionState(XR_SESSION_STATE_VISIBLE);
            break;

        case XR_SESSION_STATE_VISIBLE:
            TransitionState(XR_SESSION_STATE_FOCUSED);
            break;

        default:
            break;
    }
}

XrResult Session::CreateSwapchain(const XrSwapchainCreateInfo* createInfo, XrSwapchain* swapchain)
{
    if (createInfo == nullptr || swapchain == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (createInfo->type != XR_TYPE_SWAPCHAIN_CREATE_INFO ||
        createInfo->width == 0 ||
        createInfo->height == 0 ||
        createInfo->faceCount != 1 ||
        createInfo->arraySize == 0 ||
        createInfo->sampleCount != 1)
    {
        *swapchain = XR_NULL_HANDLE;
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (createInfo->mipCount != 1)
    {
        *swapchain = XR_NULL_HANDLE;
        return XR_ERROR_FEATURE_UNSUPPORTED;
    }

    if (teardownStarted_.load(std::memory_order_acquire))
    {
        *swapchain = XR_NULL_HANDLE;
        return XR_ERROR_SESSION_LOST;
    }

    const XrResult supportResult =
        oxrsys::swapchain::ValidateCreateInfo(graphicsContext_.api, *createInfo);
    if (supportResult != XR_SUCCESS)
    {
        *swapchain = XR_NULL_HANDLE;
        return supportResult;
    }

    *swapchain = XR_NULL_HANDLE;
    auto sc = std::make_unique<Swapchain>(graphicsContext_, createInfo);
    XrResult initializationResult = sc->InitializationResult();
    if (initializationResult != XR_SUCCESS)
    {
        return initializationResult;
    }
    sc->SetStreamingSnapshotDemand(streamingSnapshotDemand_);
    *swapchain = reinterpret_cast<XrSwapchain>(sc->GetHandle());
    {
        std::lock_guard<std::mutex> lock(swapchainsMutex_);
        if (teardownStarted_.load(std::memory_order_acquire))
        {
            *swapchain = XR_NULL_HANDLE;
            return XR_ERROR_SESSION_LOST;
        }
        swapchains_.push_back(std::move(sc));
    }
    return XR_SUCCESS;
}

XrResult Session::DestroySwapchain(Swapchain* swapchain)
{
    std::lock_guard<std::mutex> lock(swapchainsMutex_);
    for (auto it = swapchains_.begin(); it != swapchains_.end(); ++it)
    {
        if (it->get() == swapchain)
        {
            const XrResult result = (*it)->PrepareForDestroy();
            if (result != XR_SUCCESS)
            {
                return result;
            }
            swapchains_.erase(it);
            return XR_SUCCESS;
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult Session::CreateReferenceSpace(const XrReferenceSpaceCreateInfo* createInfo, XrSpace* space)
{
    if (createInfo == nullptr || space == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }
    if (!IsValidPose(createInfo->poseInReferenceSpace))
    {
        return XR_ERROR_POSE_INVALID;
    }

    switch (createInfo->referenceSpaceType)
    {
        case XR_REFERENCE_SPACE_TYPE_VIEW:
        case XR_REFERENCE_SPACE_TYPE_LOCAL:
        case XR_REFERENCE_SPACE_TYPE_STAGE:
            break;
        case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
            if (!instance_->SupportsLocalFloor())
            {
                return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
            }
            break;
        default:
            return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
    }

    const char* spaceName =
        createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_VIEW ? "VIEW" :
        createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL ? "LOCAL" :
        createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR ? "LOCAL_FLOOR" :
        createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_STAGE ? "STAGE" : "UNKNOWN";
    spdlog::info("Session: game created reference space {} (offsetInSpace y={:.3f}), "
                 "stage_height_offset_m={:.3f}",
                 spaceName, createInfo->poseInReferenceSpace.position.y,
                 Config::Get().GetValues().stageHeightOffsetM);

    auto sp = std::make_unique<Space>(this, Space::Type::Reference,
                                       createInfo->referenceSpaceType, createInfo->poseInReferenceSpace);
    *space = reinterpret_cast<XrSpace>(sp->GetHandle());
    spaces_.push_back(std::move(sp));
    return XR_SUCCESS;
}

XrResult Session::CreateActionSpace(XrAction action, XrPath subactionPath, const XrPosef& poseInSpace,
                                     XrSpace* space)
{
    if (space == nullptr)
    {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    auto sp = std::make_unique<Space>(this, action, subactionPath, poseInSpace);
    *space = reinterpret_cast<XrSpace>(sp->GetHandle());
    spaces_.push_back(std::move(sp));
    return XR_SUCCESS;
}

XrResult Session::DestroySpace(Space* space)
{
    for (auto it = spaces_.begin(); it != spaces_.end(); ++it)
    {
        if (it->get() == space)
        {
            spaces_.erase(it);
            return XR_SUCCESS;
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

void Session::StartStreamingIfNeeded()
{
    if (streamingStarted_)
    {
        return;
    }

    // A wired headset opened at xrGetSystem takes the place of the streaming
    // server: tracking is already flowing, only the frame path needs attaching.
    WiredHeadset& wired = WiredHeadset::Shared();
    if (wired.IsOpen())
    {
        if (wired.AttachGraphics(graphicsContext_))
        {
            wiredActive_ = true;
            streamingStarted_ = true;
            spdlog::info("OXRSys: Presenting on wired headset '{}' at {} Hz", wired.GetName(),
                         wired.GetRefreshRateHz());
            return;
        }
        spdlog::warn("OXRSys: Wired headset is open but could not attach the session's graphics; "
                     "falling back to streaming");
    }

    streamingServer_ = std::make_unique<StreamingServer>();
    streamingServer_->SetGraphicsContext(graphicsContext_);

    // Per-eye render resolution for the configured render_device — this must match the
    // recommendedImageRect the app renders into (see RenderBaseEyeResolution), otherwise the
    // encoder is sized off a stale default and the stream stays at that resolution.
    uint32_t width = 0;
    uint32_t height = 0;
    RenderBaseEyeResolution(width, height);
    uint32_t refreshHz = 90;

    if (streamingServer_->Start(width, height, refreshHz))
    {
        streamingStarted_ = true;
        inputManager_->SetAwaitingStreamingClient(true);
        spdlog::info("OXRSys: Streaming server started, waiting for headset connection...");
    }
    else
    {
        spdlog::warn("OXRSys: Failed to start streaming server (non-fatal, simulator mode only)");
        streamingServer_.reset();
    }
}

void Session::CheckStreamingConnection()
{
    if (wiredActive_)
    {
        WiredHeadset& wired = WiredHeadset::Shared();
        if (!inputManager_->IsStreaming() && wired.GetTrackingReceiver() != nullptr)
        {
            inputManager_->SetTrackingReceiver(wired.GetTrackingReceiver());
            inputManager_->SetStreamingClientName(wired.GetName());
            RuntimeStatus::SetStreaming("wired", wired.GetName());
            spdlog::info("OXRSys: Wired headset '{}' tracking attached", wired.GetName());
        }
        return;
    }

    if (!streamingServer_)
    {
        streamingSnapshotDemand_->store(false, std::memory_order_release);
        return;
    }

    streamingSnapshotDemand_->store(
        streamingServer_->IsClientConnected(), std::memory_order_release);

    // When a client connects, wire up the tracking receiver
    if (streamingServer_->IsClientConnected() && !inputManager_->IsStreaming())
    {
        std::string clientName = streamingServer_->GetClientName();
        inputManager_->SetTrackingReceiver(streamingServer_->GetTrackingReceiver());
        inputManager_->SetStreamingClientName(clientName);
        spdlog::info("OXRSys: Client connected ({}), receiving tracking",
                      clientName);
    }

    // When client disconnects, clear the tracking receiver
    if (!streamingServer_->IsClientConnected() && inputManager_->IsStreaming())
    {
        inputManager_->SetTrackingReceiver(nullptr);
        spdlog::info("OXRSys: Client disconnected");
    }
}
