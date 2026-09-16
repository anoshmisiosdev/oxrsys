// SPDX-License-Identifier: MPL-2.0

#include "OxrClient.h"

#include "DriverLog.h"

#include <openxr/openxr_loader_negotiation.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

namespace oxrsys
{

namespace
{

constexpr const char* kBridgeModule = "wineopenxr.dll";
constexpr uint32_t kMaxSwapchainImages = 8;

// Reports the runtime manifest the OpenXR loader will use, or an empty string.
//
// The loader that resolves this runs on the macOS side of the bridge, so it
// reads the host process environment. Wine copies that environment into the
// Windows one at process start, which is why reading it here is meaningful --
// but only for reading. SetEnvironmentVariableA writes to Wine's copy alone and
// does not reach the host loader: a probe that set the variable and then called
// xrCreateInstance still got XR_ERROR_RUNTIME_UNAVAILABLE. The driver therefore
// reports the situation rather than pretending it can fix it from in here.
//
// On macOS the loader has no working fallback either. Both
// /etc/xdg/openxr/1/active_runtime.json and ~/.config/openxr/1/active_runtime.json
// can exist and point at the runtime and the loader still fails with "failed to
// determine active runtime file path for this environment"; those search paths
// are Linux-only. XR_RUNTIME_JSON is the only mechanism.
std::string RuntimeManifestPath()
{
    char buffer[1024] = {};
    const DWORD length = GetEnvironmentVariableA("XR_RUNTIME_JSON", buffer, sizeof(buffer));
    if (length == 0 || length >= sizeof(buffer))
    {
        return {};
    }
    return std::string(buffer, length);
}

} // namespace

OxrClient::OxrClient() = default;

OxrClient::~OxrClient()
{
    Stop();
}

bool OxrClient::NegotiateBridge()
{
    runtimeManifestPath_ = RuntimeManifestPath();
    if (runtimeManifestPath_.empty())
    {
        OXRSYS_LOG("[oxrsys] XR_RUNTIME_JSON is not set in this process; the OpenXR loader on the macOS "
                   "side has no other way to find the runtime. Set it for the whole bottle by adding "
                   "\"XR_RUNTIME_JSON\" = \"/path/to/oxrsys-runtime.json\" to [EnvironmentVariables] in "
                   "the bottle's cxbottle.conf, then restart Steam so vrserver inherits it.");
    }
    else
    {
        OXRSYS_LOG("[oxrsys] runtime manifest: %s", runtimeManifestPath_.c_str());
    }

    HMODULE bridge = LoadLibraryA(kBridgeModule);
    if (bridge == nullptr)
    {
        OXRSYS_LOG("[oxrsys] LoadLibrary(%s) failed: %lu", kBridgeModule, GetLastError());
        return false;
    }

    auto negotiate = reinterpret_cast<PFN_xrNegotiateLoaderRuntimeInterface>(
        GetProcAddress(bridge, "xrNegotiateLoaderRuntimeInterface"));
    if (negotiate == nullptr)
    {
        OXRSYS_LOG("[oxrsys] %s has no xrNegotiateLoaderRuntimeInterface", kBridgeModule);
        return false;
    }

    XrNegotiateLoaderInfo loaderInfo = {};
    loaderInfo.structType = XR_LOADER_INTERFACE_STRUCT_LOADER_INFO;
    loaderInfo.structVersion = XR_LOADER_INFO_STRUCT_VERSION;
    loaderInfo.structSize = sizeof(loaderInfo);
    loaderInfo.minInterfaceVersion = 1;
    loaderInfo.maxInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    loaderInfo.minApiVersion = XR_MAKE_VERSION(1, 0, 0);
    loaderInfo.maxApiVersion = XR_MAKE_VERSION(1, 0, 999);

    XrNegotiateRuntimeRequest request = {};
    request.structType = XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST;
    request.structVersion = XR_RUNTIME_INFO_STRUCT_VERSION;
    request.structSize = sizeof(request);

    const XrResult result = negotiate(&loaderInfo, &request);
    if (result != XR_SUCCESS || request.getInstanceProcAddr == nullptr)
    {
        OXRSYS_LOG("[oxrsys] bridge negotiation failed: %d", static_cast<int>(result));
        return false;
    }

    getInstanceProcAddr_ = request.getInstanceProcAddr;
    return true;
}

bool OxrClient::CreateInstanceAndSystem()
{
    auto createInstance = GetProc<PFN_xrCreateInstance>("xrCreateInstance");
    if (createInstance == nullptr)
    {
        OXRSYS_LOG("[oxrsys] no xrCreateInstance");
        return false;
    }

    const char* extension = XR_KHR_D3D11_ENABLE_EXTENSION_NAME;

    XrInstanceCreateInfo createInfo = {XR_TYPE_INSTANCE_CREATE_INFO};
    createInfo.enabledExtensionCount = 1;
    createInfo.enabledExtensionNames = &extension;
    createInfo.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 34);
    std::snprintf(createInfo.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "OXRSys SteamVR driver");

    XrResult result = createInstance(&createInfo, &instance_);
    if (result != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] xrCreateInstance failed: %d (XR_RUNTIME_JSON=%s)",
                   static_cast<int>(result),
                   runtimeManifestPath_.empty() ? "<unset>" : runtimeManifestPath_.c_str());
        instance_ = XR_NULL_HANDLE;
        return false;
    }

    XrSystemGetInfo systemInfo = {XR_TYPE_SYSTEM_GET_INFO};
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    result = GetProc<PFN_xrGetSystem>("xrGetSystem")(instance_, &systemInfo, &systemId_);
    if (result != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] xrGetSystem failed: %d", static_cast<int>(result));
        return false;
    }

    return true;
}

bool OxrClient::CreateSessionAndSwapchains(ID3D11Device* device)
{
    // The bridge requires the graphics requirements call before session
    // creation, even though the values themselves are advisory here.
    XrGraphicsRequirementsD3D11KHR requirements = {XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    auto getRequirements = GetProc<PFN_xrGetD3D11GraphicsRequirementsKHR>("xrGetD3D11GraphicsRequirementsKHR");
    if (getRequirements == nullptr || getRequirements(instance_, systemId_, &requirements) != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] xrGetD3D11GraphicsRequirementsKHR failed");
        return false;
    }

    XrGraphicsBindingD3D11KHR binding = {XR_TYPE_GRAPHICS_BINDING_D3D11_KHR};
    binding.device = device;

    XrSessionCreateInfo sessionInfo = {XR_TYPE_SESSION_CREATE_INFO};
    sessionInfo.next = &binding;
    sessionInfo.systemId = systemId_;

    XrResult result = GetProc<PFN_xrCreateSession>("xrCreateSession")(instance_, &sessionInfo, &session_);
    if (result != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] xrCreateSession failed: %d", static_cast<int>(result));
        session_ = XR_NULL_HANDLE;
        return false;
    }

    auto createReferenceSpace = GetProc<PFN_xrCreateReferenceSpace>("xrCreateReferenceSpace");

    XrReferenceSpaceCreateInfo spaceInfo = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    spaceInfo.poseInReferenceSpace.orientation.w = 1.0f;

    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    if (createReferenceSpace(session_, &spaceInfo, &localSpace_) != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] could not create the LOCAL reference space");
        return false;
    }

    spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    if (createReferenceSpace(session_, &spaceInfo, &viewSpace_) != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] could not create the VIEW reference space");
        return false;
    }

    uint32_t viewCount = 0;
    auto enumerateViews = GetProc<PFN_xrEnumerateViewConfigurationViews>("xrEnumerateViewConfigurationViews");
    enumerateViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &viewCount, nullptr);

    XrViewConfigurationView views[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    if (viewCount != 2 ||
        enumerateViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &viewCount, views) != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] unexpected view configuration (%u views)", viewCount);
        return false;
    }

    // SteamVR renders into textures this driver allocated, so their format is
    // known: prefer the matching runtime format so the per-frame copy is a
    // straight blit rather than a conversion.
    int64_t formats[32] = {};
    uint32_t formatCount = 0;
    GetProc<PFN_xrEnumerateSwapchainFormats>("xrEnumerateSwapchainFormats")(session_, 32, &formatCount, formats);
    if (formatCount == 0)
    {
        OXRSYS_LOG("[oxrsys] runtime reported no swapchain formats");
        return false;
    }

    int64_t chosenFormat = formats[0];
    for (uint32_t i = 0; i < formatCount; ++i)
    {
        if (formats[i] == static_cast<int64_t>(DXGI_FORMAT_R8G8B8A8_UNORM_SRGB))
        {
            chosenFormat = formats[i];
            break;
        }
    }
    swapchainFormat_ = static_cast<DXGI_FORMAT>(chosenFormat);

    for (size_t eye = 0; eye < eyes_.size(); ++eye)
    {
        XrSwapchainCreateInfo swapchainInfo = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
        swapchainInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
        swapchainInfo.format = chosenFormat;
        swapchainInfo.sampleCount = 1;
        swapchainInfo.width = views[eye].recommendedImageRectWidth;
        swapchainInfo.height = views[eye].recommendedImageRectHeight;
        swapchainInfo.faceCount = 1;
        swapchainInfo.arraySize = 1;
        swapchainInfo.mipCount = 1;

        if (GetProc<PFN_xrCreateSwapchain>("xrCreateSwapchain")(session_, &swapchainInfo, &eyes_[eye].swapchain) != XR_SUCCESS)
        {
            OXRSYS_LOG("[oxrsys] xrCreateSwapchain failed for eye %zu", eye);
            return false;
        }

        eyes_[eye].width = swapchainInfo.width;
        eyes_[eye].height = swapchainInfo.height;

        XrSwapchainImageD3D11KHR images[kMaxSwapchainImages];
        for (XrSwapchainImageD3D11KHR& image : images)
        {
            image = {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR};
        }

        uint32_t imageCount = 0;
        if (GetProc<PFN_xrEnumerateSwapchainImages>("xrEnumerateSwapchainImages")(
                eyes_[eye].swapchain,
                kMaxSwapchainImages,
                &imageCount,
                reinterpret_cast<XrSwapchainImageBaseHeader*>(images)) != XR_SUCCESS)
        {
            OXRSYS_LOG("[oxrsys] xrEnumerateSwapchainImages failed for eye %zu", eye);
            return false;
        }

        eyes_[eye].images.clear();
        eyes_[eye].images.reserve(imageCount);
        for (uint32_t i = 0; i < imageCount; ++i)
        {
            eyes_[eye].images.push_back(images[i].texture);
        }
    }

    OXRSYS_LOG("[oxrsys] OpenXR session up: %ux%u per eye, format %u, %zu images per eye",
               eyes_[0].width,
               eyes_[0].height,
               static_cast<unsigned>(swapchainFormat_),
               eyes_[0].images.size());
    return true;
}

bool OxrClient::WaitForSessionReady()
{
    pollEvent_ = GetProc<PFN_xrPollEvent>("xrPollEvent");
    auto beginSession = GetProc<PFN_xrBeginSession>("xrBeginSession");
    if (pollEvent_ == nullptr || beginSession == nullptr)
    {
        return false;
    }

    XrSessionState state = XR_SESSION_STATE_UNKNOWN;
    for (int attempt = 0; attempt < 100 && state < XR_SESSION_STATE_READY; ++attempt)
    {
        XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
        while (pollEvent_(instance_, &event) == XR_SUCCESS)
        {
            if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED)
            {
                state = reinterpret_cast<XrEventDataSessionStateChanged*>(&event)->state;
            }
            event = {XR_TYPE_EVENT_DATA_BUFFER};
        }

        if (state < XR_SESSION_STATE_READY)
        {
            Sleep(20);
        }
    }

    if (state < XR_SESSION_STATE_READY)
    {
        OXRSYS_LOG("[oxrsys] session never became ready (state %d)", static_cast<int>(state));
        return false;
    }

    XrSessionBeginInfo beginInfo = {XR_TYPE_SESSION_BEGIN_INFO};
    beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

    const XrResult result = beginSession(session_, &beginInfo);
    if (result != XR_SUCCESS)
    {
        OXRSYS_LOG("[oxrsys] xrBeginSession failed: %d", static_cast<int>(result));
        return false;
    }

    return true;
}

void OxrClient::LoadFrameFunctions()
{
    waitFrame_ = GetProc<PFN_xrWaitFrame>("xrWaitFrame");
    beginFrame_ = GetProc<PFN_xrBeginFrame>("xrBeginFrame");
    endFrame_ = GetProc<PFN_xrEndFrame>("xrEndFrame");
    locateViews_ = GetProc<PFN_xrLocateViews>("xrLocateViews");
    locateSpace_ = GetProc<PFN_xrLocateSpace>("xrLocateSpace");
    acquireSwapchainImage_ = GetProc<PFN_xrAcquireSwapchainImage>("xrAcquireSwapchainImage");
    waitSwapchainImage_ = GetProc<PFN_xrWaitSwapchainImage>("xrWaitSwapchainImage");
    releaseSwapchainImage_ = GetProc<PFN_xrReleaseSwapchainImage>("xrReleaseSwapchainImage");
}

bool OxrClient::Start(ID3D11Device* device, ID3D11DeviceContext* context)
{
    if (running_.load(std::memory_order_acquire))
    {
        return true;
    }

    if (startFailed_ || device == nullptr || context == nullptr)
    {
        return false;
    }

    context_ = context;

    if (!NegotiateBridge() || !CreateInstanceAndSystem() || !CreateSessionAndSwapchains(device) || !WaitForSessionReady())
    {
        OXRSYS_LOG("[oxrsys] OpenXR client unavailable; the HMD stays tracked with a static pose");
        startFailed_ = true;
        Stop();
        return false;
    }

    LoadFrameFunctions();
    if (waitFrame_ == nullptr || beginFrame_ == nullptr || endFrame_ == nullptr)
    {
        OXRSYS_LOG("[oxrsys] runtime is missing frame entry points");
        startFailed_ = true;
        Stop();
        return false;
    }

    running_.store(true, std::memory_order_release);
    frameThread_ = std::thread([this] { FrameLoop(); });

    OXRSYS_LOG("[oxrsys] OpenXR client running; frames now go to the runtime");
    return true;
}

void OxrClient::Stop()
{
    running_.store(false, std::memory_order_release);

    if (frameThread_.joinable())
    {
        frameThread_.join();
    }

    if (instance_ == XR_NULL_HANDLE)
    {
        return;
    }

    for (Eye& eye : eyes_)
    {
        if (eye.swapchain != XR_NULL_HANDLE)
        {
            auto destroySwapchain = GetProc<PFN_xrDestroySwapchain>("xrDestroySwapchain");
            if (destroySwapchain != nullptr)
            {
                destroySwapchain(eye.swapchain);
            }
            eye.swapchain = XR_NULL_HANDLE;
        }
        eye.images.clear();
    }

    auto destroySpace = GetProc<PFN_xrDestroySpace>("xrDestroySpace");
    for (XrSpace* space : {&viewSpace_, &localSpace_})
    {
        if (*space != XR_NULL_HANDLE && destroySpace != nullptr)
        {
            destroySpace(*space);
        }
        *space = XR_NULL_HANDLE;
    }

    if (session_ != XR_NULL_HANDLE)
    {
        auto destroySession = GetProc<PFN_xrDestroySession>("xrDestroySession");
        if (destroySession != nullptr)
        {
            destroySession(session_);
        }
        session_ = XR_NULL_HANDLE;
    }

    auto destroyInstance = GetProc<PFN_xrDestroyInstance>("xrDestroyInstance");
    if (destroyInstance != nullptr)
    {
        destroyInstance(instance_);
    }
    instance_ = XR_NULL_HANDLE;
}

bool OxrClient::CopyEye(size_t eye, ID3D11Texture2D* source)
{
    Eye& target = eyes_[eye];

    uint32_t imageIndex = 0;
    XrSwapchainImageAcquireInfo acquireInfo = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (acquireSwapchainImage_(target.swapchain, &acquireInfo, &imageIndex) != XR_SUCCESS)
    {
        return false;
    }

    XrSwapchainImageWaitInfo waitInfo = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    waitInfo.timeout = XR_INFINITE_DURATION;
    waitSwapchainImage_(target.swapchain, &waitInfo);

    if (source != nullptr && imageIndex < target.images.size())
    {
        D3D11_TEXTURE2D_DESC sourceDesc = {};
        source->GetDesc(&sourceDesc);

        // SteamVR's render target is a few pixels smaller than the runtime's
        // recommended size, so copy the overlapping region rather than
        // requiring the two to agree.
        D3D11_BOX box = {};
        box.left = 0;
        box.top = 0;
        box.front = 0;
        box.right = std::min(sourceDesc.Width, target.width);
        box.bottom = std::min(sourceDesc.Height, target.height);
        box.back = 1;

        context_->CopySubresourceRegion(target.images[imageIndex], 0, 0, 0, 0, source, 0, &box);
    }

    XrSwapchainImageReleaseInfo releaseInfo = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    return releaseSwapchainImage_(target.swapchain, &releaseInfo) == XR_SUCCESS;
}

void OxrClient::SetPendingEyes(ID3D11Texture2D* leftEye, ID3D11Texture2D* rightEye)
{
    std::lock_guard<std::mutex> lock(pendingMutex_);
    pendingEyes_[0] = leftEye;
    pendingEyes_[1] = rightEye;
}

void OxrClient::PumpEvents()
{
    if (pollEvent_ == nullptr)
    {
        return;
    }

    XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
    while (pollEvent_(instance_, &event) == XR_SUCCESS)
    {
        if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED)
        {
            const XrSessionState state = reinterpret_cast<XrEventDataSessionStateChanged*>(&event)->state;
            if (state != sessionState_)
            {
                sessionState_ = state;
                OXRSYS_LOG("[oxrsys] session state -> %d", static_cast<int>(state));
            }

            if (state == XR_SESSION_STATE_STOPPING || state == XR_SESSION_STATE_EXITING ||
                state == XR_SESSION_STATE_LOSS_PENDING)
            {
                running_.store(false, std::memory_order_release);
            }
        }
        else if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING)
        {
            OXRSYS_LOG("[oxrsys] runtime is going away");
            running_.store(false, std::memory_order_release);
        }

        event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
}

void OxrClient::FrameLoop()
{
    while (running_.load(std::memory_order_acquire))
    {
        PumpEvents();

        if (!running_.load(std::memory_order_acquire))
        {
            break;
        }

        XrFrameState frameState = {XR_TYPE_FRAME_STATE};
        XrFrameWaitInfo frameWaitInfo = {XR_TYPE_FRAME_WAIT_INFO};
        if (waitFrame_(session_, &frameWaitInfo, &frameState) != XR_SUCCESS)
        {
            OXRSYS_LOG("[oxrsys] xrWaitFrame failed; stopping the frame loop");
            running_.store(false, std::memory_order_release);
            break;
        }

        XrFrameBeginInfo frameBeginInfo = {XR_TYPE_FRAME_BEGIN_INFO};
        beginFrame_(session_, &frameBeginInfo);

        XrViewLocateInfo viewLocateInfo = {XR_TYPE_VIEW_LOCATE_INFO};
        viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        viewLocateInfo.displayTime = frameState.predictedDisplayTime;
        viewLocateInfo.space = localSpace_;

        XrViewState viewState = {XR_TYPE_VIEW_STATE};
        XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t viewCount = 0;
        locateViews_(session_, &viewLocateInfo, &viewState, 2, &viewCount, views);

        if (locateSpace_ != nullptr)
        {
            XrSpaceLocation location = {XR_TYPE_SPACE_LOCATION};
            const XrResult located = locateSpace_(viewSpace_, localSpace_, frameState.predictedDisplayTime, &location);
            if (located == XR_SUCCESS)
            {
                // Take orientation and position independently. Requiring both
                // bits together means one missing flag silently downgrades the
                // whole pose to identity, which reads as a headset that renders
                // but never moves.
                const bool hasOrientation = (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
                const bool hasPosition = (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;

                HeadPose pose;
                pose.valid = hasOrientation || hasPosition;
                pose.orientation[0] = location.pose.orientation.x;
                pose.orientation[1] = location.pose.orientation.y;
                pose.orientation[2] = location.pose.orientation.z;
                pose.orientation[3] = location.pose.orientation.w;
                pose.position[0] = location.pose.position.x;
                pose.position[1] = location.pose.position.y;
                pose.position[2] = location.pose.position.z;

                if ((framesSubmitted_ % 180) == 0)
                {
                    OXRSYS_LOG("[oxrsys] view[0] pose: flags=0x%x pos=(%.3f, %.3f, %.3f) rot=(%.3f, %.3f, %.3f, %.3f)",
                               static_cast<unsigned>(viewState.viewStateFlags),
                               static_cast<double>(views[0].pose.position.x),
                               static_cast<double>(views[0].pose.position.y),
                               static_cast<double>(views[0].pose.position.z),
                               static_cast<double>(views[0].pose.orientation.x),
                               static_cast<double>(views[0].pose.orientation.y),
                               static_cast<double>(views[0].pose.orientation.z),
                               static_cast<double>(views[0].pose.orientation.w));

                    OXRSYS_LOG("[oxrsys] head pose: flags=0x%llx pos=(%.3f, %.3f, %.3f) rot=(%.3f, %.3f, %.3f, %.3f)%s",
                               static_cast<unsigned long long>(location.locationFlags),
                               static_cast<double>(pose.position[0]),
                               static_cast<double>(pose.position[1]),
                               static_cast<double>(pose.position[2]),
                               static_cast<double>(pose.orientation[0]),
                               static_cast<double>(pose.orientation[1]),
                               static_cast<double>(pose.orientation[2]),
                               static_cast<double>(pose.orientation[3]),
                               pose.valid ? "" : "  [REJECTED: no valid bits]");
                }

                std::lock_guard<std::mutex> lock(poseMutex_);
                headPose_ = pose;
            }
            else if ((framesSubmitted_ % 720) == 0)
            {
                OXRSYS_LOG("[oxrsys] xrLocateSpace failed: %d", static_cast<int>(located));
            }
        }
        else if (framesSubmitted_ == 0)
        {
            OXRSYS_LOG("[oxrsys] no xrLocateSpace entry point; the HMD cannot report head movement");
        }

        ID3D11Texture2D* sources[2] = {nullptr, nullptr};
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            sources[0] = pendingEyes_[0];
            sources[1] = pendingEyes_[1];
        }

        XrCompositionLayerProjectionView projectionViews[2] = {};
        for (size_t eye = 0; eye < eyes_.size(); ++eye)
        {
            CopyEye(eye, sources[eye]);

            projectionViews[eye] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
            projectionViews[eye].pose = views[eye].pose;
            projectionViews[eye].fov = views[eye].fov;
            projectionViews[eye].subImage.swapchain = eyes_[eye].swapchain;
            projectionViews[eye].subImage.imageRect.extent.width = static_cast<int32_t>(eyes_[eye].width);
            projectionViews[eye].subImage.imageRect.extent.height = static_cast<int32_t>(eyes_[eye].height);
        }

        XrCompositionLayerProjection layer = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        layer.space = localSpace_;
        layer.viewCount = 2;
        layer.views = projectionViews;

        const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer)};

        XrFrameEndInfo frameEndInfo = {XR_TYPE_FRAME_END_INFO};
        frameEndInfo.displayTime = frameState.predictedDisplayTime;
        frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        // Always submit the layer, even when the runtime says shouldRender is
        // false. Honouring it looked correct and stopped the encoder: with no
        // layer there is nothing to encode, the video channel stays silent and
        // the headset sits on "waiting for video" while every other channel
        // looks healthy. A frame that is merely stale is better than no frame.
        frameEndInfo.layerCount = 1;
        frameEndInfo.layers = layers;

        const XrResult result = endFrame_(session_, &frameEndInfo);
        if (result != XR_SUCCESS)
        {
            OXRSYS_LOG("[oxrsys] xrEndFrame failed: %d", static_cast<int>(result));
        }

        ++framesSubmitted_;
        if (framesSubmitted_ <= 5 || (framesSubmitted_ % 720) == 0)
        {
            OXRSYS_LOG("[oxrsys] %llu frames submitted to the runtime",
                       static_cast<unsigned long long>(framesSubmitted_));
        }
    }

    OXRSYS_LOG("[oxrsys] frame loop ended after %llu frames", static_cast<unsigned long long>(framesSubmitted_));
}

HeadPose OxrClient::GetHeadPose() const
{
    std::lock_guard<std::mutex> lock(poseMutex_);
    return headPose_;
}

} // namespace oxrsys
