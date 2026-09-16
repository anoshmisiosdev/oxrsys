// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <windows.h>

#include "TextureBlitter.h"

#include <d3d11.h>

#define XR_USE_PLATFORM_WIN32 1
#define XR_USE_GRAPHICS_API_D3D11 1
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace oxrsys
{

// The head pose OXRSys reports, in its LOCAL reference space.
struct HeadPose
{
    float orientation[4] = {0.0f, 0.0f, 0.0f, 1.0f}; // x, y, z, w
    float position[3] = {0.0f, 0.0f, 0.0f};
    bool valid = false;
};

// An OpenXR application, living inside vrserver.exe.
//
// This is the other half of the driver: SteamVR gives it composited eye
// textures and wants a head pose back, and OXRSys is already a working OpenXR
// runtime that encodes and streams. Rather than reimplementing encode,
// streaming and tracking on the driver side, the driver becomes an ordinary
// OpenXR client of the runtime through the wineopenxr bridge, and OXRSys does
// what it already does for any other application.
//
// The bridge is negotiated directly rather than through an OpenXR loader: the
// driver knows exactly which runtime it wants, and going straight to
// wineopenxr.dll avoids depending on the bottle's ActiveRuntime registry key.
//
// Everything here degrades to "not running": if the bridge, the runtime or the
// session is unavailable the driver keeps working as a tracked HMD with a
// static pose rather than taking vrserver down.
class OxrClient
{
public:
    OxrClient();
    ~OxrClient();

    // Brings up instance, session and swapchains against the supplied device.
    // Safe to call repeatedly; only the first call does the work, and a failure
    // is latched so the driver does not retry on every frame.
    bool Start(ID3D11Device* device, ID3D11DeviceContext* context);

    void Stop();

    bool IsRunning() const
    {
        return running_.load(std::memory_order_acquire);
    }

    DXGI_FORMAT GetSwapchainFormat() const
    {
        return swapchainFormat_;
    }

    // Hands over the latest pair of eye textures and returns immediately.
    //
    // The frame loop runs on a thread of its own rather than on whichever
    // thread calls this. xrWaitFrame blocks until the runtime is ready for the
    // next frame, and the caller here is SteamVR's compositor render thread:
    // letting it block there makes the whole compositor run at the runtime's
    // mercy, and a runtime that stops pacing stops SteamVR dead.
    void SetPendingEyes(ID3D11Texture2D* leftEye,
                        const UvRect& leftBounds,
                        ID3D11Texture2D* rightEye,
                        const UvRect& rightBounds);

    HeadPose GetHeadPose() const;

private:
    template <typename T>
    T GetProc(const char* name) const
    {
        PFN_xrVoidFunction function = nullptr;
        if (getInstanceProcAddr_ == nullptr || getInstanceProcAddr_(instance_, name, &function) != XR_SUCCESS)
        {
            return nullptr;
        }
        return reinterpret_cast<T>(function);
    }

    bool NegotiateBridge();
    bool CreateInstanceAndSystem();
    bool CreateSessionAndSwapchains(ID3D11Device* device);
    bool WaitForSessionReady();
    void LoadFrameFunctions();
    void FrameLoop();

    // Drains the runtime's event queue. An OpenXR application has to do this
    // every frame, not just while waiting to reach READY: the session advances
    // through SYNCHRONIZED, VISIBLE and FOCUSED by way of these events, and a
    // runtime is entitled to stop pacing frames for an application that never
    // collects them.
    void PumpEvents();

    bool CopyEye(size_t eye, ID3D11Texture2D* source, const UvRect& sourceBounds);

    struct Eye
    {
        XrSwapchain swapchain = XR_NULL_HANDLE;
        std::vector<ID3D11Texture2D*> images;
        uint32_t width = 0;
        uint32_t height = 0;
    };

    ID3D11DeviceContext* context_ = nullptr;

    PFN_xrGetInstanceProcAddr getInstanceProcAddr_ = nullptr;
    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId systemId_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace localSpace_ = XR_NULL_HANDLE;
    XrSpace viewSpace_ = XR_NULL_HANDLE;
    std::array<Eye, 2> eyes_;
    DXGI_FORMAT swapchainFormat_ = DXGI_FORMAT_UNKNOWN;

    PFN_xrWaitFrame waitFrame_ = nullptr;
    PFN_xrBeginFrame beginFrame_ = nullptr;
    PFN_xrEndFrame endFrame_ = nullptr;
    PFN_xrLocateViews locateViews_ = nullptr;
    PFN_xrLocateSpace locateSpace_ = nullptr;
    PFN_xrAcquireSwapchainImage acquireSwapchainImage_ = nullptr;
    PFN_xrWaitSwapchainImage waitSwapchainImage_ = nullptr;
    PFN_xrReleaseSwapchainImage releaseSwapchainImage_ = nullptr;
    PFN_xrPollEvent pollEvent_ = nullptr;

    std::string runtimeManifestPath_;

    std::atomic<bool> running_{false};
    bool startFailed_ = false;
    uint64_t framesSubmitted_ = 0;
    XrSessionState sessionState_ = XR_SESSION_STATE_UNKNOWN;

    std::thread frameThread_;
    std::mutex pendingMutex_;
    ID3D11Texture2D* pendingEyes_[2] = {nullptr, nullptr};
    UvRect pendingBounds_[2];

    TextureBlitter blitter_;
    // Whether to invert V on the way to the runtime.
    //
    // Off by default, and that default is measured rather than reasoned about:
    // a capture of the headset's own composited view shows SteamVR's frames
    // already arriving the right way up, with the UI text upright. Inverting V
    // "to convert between texture origins" turned a correct image upside down.
    // The setting stays so the behaviour can be changed without a rebuild.
    bool flipVertical_ = false;

    mutable std::mutex poseMutex_;
    HeadPose headPose_;
};

} // namespace oxrsys
