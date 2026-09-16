// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <windows.h>

#include <d3d11.h>

#define XR_USE_PLATFORM_WIN32 1
#define XR_USE_GRAPHICS_API_D3D11 1
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
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

    // Waits for the runtime's frame cadence, locates the head, copies the two
    // submitted eye textures into the runtime's swapchains and ends the frame.
    // Returns false if the frame could not be submitted.
    bool SubmitFrame(ID3D11Texture2D* leftEye, ID3D11Texture2D* rightEye);

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
    bool CopyEye(size_t eye, ID3D11Texture2D* source);

    // Reads a scanline back out of the runtime's swapchain image after the
    // copy, so the texels actually handed to OXRSys can be compared with the
    // ones SteamVR rendered rather than assumed equal to them.
    void SampleRuntimeImage(ID3D11Texture2D* image);

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

    ID3D11Texture2D* pixelSampleStaging_ = nullptr;

    std::atomic<bool> running_{false};
    bool startFailed_ = false;
    uint64_t framesSubmitted_ = 0;

    mutable std::mutex poseMutex_;
    HeadPose headPose_;
};

} // namespace oxrsys
