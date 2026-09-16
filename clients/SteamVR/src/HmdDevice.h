// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "OpenVRMsAbi.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace oxrsys
{

class DirectModeComponent;

// Display geometry and timing the driver advertises to SteamVR.
//
// The defaults describe the panel OXRSys currently streams to a Quest 2. Every
// field can be overridden from the `driver_oxrsys` section of
// `steamvr.vrsettings`, so the driver can be pointed at a different headset
// without a rebuild.
struct HmdDisplayConfig
{
    int32_t windowX = 0;
    int32_t windowY = 0;
    uint32_t windowWidth = 3024;
    uint32_t windowHeight = 1680;
    uint32_t renderWidth = 1512;
    uint32_t renderHeight = 1680;
    float displayFrequency = 72.0f;
    float secondsFromVsyncToPhotons = 0.011f;
    float ipdMeters = 0.063f;
    // Symmetric half-angle tangents. OXRSys reports the real per-eye field of
    // view once a session is running; until then a roughly 100 degree square
    // frustum is close enough for SteamVR to finish start-up.
    float tanHalfFovHorizontal = 1.19f;
    float tanHalfFovVertical = 1.19f;
};

void LoadHmdDisplayConfig(HmdDisplayConfig& config);

// Display geometry reported to SteamVR.
//
// Kept as a standalone object rather than a second base class of HmdDevice:
// each interface pointer handed to vrserver then refers to an object with a
// single vtable, which keeps the layout trivially comparable with MSVC's.
class HmdDisplayComponent final : public IVRDisplayComponentMsAbi
{
public:
    explicit HmdDisplayComponent(const HmdDisplayConfig& config)
        : config_(config)
    {
    }

private:
    void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
    bool IsDisplayOnDesktop() override;
    bool IsDisplayRealDisplay() override;
    void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override;
    void GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
    void GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) override;
    vr::DistortionCoordinates_t* ComputeDistortion(vr::DistortionCoordinates_t* pReturnSlot, vr::EVREye eEye, float fU, float fV) override;
    bool ComputeInverseDistortion(vr::HmdVector2_t* pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV) override;

    const HmdDisplayConfig& config_;
};

// The head-mounted display OXRSys presents to SteamVR.
//
// Direct mode is deliberate: the SteamVR compositor hands submitted textures
// straight to the driver instead of presenting them through a DXGI swapchain,
// which is the path that matters when the compositor runs under Wine on top of
// DXMT.
class HmdDevice final : public ITrackedDeviceServerDriverMsAbi
{
public:
    explicit HmdDevice(const HmdDisplayConfig& config);
    ~HmdDevice();

    const char* GetSerialNumber() const
    {
        return serialNumber_.c_str();
    }

    uint32_t GetObjectId() const
    {
        return objectId_.load(std::memory_order_relaxed);
    }

    // Pushes the current pose to SteamVR. Called from the provider's frame loop.
    void RunFrame();

    vr::EVRInitError Activate(uint32_t unObjectId) override;
    void Deactivate() override;
    void EnterStandby() override;
    void* GetComponent(const char* pchComponentNameAndVersion) override;
    void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    vr::DriverPose_t* GetPose(vr::DriverPose_t* pReturnSlot) override;

private:
    vr::DriverPose_t BuildPose() const;

    // Drives SteamVR's frame scheduler.
    //
    // The compositor will not schedule a frame until it knows when the display
    // refreshes. Under Wine its own GPU timing queries come back disjoint, so
    // it never works that out for itself; the driver therefore declares
    // Prop_DriverDirectModeSendsVsyncEvents_Bool and supplies the cadence from
    // here, which is what every headset that is not a real attached display
    // has to do anyway.
    void VsyncLoop();

    HmdDisplayConfig config_;
    HmdDisplayComponent displayComponent_;
    std::unique_ptr<DirectModeComponent> directModeComponent_;
    std::string serialNumber_;
    std::string modelNumber_;
    std::atomic<uint32_t> objectId_{vr::k_unTrackedDeviceIndexInvalid};
    vr::PropertyContainerHandle_t propertyContainer_ = vr::k_ulInvalidPropertyContainer;
    std::thread vsyncThread_;
    std::atomic<bool> vsyncRunning_{false};
};

} // namespace oxrsys
