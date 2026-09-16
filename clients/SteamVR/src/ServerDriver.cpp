// SPDX-License-Identifier: MPL-2.0

// Entry point of the OXRSys SteamVR driver.
//
// vrserver.exe loads this DLL, calls HmdDriverFactory for
// IServerTrackedDeviceProvider_004, and from there the driver adds a single
// head-mounted display backed by the OXRSys runtime on the macOS side.

#include "DriverLog.h"
#include "HmdDevice.h"

#include <openvr_driver.h>

#include <cstring>
#include <memory>

namespace oxrsys
{

namespace
{
constexpr const char* kMirrorLogPath = "C:\\oxrsys_steamvr_driver.log";
} // namespace

class ServerDriver final : public vr::IServerTrackedDeviceProvider
{
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

private:
    std::unique_ptr<HmdDevice> hmd_;
};

vr::EVRInitError ServerDriver::Init(vr::IVRDriverContext* pDriverContext)
{
    DriverLogOpen(kMirrorLogPath);

    const vr::EVRInitError contextError = vr::InitServerDriverContext(pDriverContext);
    if (contextError != vr::VRInitError_None)
    {
        OXRSYS_LOG("[oxrsys] InitServerDriverContext failed: %d", static_cast<int>(contextError));
        return contextError;
    }

    OXRSYS_LOG("[oxrsys] driver init, build " __DATE__ " " __TIME__);

    HmdDisplayConfig config;
    LoadHmdDisplayConfig(config);
    OXRSYS_LOG("[oxrsys] display: window %ux%u, render %ux%u per eye, %.2f Hz",
               config.windowWidth,
               config.windowHeight,
               config.renderWidth,
               config.renderHeight,
               static_cast<double>(config.displayFrequency));

    hmd_ = std::make_unique<HmdDevice>(config);

    const bool added = vr::VRServerDriverHost()->TrackedDeviceAdded(
        hmd_->GetSerialNumber(),
        vr::TrackedDeviceClass_HMD,
        hmd_->AsOpenVR());

    if (!added)
    {
        OXRSYS_LOG("[oxrsys] TrackedDeviceAdded failed for %s", hmd_->GetSerialNumber());
        hmd_.reset();
        return vr::VRInitError_Driver_Unknown;
    }

    OXRSYS_LOG("[oxrsys] TrackedDeviceAdded ok: %s", hmd_->GetSerialNumber());
    return vr::VRInitError_None;
}

void ServerDriver::Cleanup()
{
    OXRSYS_LOG("[oxrsys] driver cleanup");
    hmd_.reset();
    vr::CleanupDriverContext();
    DriverLogClose();
}

const char* const* ServerDriver::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void ServerDriver::RunFrame()
{
    if (hmd_)
    {
        hmd_->RunFrame();
    }
}

bool ServerDriver::ShouldBlockStandbyMode()
{
    // SteamVR decides the headset is not being worn from a proximity sensor.
    // This one has none, so it probes /user/head/proximity, gets nothing, and
    // parks the compositor in standby a few seconds after start-up -- which
    // stops Present and strands whatever application is running. Blocking
    // standby is what this entry point is for.
    return true;
}

void ServerDriver::EnterStandby()
{
}

void ServerDriver::LeaveStandby()
{
}

namespace
{
ServerDriver g_serverDriver;
} // namespace

} // namespace oxrsys

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
    if (pInterfaceName != nullptr && std::strcmp(pInterfaceName, vr::IServerTrackedDeviceProvider_Version) == 0)
    {
        if (pReturnCode != nullptr)
        {
            *pReturnCode = vr::VRInitError_None;
        }
        return &oxrsys::g_serverDriver;
    }

    if (pReturnCode != nullptr)
    {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
