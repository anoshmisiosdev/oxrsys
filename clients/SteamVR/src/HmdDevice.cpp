// SPDX-License-Identifier: MPL-2.0

#include "HmdDevice.h"

#include "DirectModeComponent.h"
#include "DriverLog.h"

#include <chrono>
#include <cstring>

namespace oxrsys
{

namespace
{

constexpr const char* kSettingsSection = "driver_oxrsys";

int32_t ReadInt32(const char* pchKey, int32_t defaultValue)
{
    if (vr::VRSettings() == nullptr)
    {
        return defaultValue;
    }

    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const int32_t value = vr::VRSettings()->GetInt32(kSettingsSection, pchKey, &error);
    return error == vr::VRSettingsError_None ? value : defaultValue;
}

float ReadFloat(const char* pchKey, float defaultValue)
{
    if (vr::VRSettings() == nullptr)
    {
        return defaultValue;
    }

    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const float value = vr::VRSettings()->GetFloat(kSettingsSection, pchKey, &error);
    return error == vr::VRSettingsError_None ? value : defaultValue;
}

constexpr vr::HmdQuaternion_t kIdentityRotation = {1.0, 0.0, 0.0, 0.0};

} // namespace

void LoadHmdDisplayConfig(HmdDisplayConfig& config)
{
    config.windowX = ReadInt32("windowX", config.windowX);
    config.windowY = ReadInt32("windowY", config.windowY);
    config.windowWidth = static_cast<uint32_t>(ReadInt32("windowWidth", static_cast<int32_t>(config.windowWidth)));
    config.windowHeight = static_cast<uint32_t>(ReadInt32("windowHeight", static_cast<int32_t>(config.windowHeight)));
    config.renderWidth = static_cast<uint32_t>(ReadInt32("renderWidth", static_cast<int32_t>(config.renderWidth)));
    config.renderHeight = static_cast<uint32_t>(ReadInt32("renderHeight", static_cast<int32_t>(config.renderHeight)));
    config.displayFrequency = ReadFloat("displayFrequency", config.displayFrequency);
    config.secondsFromVsyncToPhotons = ReadFloat("secondsFromVsyncToPhotons", config.secondsFromVsyncToPhotons);
    config.ipdMeters = ReadFloat("ipdMeters", config.ipdMeters);
    config.tanHalfFovHorizontal = ReadFloat("tanHalfFovHorizontal", config.tanHalfFovHorizontal);
    config.tanHalfFovVertical = ReadFloat("tanHalfFovVertical", config.tanHalfFovVertical);
}

// --- HmdDisplayComponent -----------------------------------------------------

void HmdDisplayComponent::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnX = config_.windowX;
    *pnY = config_.windowY;
    *pnWidth = config_.windowWidth;
    *pnHeight = config_.windowHeight;
}

bool HmdDisplayComponent::IsDisplayOnDesktop()
{
    // The panel lives on the headset, not on the Mac desktop.
    return false;
}

bool HmdDisplayComponent::IsDisplayRealDisplay()
{
    // Direct mode: there is no display attached to this machine at all.
    return false;
}

void HmdDisplayComponent::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnWidth = config_.renderWidth;
    *pnHeight = config_.renderHeight;
}

void HmdDisplayComponent::GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnY = 0;
    *pnWidth = config_.windowWidth / 2;
    *pnHeight = config_.windowHeight;
    *pnX = eEye == vr::Eye_Left ? 0 : config_.windowWidth / 2;
}

void HmdDisplayComponent::GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
    (void)eEye;
    *pfLeft = -config_.tanHalfFovHorizontal;
    *pfRight = config_.tanHalfFovHorizontal;
    *pfTop = -config_.tanHalfFovVertical;
    *pfBottom = config_.tanHalfFovVertical;
}

vr::DistortionCoordinates_t* HmdDisplayComponent::ComputeDistortion(vr::DistortionCoordinates_t* pReturnSlot, vr::EVREye eEye, float fU, float fV)
{
    (void)eEye;

    // The headset client applies its own lens correction, so the driver hands
    // SteamVR an identity mapping.
    pReturnSlot->rfRed[0] = fU;
    pReturnSlot->rfRed[1] = fV;
    pReturnSlot->rfGreen[0] = fU;
    pReturnSlot->rfGreen[1] = fV;
    pReturnSlot->rfBlue[0] = fU;
    pReturnSlot->rfBlue[1] = fV;
    return pReturnSlot;
}

bool HmdDisplayComponent::ComputeInverseDistortion(vr::HmdVector2_t* pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV)
{
    (void)eEye;
    (void)unChannel;

    if (pResult == nullptr)
    {
        return false;
    }

    pResult->v[0] = fU;
    pResult->v[1] = fV;
    return true;
}

// --- HmdDevice ---------------------------------------------------------------

HmdDevice::HmdDevice(const HmdDisplayConfig& config)
    : config_(config)
    , displayComponent_(config_)
    , directModeComponent_(std::make_unique<DirectModeComponent>())
    , serialNumber_("OXRSYS-HMD-0001")
    , modelNumber_("OXRSys Virtual HMD")
{
}

HmdDevice::~HmdDevice() = default;

vr::EVRInitError HmdDevice::Activate(uint32_t unObjectId)
{
    objectId_.store(unObjectId, std::memory_order_relaxed);
    propertyContainer_ = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    OXRSYS_LOG("[oxrsys] Activate: object id %u, property container %llu",
               unObjectId,
               static_cast<unsigned long long>(propertyContainer_));

    vr::CVRPropertyHelpers* properties = vr::VRProperties();

    properties->SetStringProperty(propertyContainer_, vr::Prop_TrackingSystemName_String, "oxrsys");
    properties->SetStringProperty(propertyContainer_, vr::Prop_ManufacturerName_String, "OXRSys");
    properties->SetStringProperty(propertyContainer_, vr::Prop_ModelNumber_String, modelNumber_.c_str());
    properties->SetStringProperty(propertyContainer_, vr::Prop_SerialNumber_String, serialNumber_.c_str());
    properties->SetStringProperty(propertyContainer_, vr::Prop_RenderModelName_String, modelNumber_.c_str());
    properties->SetStringProperty(propertyContainer_, vr::Prop_InputProfilePath_String, "{oxrsys}/input/oxrsys_hmd_profile.json");

    properties->SetFloatProperty(propertyContainer_, vr::Prop_UserIpdMeters_Float, config_.ipdMeters);
    properties->SetFloatProperty(propertyContainer_, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
    properties->SetFloatProperty(propertyContainer_, vr::Prop_DisplayFrequency_Float, config_.displayFrequency);
    properties->SetFloatProperty(propertyContainer_, vr::Prop_SecondsFromVsyncToPhotons_Float, config_.secondsFromVsyncToPhotons);

    properties->SetUint64Property(propertyContainer_, vr::Prop_CurrentUniverseId_Uint64, 2);

    properties->SetBoolProperty(propertyContainer_, vr::Prop_IsOnDesktop_Bool, false);
    properties->SetBoolProperty(propertyContainer_, vr::Prop_DeviceIsWireless_Bool, true);
    properties->SetBoolProperty(propertyContainer_, vr::Prop_ContainsProximitySensor_Bool, true);
    properties->SetBoolProperty(propertyContainer_, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
    properties->SetBoolProperty(propertyContainer_, vr::Prop_HasDriverDirectModeComponent_Bool, true);
    properties->SetBoolProperty(propertyContainer_, vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);

    // SteamVR uses this to decide whether an HMD is present at all; without it
    // some of the start-up paths keep waiting for a display.
    properties->SetBoolProperty(propertyContainer_, vr::Prop_DisplayDebugMode_Bool, false);

    // Tell SteamVR the headset is being worn.
    //
    // Without a proximity input it probes /user/head/proximity, rejects it as
    // the wrong type, decides nobody is wearing the headset and parks the
    // compositor in standby -- which stops it scheduling renders, so the image
    // only updates when something else forces one.
    if (vr::VRDriverInput() != nullptr)
    {
        const vr::EVRInputError inputError =
            vr::VRDriverInput()->CreateBooleanComponent(propertyContainer_, "/proximity", &proximityComponent_);
        if (inputError == vr::VRInputError_None)
        {
            vr::VRDriverInput()->UpdateBooleanComponent(proximityComponent_, true, 0.0);
        }
        else
        {
            OXRSYS_LOG("[oxrsys] could not create the proximity input: %d", static_cast<int>(inputError));
        }
    }

    vsyncRunning_.store(true, std::memory_order_release);
    vsyncThread_ = std::thread([this] { VsyncLoop(); });

    return vr::VRInitError_None;
}

void HmdDevice::VsyncLoop()
{
    const float frequency = config_.displayFrequency > 1.0f ? config_.displayFrequency : 72.0f;
    const auto framePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(frequency)));

    auto nextFrame = std::chrono::steady_clock::now() + framePeriod;

    while (vsyncRunning_.load(std::memory_order_acquire))
    {
        std::this_thread::sleep_until(nextFrame);
        nextFrame += framePeriod;

        // A long stall (start-up, standby) would otherwise leave the schedule
        // chasing a deadline already in the past for thousands of frames.
        const auto now = std::chrono::steady_clock::now();
        if (nextFrame < now)
        {
            nextFrame = now + framePeriod;
        }

        if (vr::VRServerDriverHost() == nullptr)
        {
            continue;
        }

        vr::VRServerDriverHost()->VsyncEvent(0.0);
        RunFrame();

        // Re-assert presence periodically; SteamVR otherwise drifts back into
        // standby after a while.
        if (proximityComponent_ != vr::k_ulInvalidInputComponentHandle && vr::VRDriverInput() != nullptr)
        {
            vr::VRDriverInput()->UpdateBooleanComponent(proximityComponent_, true, 0.0);
        }
    }
}

void HmdDevice::Deactivate()
{
    OXRSYS_LOG("[oxrsys] Deactivate");

    vsyncRunning_.store(false, std::memory_order_release);
    if (vsyncThread_.joinable())
    {
        vsyncThread_.join();
    }

    objectId_.store(vr::k_unTrackedDeviceIndexInvalid, std::memory_order_relaxed);
}

void HmdDevice::EnterStandby()
{
    OXRSYS_LOG("[oxrsys] EnterStandby");
}

void* HmdDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    if (pchComponentNameAndVersion == nullptr)
    {
        return nullptr;
    }

    if (std::strcmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version) == 0)
    {
        OXRSYS_LOG("[oxrsys] GetComponent: %s -> display component", pchComponentNameAndVersion);
        return static_cast<IVRDisplayComponentMsAbi*>(&displayComponent_);
    }

    if (std::strcmp(pchComponentNameAndVersion, vr::IVRDriverDirectModeComponent_Version) == 0)
    {
        OXRSYS_LOG("[oxrsys] GetComponent: %s -> direct mode component", pchComponentNameAndVersion);
        return static_cast<vr::IVRDriverDirectModeComponent*>(directModeComponent_.get());
    }

    OXRSYS_LOG("[oxrsys] GetComponent: %s -> unsupported", pchComponentNameAndVersion);
    return nullptr;
}

void HmdDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    (void)pchRequest;
    if (pchResponseBuffer != nullptr && unResponseBufferSize > 0)
    {
        pchResponseBuffer[0] = '\0';
    }
}

vr::DriverPose_t HmdDevice::BuildPose() const
{
    vr::DriverPose_t pose = {};

    pose.poseTimeOffset = 0.0;
    pose.qWorldFromDriverRotation = kIdentityRotation;
    pose.qDriverFromHeadRotation = kIdentityRotation;
    pose.qRotation = kIdentityRotation;
    pose.vecPosition[0] = 0.0;
    pose.vecPosition[1] = 0.0;
    pose.vecPosition[2] = 0.0;
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.willDriftInYaw = false;
    pose.shouldApplyHeadModel = false;

    // OpenXR and OpenVR agree on handedness and axes, so the runtime's pose in
    // its LOCAL space maps straight across. Until the runtime has located the
    // head the identity pose above stands, which keeps the HMD tracked rather
    // than making SteamVR treat it as lost.
    const HeadPose head = directModeComponent_->GetHeadPose();
    if (head.valid)
    {
        pose.qRotation.x = head.orientation[0];
        pose.qRotation.y = head.orientation[1];
        pose.qRotation.z = head.orientation[2];
        pose.qRotation.w = head.orientation[3];
        pose.vecPosition[0] = head.position[0];
        pose.vecPosition[1] = head.position[1];
        pose.vecPosition[2] = head.position[2];
    }

    return pose;
}

vr::DriverPose_t* HmdDevice::GetPose(vr::DriverPose_t* pReturnSlot)
{
    *pReturnSlot = BuildPose();
    return pReturnSlot;
}

void HmdDevice::RunFrame()
{
    const uint32_t objectId = objectId_.load(std::memory_order_relaxed);
    if (objectId == vr::k_unTrackedDeviceIndexInvalid)
    {
        return;
    }

    const vr::DriverPose_t pose = BuildPose();
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(objectId, pose, sizeof(pose));
}

} // namespace oxrsys
