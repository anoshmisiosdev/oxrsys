// SPDX-License-Identifier: MPL-2.0

#pragma once

// Microsoft-ABI restatements of the OpenVR driver interfaces this driver
// implements.
//
// The driver DLL is cross-compiled with mingw-w64 GCC, but it is loaded by
// Valve's MSVC-built `vrserver.exe`. For a single-inheritance class with no
// virtual destructor the two compilers agree on the x86-64 vtable layout and on
// the Microsoft calling convention, so most interfaces can simply be derived
// from the declarations in <openvr_driver.h>.
//
// They disagree about one thing: a virtual member function that returns a class
// in memory. MSVC passes `this` in RCX and the hidden return-slot pointer in
// RDX; GCC passes the hidden return-slot pointer first and `this` second. Any
// such method therefore has to be restated with the return slot as an explicit
// first parameter, which makes GCC emit exactly the signature MSVC expects
// (`this` in RCX, return slot in RDX, slot address returned in RAX).
//
// Only the affected methods change. Everything else is declared exactly as in
// <openvr_driver.h>, in the same order, so the vtables line up slot for slot.
// Objects are handed to vrserver through `AsOpenVR()` helpers that reinterpret
// the pointer as the upstream type.

#include <openvr_driver.h>

namespace oxrsys
{

// Mirrors vr::ITrackedDeviceServerDriver.
// Slot 5, `DriverPose_t GetPose()`, returns a class in memory.
class ITrackedDeviceServerDriverMsAbi
{
public:
    virtual vr::EVRInitError Activate(uint32_t unObjectId) = 0;
    virtual void Deactivate() = 0;
    virtual void EnterStandby() = 0;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) = 0;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
    virtual vr::DriverPose_t* GetPose(vr::DriverPose_t* pReturnSlot) = 0;

    vr::ITrackedDeviceServerDriver* AsOpenVR()
    {
        return reinterpret_cast<vr::ITrackedDeviceServerDriver*>(this);
    }
};

// Mirrors vr::IVRDisplayComponent.
// Slot 6, `DistortionCoordinates_t ComputeDistortion(...)`, returns a class in
// memory.
class IVRDisplayComponentMsAbi
{
public:
    virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
    virtual bool IsDisplayOnDesktop() = 0;
    virtual bool IsDisplayRealDisplay() = 0;
    virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
    virtual void GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
    virtual void GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
    virtual vr::DistortionCoordinates_t* ComputeDistortion(vr::DistortionCoordinates_t* pReturnSlot, vr::EVREye eEye, float fU, float fV) = 0;
    virtual bool ComputeInverseDistortion(vr::HmdVector2_t* pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV) = 0;
};

} // namespace oxrsys
