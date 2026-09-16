<!-- SPDX-License-Identifier: MPL-2.0 -->

# SteamVR driver

`clients/SteamVR/` builds `driver_oxrsys.dll`, a SteamVR (OpenVR) driver that makes
OXRSys appear to SteamVR as a head-mounted display. It is the mirror image of the
OpenComposite path: instead of translating a game's OpenVR calls to OpenXR, it lets a
real, unmodified SteamVR install drive OXRSys.

The target is SteamVR running as Windows code under CrossOver on an Apple Silicon Mac,
with the OXRSys runtime on the macOS side doing encode and streaming.

```
SteamVR compositor (vrcompositor.exe, MSVC, under Wine)
  -> driver_oxrsys.dll        (loaded into vrserver.exe; direct mode)
  -> wineopenxr bridge        (PE -> __wine_unix_call -> native x86_64 .so)
  -> OXRSys runtime           (native macOS OpenXR, XR_KHR_metal_enable)
  -> encode + stream          (Quest over USB/WiFi, or a wired WMR headset)
```

## Why direct mode

The driver advertises `IVRDriverDirectModeComponent`. In direct mode the compositor does
not present through a desktop DXGI swapchain; it asks the driver to allocate the shared
textures it renders into and hands them back once per frame through `SubmitLayer` and
`Present`.

That matters here for two reasons. The frames have to end up in OXRSys rather than on a
Windows monitor that does not exist, and the DXGI present path is where the compositor
stalls under Wine (`Failed Watchdog timeout in thread Render in
CGraphicsDevice::WaitForPresent`). Direct mode side-steps both. A verified start-up shows
`Headset is using driver direct mode` in `vrcompositor.txt` with no watchdog timeout.

## The MSVC/GCC vtable problem

The DLL is cross-compiled with mingw-w64 GCC but loaded by Valve's MSVC-built
`vrserver.exe`, so the two compilers have to agree on the layout of every interface that
crosses the boundary.

For a single-inheritance class with no virtual destructor they do agree — on vtable slot
order and on the Microsoft calling convention — with one exception: a virtual member
function that returns a class in memory. MSVC passes `this` in RCX and the hidden
return-slot pointer in RDX; GCC passes the return-slot pointer first. Disassembling a
mingw build confirms it:

```
Impl::Get():
   mov    %rcx,%r8      # RCX is the return slot
   mov    %rdx,%r9      # RDX is `this`
```

`src/OpenVRMsAbi.h` restates the two affected interfaces
(`ITrackedDeviceServerDriver::GetPose`, `IVRDisplayComponent::ComputeDistortion`) with the
return slot as an explicit first parameter, which makes GCC emit exactly the signature
MSVC expects. Everything else derives from `<openvr_driver.h>` unchanged.

This is why `build.sh` insists on mingw-w64: building the driver with a different compiler
means revisiting that header.

## Build

```bash
clients/SteamVR/build.sh [output-directory]     # default: clients/SteamVR/build
```

Produces a driver directory named `oxrsys`, laid out the way SteamVR expects:

```
oxrsys/driver.vrdrivermanifest
oxrsys/bin/win64/driver_oxrsys.dll
oxrsys/resources/settings/default.vrsettings
```

The DLL links `libgcc`, `libstdc++` and `libwinpthread` statically, so it depends only on
`KERNEL32` and the UCRT and can be dropped anywhere in a bottle.

## Install

Copy the `oxrsys` directory into the bottle and register it as an external driver in
`drive_c/users/<user>/AppData/Local/openvr/openvrpaths.vrpath`:

```json
"external_drivers" : [ "C:\\oxrsys-steamvr\\oxrsys" ]
```

Then point SteamVR at it in `Steam/config/steamvr.vrsettings`:

```json
"steamvr"       : { "forcedDriver": "oxrsys", "requireHmd": false },
"driver_oxrsys" : { "enable": true }
```

Back up both files first; SteamVR rewrites them.

## Configuration

Display geometry and timing come from the `driver_oxrsys` section of
`steamvr.vrsettings`, falling back to `resources/settings/default.vrsettings`:
`windowX`, `windowY`, `windowWidth`, `windowHeight`, `renderWidth`, `renderHeight`,
`displayFrequency`, `secondsFromVsyncToPhotons`, `ipdMeters`, `tanHalfFovHorizontal`,
`tanHalfFovVertical`.

The defaults describe the panel OXRSys currently streams to a Quest 2: 1512x1680 per eye
at 72 Hz. They are deliberately overridable so the driver can follow whatever the runtime
is actually driving without a rebuild.

## Logs

Driver output goes to `vrserver.txt` through `IVRDriverLog`, prefixed `oxrsys:`, and is
mirrored to `C:\oxrsys_steamvr_driver.log` so that failures before driver-context
initialisation are still visible.

## Status

- **Loads and owns the HMD.** `vrserver.txt` reports
  `Loaded server driver oxrsys (IServerTrackedDeviceProvider_004)`,
  `Active HMD set to oxrsys.OXRSYS-HMD-0001` and `Using existing HMD
  oxrsys.OXRSYS-HMD-0001`; `vrmonitor.txt` reports
  `CQVRController::CheckHmdDriverName: ActualTrackingSystemName: oxrsys`.
- **Direct mode is accepted.** The compositor takes the direct-mode path, reads the
  driver's recommended 1512x1680 at 72 Hz, builds its distortion meshes and initialises
  its system layer without going near a DXGI present.
- **Poses are static.** `GetPose` reports a valid, connected, identity pose. Head tracking
  from OXRSys is not wired up yet.
- **Swap texture sets are not allocated yet.** `CreateSwapTextureSet` returns null
  handles, so the compositor stops at
  `VRInitError_Compositor_CreateDriverDirectModeResolveTextures`. Allocating real shared
  D3D11 textures on a DXMT device is the next step, followed by forwarding submitted
  layers into an OpenXR session on OXRSys.
