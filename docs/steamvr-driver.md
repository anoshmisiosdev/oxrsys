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

## Frames and poses

The driver is itself an OpenXR application. Rather than reimplementing encode,
streaming and tracking on the driver side, it opens a session on OXRSys through
the wineopenxr bridge from inside `vrserver.exe` and lets the runtime do what it
already does for any other application.

It negotiates `wineopenxr.dll` directly instead of going through an OpenXR
loader: the driver knows which runtime it wants, and that avoids depending on
the bottle's `ActiveRuntime` registry key. The session binds the same D3D11
device the direct-mode swap textures are allocated on, so handing a frame over
is a `CopySubresourceRegion` between two textures on one device, with no
cross-process import. The head pose the HMD reports to SteamVR comes from
`xrLocateSpace` on the runtime's VIEW space.

### Vsync

The compositor will not schedule a frame until it knows when the display
refreshes. Under Wine its own GPU timing queries come back disjoint
(`Aborting GetDeltas(CompositorPresent) disjoint!`), so it never works that out
for itself and presents a handful of frames and stops. The driver therefore
declares `Prop_DriverDirectModeSendsVsyncEvents_Bool` and drives the cadence
from a thread of its own, which is what any headset that is not a real attached
display has to do anyway.

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

### The runtime manifest, and where it has to be set

The OpenXR loader that resolves the runtime lives on the macOS side of the
bridge, so it reads the host process environment and needs `XR_RUNTIME_JSON`.
Without it `xrCreateInstance` returns `XR_ERROR_RUNTIME_UNAVAILABLE` (`-51`) and
the driver falls back to a tracked HMD with a static pose, logging the resolved
path (or `<unset>`) next to the error.

Set it for the whole bottle, in `cxbottle.conf`:

```ini
[EnvironmentVariables]
"XR_RUNTIME_JSON" = "/path/to/oxrsys-runtime.json"
```

CrossOver applies that to the host environment of every process in the bottle,
so `vrserver.exe` inherits it however it was started -- including when Steam
spawns it for a game launch. Steam has to be restarted once after the change,
because it passes its own environment to its children.

Two things that look like they should work and do not, both measured:

- **The driver cannot set it itself.** `SetEnvironmentVariableA` writes to Wine's
  copy of the environment, not the host one the loader reads. A probe that set
  the variable and then called `xrCreateInstance` still got `-51`.
- **The loader's file fallbacks do not apply on macOS.**
  `/etc/xdg/openxr/1/active_runtime.json` and
  `~/.config/openxr/1/active_runtime.json` can both exist and point at the
  runtime, and the loader still fails with "failed to determine active runtime
  file path for this environment". Those search paths are Linux-only.

Exporting the variable before starting Steam works too, but only for that
launch; the bottle setting is the one that survives.

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

Verified with SteamVR 2.17.9 in a CrossOver bottle, this driver supplying the
HMD and the null driver disabled.

- **Loads and owns the HMD.** `vrserver.txt` reports
  `Loaded server driver oxrsys (IServerTrackedDeviceProvider_004)`,
  `Active HMD set to oxrsys.OXRSYS-HMD-0001` and `Using existing HMD
  oxrsys.OXRSYS-HMD-0001`; `vrmonitor.txt` reports
  `CQVRController::CheckHmdDriverName: ActualTrackingSystemName: oxrsys` and
  reaches `SteamVRSystemState_Ready`. SteamVR records the HMD in its own
  settings as `"ActualHMDDriver": "oxrsys"`.
- **Direct mode runs.** The compositor takes the direct-mode path, allocates its
  swap texture sets from the driver, and sustains `Present` without going near a
  DXGI present.
- **Frames reach the runtime and the headset.** The OpenXR session comes up at
  1512x1680 per eye and the runtime reports `state=streaming` to a connected
  Quest 2 with the application named `OXRSys SteamVR driver`.
- **Poses come from the runtime**, though they have not yet been checked against
  real head movement.
- **Only the first layer of each frame is forwarded.** Overlays and the
  dashboard are dropped rather than composited.
- **Room setup.** SteamVR stays at `NotReady` until a chaperone universe exists
  for the driver's universe id; `Steam/config/chaperone_info.vrchap` with
  `"universeID": "2"` satisfies it without running the wizard.

### Synchronisation

`Present` receives a sync texture the compositor owns. The driver opens it with
`OpenSharedResource` and takes its keyed mutex for the duration of the handover;
without that the compositor logs `WaitForAcquire timed out (FAILED); rendering
the next frame before the driver took the sync texture` and runs its pipeline
unsynchronised. Both the open and the acquire are confirmed working
(`sync texture acquired via keyed mutex`), which also demonstrates that DXMT's
cross-process texture sharing carries handles in both directions.

### Checking the content path

`CreateSwapTextureSet` hands SteamVR shared handles for textures the driver
allocated, and the compositor renders into them from its own process. Whether
that work actually lands in this process is not something the API reports, so
the driver reads one texel back from the texture it is about to forward and logs
it. A sample that never changes means no content is arriving, whatever the frame
counters say.

A flat colour is not by itself a bug: with no application submitting, the
compositor renders its own loading background, and `vrcompositor.txt` says so
(`Loading...  0 total....  0 presents.`). Read that line before suspecting the
copy.

### Known issue: the compositor's own output is a flat colour

As measured, SteamVR's compositor renders a uniform colour and the driver
forwards it faithfully. The source scanline read out of the texture SteamVR
rendered into and the destination scanline read out of the runtime swapchain
image after the copy are bit-identical (both uniform, 64-texel checksum
`0x04d40000`), so nothing is lost between the compositor and OXRSys. The same
flat colour appeared on SteamVR's desktop window when it ran on the built-in
null driver, before this driver existed, which places the defect upstream of the
driver in the game-to-compositor hop rather than in anything here.

This is where the setup differs from Proton, which is worth knowing before
digging further: under Proton, SteamVR's compositor is a native Linux process
and a thin Wine-side shim forwards the game's calls out to it, so the texture
handover happens once, host-native. Here `vrserver.exe` and `vrcompositor.exe`
both run as Windows binaries under CrossOver, so the game-to-compositor share
also has to work inside Wine.
