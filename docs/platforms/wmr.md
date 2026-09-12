# Windows Mixed Reality Headsets (macOS)

Status: **runtime integration, orientation only**. The Monado WMR driver
builds and links on macOS, a probe tool prints IMU orientation, a display tool
renders a test scene on the panel, and the runtime can use the headset in
place of a streaming client: with `wired_headset = true` an OpenXR app renders
at the panel's native eye size and refresh rate, sees the headset's FOV and
orientation, and its frames are shown on the panel through the distortion
warp. Verified end to end on a Dell Visor with the smoke client below. There
is no positional tracking and no controller input yet.

## What This Is

Windows Mixed Reality headsets (HP Reverb G1/G2, Samsung Odyssey/Odyssey+,
Lenovo Explorer, Dell Visor, Acer AH100/AH101, Medion Erazer X1000, Fujitsu
FMVHDS1, HP VR1000) have no tracking computer on board. The host must talk to
the headset over USB HID, switch the panel on, read the IMU, run the cameras,
and apply the per-device lens distortion itself.

[Monado](https://gitlab.freedesktop.org/monado/monado) has an open, permissively
licensed (BSL-1.0) driver for all of that, but its build only enables it on
Linux and its HID layer is Linux `hidraw` only. `drivers/` compiles that driver
out of an unmodified, commit-pinned Monado checkout and adds the small pieces
macOS is missing.

## Layout

```text
drivers/
├── CMakeLists.txt                 # FetchContent Monado + Eigen, driver library, probe tool
├── monado/
│   ├── os_hid_hidapi.c/.h         # hidapi backend for Monado's os_hid_device
│   ├── wmr_macos.c/.h             # hidapi enumeration + wmr_hmd_create() (replaces wmr_prober.c)
│   ├── u_file_macos.c             # config-dir helpers Monado only ships for Linux
│   └── t_euroc_recorder_stub.c    # no-op dataset recorder (real one needs OpenCV)
└── tools/
    ├── wmr_probe.c                # oxrsys_wmr_probe
    ├── wmr_display.mm             # oxrsys_wmr_display
    └── wmr_edid_override.py       # macOS display override generator (see below)
```

The library target is `oxrsys_monado_wmr`. Monado sources are compiled with
warnings disabled; the files under `drivers/monado/` are project code under
MPL-2.0 and build with `-Wall -Wextra`.

The Monado revision is pinned by commit hash in `drivers/CMakeLists.txt`. When
bumping it, re-check the source file lists there against Monado's
`src/xrt/drivers/CMakeLists.txt` and `src/xrt/auxiliary/*/CMakeLists.txt`.

## Build

Extra dependencies on macOS:

```bash
brew install hidapi libusb
```

The driver is part of the normal macOS build (`OXRSYS_BUILD_WMR_DRIVER`
defaults to `ON` on Apple platforms, `OFF` elsewhere):

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target oxrsys_wmr_probe
```

To iterate against a local Monado checkout instead of the pinned fetch:

```bash
cmake -B build -G Ninja -DFETCHCONTENT_SOURCE_DIR_MONADO=/path/to/monado
```

## Probe

```bash
./build/drivers/oxrsys_wmr_probe --list                # dump every HID device hidapi sees
./build/drivers/oxrsys_wmr_probe                       # open the headset, print orientation at 10 Hz
./build/drivers/oxrsys_wmr_probe --seconds 5 --rate 2 --log-level debug
```

On success the probe prints the headset model, the panel resolution and refresh
rate, per-eye viewports and field of view read from the headset's own
calibration blob, and then a stream of head-orientation quaternions and
yaw/pitch/roll. `flags` shows `valid,tracked` once the IMU fusion has settled.

Exit codes: `0` ran, `1` no usable headset (reason on stderr), `2` bad
arguments.

## Display

```bash
./build/drivers/oxrsys_wmr_display --list-displays     # what macOS sees right now, with modes
./build/drivers/oxrsys_wmr_display --solid             # cycle red/green/blue on the panel
./build/drivers/oxrsys_wmr_display --pattern           # lens-calibration rings and crosshair
./build/drivers/oxrsys_wmr_display                     # room scene on the headset, Ctrl-C to stop
./build/drivers/oxrsys_wmr_display --no-distortion     # raw eye images, for comparison
./build/drivers/oxrsys_wmr_display --simulate --screenshot out.png   # no hardware: synthetic headset
```

What it does, in order:

1. Records which displays are online, then opens the headset through the
   driver. `wmr_hmd_create()` sends the panel-on command, so the display
   hot-plugs a moment later.
2. Waits up to `--display-timeout` seconds (default 20) for a non-built-in
   display that has a mode matching the panel's pixel size, or failing that
   any display that was not online in step 1, or uses `--display-id`. If
   nothing appears it lists the online displays with their modes and exits 1.
3. Takes the display out of any mirror set and switches it to the panel's
   native mode at the highest refresh rate (or its largest mode, with a
   warning, when no exact one exists). With `--capture` it also captures the
   display through `CGDisplayCapture`; by default it does not.
4. Covers it with a borderless window at the screen-saver level (shielding
   level when captured) holding a `CAMetalLayer` whose drawable matches the
   display's pixel size.
5. Each refresh: predicts the eye poses ~16 ms ahead with the driver's
   `get_view_poses`, renders a procedural room per eye (coloured walls, 0.5 m
   grid, bright marker straight ahead) into eye-sized textures, then draws the
   driver's distortion mesh per view, sampling red, green and blue at their own
   UVs. Mesh coordinates follow Monado's Vulkan orientation, so only the clip y
   axis is flipped for Metal.

`--simulate` swaps in a synthetic 2880x1600 headset with a mild radial
distortion and a slowly turning head, shown in a desktop window. Together with
`--screenshot` it is the way to check the render path without hardware; the
tool always exits, falling back to a timer when the display link delivers no
callbacks (as happens without an interactive window-server session).

A frame is: two eye passes, one warp pass into a panel-sized texture, one blit
into the drawable. The panel texture is CPU-shared only so `--screenshot` can
read it; the runtime integration should render the warp straight into the
drawable.

### macOS hides the panel: the EDID override

This is the expected first failure, and it is a policy, not a fault. Every
WMR headset's EDID carries Microsoft's vendor block (OUI CA-12-5C) that
declares the panel a non-desktop head-mounted display. macOS reads it, builds
a display pipe for the panel (visible in `ioreg` as a `dispext` framebuffer
with the right resolution), and then keeps the display off the desktop: it is
never returned by `CGGetOnlineDisplayList`, so nothing can be drawn on it and
the tool reports that no display appeared even though the HDMI link trained.

Apple's display overrides can patch EDID bytes before they are interpreted
(`edid-patches`, the same mechanism Apple ships fixes with under
`/System/Library/Displays`). `drivers/tools/wmr_edid_override.py` reads the
EDID from IOKit, rewrites the Microsoft block's tag to a reserved value so it
is skipped, fixes the extension checksum, and writes the override:

```bash
python3 drivers/tools/wmr_edid_override.py                 # show the patch
sudo python3 drivers/tools/wmr_edid_override.py --install  # write it
```

Then unplug and replug the headset's video cable (or sleep and wake). The
panel should now be listed by `--list-displays` at its native size; run the
display tool as usual. To undo, delete the file the tool printed under
`/Library/Displays/Contents/Resources/Overrides/`.

Verified on a Dell Visor (EDID vendor `10ac`, product `7fce`): the block sits
at byte 146 with usage byte `0x07` (VR headset, no desktop use), and the
patch changes two bytes.

## Runtime Integration

`runtime/src/WiredHeadset.mm` is the wired backend. It is a process-wide
singleton because the panel window and the USB driver outlive sessions, and
because the app asks for view sizes before it creates a session.

- **Config.** `wired_headset = true` in `oxrsys-runtime.toml` (plus optional
  `wired_display_id` and `wired_eye_height_m`). Off by default; streaming
  setups are untouched.
- **Open.** `Instance::GetSystem` calls `WiredHeadset::EnsureOpen`, which
  opens the headset through the driver (panel on), finds its display, switches
  it to native mode, and sets `Instance::EyeWidth/EyeHeight` to the panel's
  per-eye size so `xrEnumerateViewConfigurationViews` recommends it.
- **Tracking.** A thread polls the driver's head pose at 250 Hz, predicted two
  frames ahead, and injects it into a `TrackingReceiver` as a
  `TrackingPacket`, carrying the headset's left-eye FOV and a default IPD.
  `InputManager`, spaces, and reference-space handling see the same data a
  streaming client would send. Position is fixed at `wired_eye_height_m`
  above the STAGE floor.
- **Frames.** `Session::StartStreamingIfNeeded` attaches the session's Metal
  device instead of starting the streaming server. `Session::EndFrame` hands
  the projection layer's snapshot images to a latest-frame-only queue; a
  presenter thread waits on the snapshot's shared event on the GPU and draws
  both eyes through the distortion mesh (`drivers/monado/wmr_panel.mm`) into
  the panel's drawable. `xrWaitFrame` paces at the panel's refresh rate.
  `xrEndFrame` never blocks on the GPU or the display.
- **Status.** `runtime_status.json` reports `state = "streaming"`,
  `transport = "wired"`, `device_type = "wmr"`.

### Smoke test

`oxrsys_wmr_xr_smoke` is a minimal native OpenXR client (loader → runtime →
WiredHeadset) that renders a tangent-space grid per eye tinted by the view
orientation and submits it as a projection layer:

```bash
XR_RUNTIME_JSON=build/runtime/oxrsys-runtime.json ./build/drivers/oxrsys_wmr_xr_smoke --seconds 20
```

With `wired_headset = true` in the config, the panel shows the grid and the
log reports the session presenting on the headset at 90 Hz with the panel's
eye size recommended.

## Controllers

Three kinds of controller work with the wired backend, all orientation-only
(3DoF) for now: the controller's IMU gives its rotation and the runtime places
it with a fixed arm model around the head that follows head yaw.

| Controller | Link | Inputs mapped |
|---|---|---|
| WMR motion controllers (original, Odyssey, Reverb G2) | Bluetooth to the Mac, or the headset's own radio on Reverb G2 / Odyssey+ | trigger, squeeze (grip), menu, thumbstick and click, trackpad click as the lower face button; G2 A/B/X/Y and analog squeeze |
| PlayStation Move (ZCM1, ZCM2) | Bluetooth to the Mac | trigger, Move button (grip), Start (menu), Cross/Circle as the face buttons |

The runtime prefers the headset's controllers and falls back to PS Moves; with
Moves, the first one found is the right hand and the second the left. Inputs
land on the packet's Touch-style fields, so games see an Oculus Touch
profile.

### Pairing

- **WMR controllers.** Open the battery cover; hold the small pairing button
  inside until the LEDs flash, then connect `Motion controller - Left` /
  `Motion controller - Right` in System Settings → Bluetooth. Both must be
  paired to the Mac, not to a Windows PC. Check with
  `oxrsys_wmr_probe --list` (they show as `Bluetooth motion controller`) and
  watch them with `oxrsys_wmr_probe --controllers`.
- **PS Move ZCM2** (PS4 era, micro-USB): hold PS until the LED blinks and
  pair it in System Settings → Bluetooth like any gamepad.
- **PS Move ZCM1** (PS3 era, mini-USB): it only pairs to the host whose
  Bluetooth address was written to it over USB. Pair it once with a tool that
  does that (for example `psmove pair` from psmoveapi), then unplug USB; the
  controller only streams sensor data over Bluetooth, and the runtime skips
  USB-attached Moves. `oxrsys_wmr_probe --psmove --list` shows the bus each
  Move is on; `oxrsys_wmr_probe --psmove` prints their state.

### Positional tracking (not built)

The headset's two 640x480 monochrome cameras can locate a lit PS Move
sphere by brightness (Monado's PS Move tracker) and the WMR controllers' LED
rings (Monado's constellation module). Both need OpenCV: configure with
`-DOXRSYS_WMR_OPENCV=ON` after `brew install opencv`. The tracking sources
and the camera-to-tracker plumbing are the next step; today the camera
frames go into a no-op sink. With mono cameras only one sphere can be told
apart at a time unless the controllers use different brightness.

### Known gaps in the runtime path

- Orientation only; the head sits at a fixed height. Positional tracking
  needs SLAM or an external tracker.
- Slow initial levelling. Monado's 3DoF fusion starts from identity and pulls
  toward gravity at only 3°/s while the headset is still (faster while it
  moves), so a headset picked up off a desk reads a wrong pitch for up to
  half a minute. Worn from the start, or moved around for a few seconds, it
  levels quickly. On a Dell Visor the fusion was verified to converge and hold
  (yaw drift about 1°/s from gyro bias, no positional reference to correct it).
- The protocol carries one FOV for both eyes; the right eye is mirrored from
  the left. WMR eyes differ by well under a degree, so this is tolerable.
- No timewarp: a late frame is shown as rendered. Prediction covers the
  nominal pipeline latency only.
- The x86_64 (Rosetta) runtime build used by the Wine bridge needs x86_64
  hidapi and libusb; Homebrew's arm64 libraries do not link into it.

### Troubleshooting a blank panel

The tool prints a status line every two seconds (frames, fps, whether the
display link or the fallback timer is pacing, drawable size, window
visibility). Read it together with the display lines printed at start-up.

- **Uniform light gray, and the tool said no display appeared.** A backlit LCD
  with no pixel data is light gray. First check for the EDID override case
  above: if `ioreg -l | grep '"ProductName" = "MR"'` finds the panel, macOS
  has the link and is hiding the display. If IOKit has no EDID for it, macOS
  never brought up the video link: check the cable or adapter. Headsets
  differ: the Dell Visor, Lenovo Explorer, Acer, and Samsung Odyssey use HDMI
  2.0 (2880x1440 at 90 Hz needs a 400 MHz pixel clock, beyond HDMI 1.4
  adapters), the Reverb G1/G2 use DisplayPort 1.3/1.4. Plug USB before
  running the tool, since the panel only enables its video link after the
  activate command.
- **A new display appeared but with a smaller mode.** The link came up at
  reduced bandwidth. The tool uses the largest mode and scales; the image will
  be soft. Same cable/adapter advice applies.
- **Display appeared and frames are rendering but the panel stays gray.** Try
  `--solid` (cycling colours, no scene) and then `--capture`, which switches
  to the shielding-level window path. Report the start-up lines and status
  lines.
- **Oasis or other Windows drivers.** Oasis rebinds the Windows USB driver on
  that PC only; it does not change the headset, so a headset set up with Oasis
  works here unchanged.

## How A Headset Is Opened

1. hidapi enumerates all HID interfaces.
2. The `HoloLens Sensors` device (`045e:0659`) must expose USB interface 2.
   That interface carries the IMU stream and configuration reads.
3. A vendor "companion" device on interface 0 (for example HP `03f0:0580` for
   the Reverb G2) identifies the model and controls the panel.
4. Both are opened cooperatively (hidapi's macOS exclusive-open is turned off)
   and handed to Monado's `wmr_hmd_create()`, which activates the display,
   enables the IMU, opens the tracking cameras over libusb (interface 3) and
   starts the reader thread.

If `--list` shows `HoloLens Sensors` but with interface `-1`, hidapi could not
read the USB interface number; report that with the full `--list` output.

## Known Gaps

- **Cameras are mandatory in the driver.** `wmr_hmd_create()` fails if libusb
  cannot claim interface 3 of the sensors device. If macOS ever holds that
  interface, the fix is a small Monado patch to make the source optional.
- **Orientation only.** Positional (6DoF) tracking in Monado comes from its
  Basalt SLAM integration, which is not built here.
- **No runtime integration yet.** The display tool proves the presentation
  path; the next step is a local wired backend in the runtime that feeds
  `xrt_device` poses into the tracking path and presents submitted OpenXR
  layers through the same warp instead of a test scene.
- **Display detection is by pixel size.** The panel is matched by a display
  mode equal to `screens[0]`; a monitor with the same native resolution would
  also match, so use `--display-id` in that case.
- **Refresh rate.** The driver does not report a nominal frame interval; the
  tool picks the highest-rate mode at the native size and lets the display link
  pace frames.
- **Controller calibration cache path.** Monado writes cached controller
  calibration to `$XDG_CONFIG_HOME/monado/wmr/` or, failing that,
  `~/monado/wmr/`. That is Monado's choice on non-Linux platforms and may be
  redirected later.
- **Bluetooth controllers** (the ones not paired through the headset's own
  radio) are not enumerated yet; only Reverb G2 and Odyssey+ controllers, which
  talk through the HMD, are returned by the open call.
