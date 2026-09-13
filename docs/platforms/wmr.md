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
./build/drivers/oxrsys_wmr_probe --snapshot /tmp/snap   # one frame per tracking camera, as PGM
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

The headset is owned by its own process, `oxrsys-headset-helper`
(`runtime/headset_helper/`), not by the game. It opens the headset through
the driver, captures the panel's display with `CGDisplayCapture` and covers it
with a shielding-level window so no other window can ever appear on it, runs
tracking, and renders a head-tracked lobby whenever no session is submitting
frames. The runtime inside the game process, `runtime/src/WiredHeadset.mm`, is
a thin client: it connects to the helper's Unix socket (starting the helper if
it is not running), receives tracking packets, shares four side-by-side frame
surfaces once through a Mach rendezvous (the same scheme as the encoder
helper), and then submits frames by slot number. Nothing in the game process
touches USB or AppKit, which is why this works inside an x86_64 Wine process
where an in-process window did not.

```
  game process (any arch)                        oxrsys-headset-helper (arm64)
  WiredHeadset ─ Unix socket ─────────────────►  socket server
    Hello / Surfaces / SubmitFrame(slot)          Monado driver, tracking thread
    ◄── HeadsetInfo, Tracking, FrameReleased      WmrPanel (captured display)
    IOSurface x4  ── Mach send rights ────────►   lobby or client frame, 90 Hz
```

`WiredHeadset` is a process-wide singleton because the app asks for view
sizes before it creates a session.

- **Config.** `wired_headset = true` in `oxrsys-runtime.toml` (plus optional
  `wired_display_id`, `wired_eye_height_m`, `wired_position_tracking`,
  `wired_vit_library`, `wired_camera_monitor` (default `true`) and
  `wired_controller_adapter`; the runtime passes them to the helper when it starts
  it). Off by default; streaming setups are untouched. The Home app's Streaming tab has a Headset section
  that writes these keys: pick `Wired Windows Mixed Reality headset (USB)` as
  the headset mode, then adjust the eye height and, if auto-detection picks
  the wrong display, the panel display ID (see
  [macos-home.md](macos-home.md)).
- **Open.** `Instance::GetSystem` calls `WiredHeadset::EnsureOpen`, which
  connects to the helper (spawning `wired_helper_path`, or the
  `oxrsys-headset-helper` next to the runtime dylib, in its own session so it
  outlives the game), receives the headset description, and sets
  `Instance::EyeWidth/EyeHeight` to the panel's per-eye size so
  `xrEnumerateViewConfigurationViews` recommends it.
- **Tracking.** The helper polls the driver's head pose at 250 Hz, predicted
  two frames ahead, and streams `TrackingPacket`s (with the headset's left-eye
  FOV and a default IPD) over the socket; the client injects them into a
  `TrackingReceiver`, so `InputManager`, spaces and reference-space handling
  see the same data a streaming client would send. With Basalt (below) the
  position is the SLAM position plus the eye height; without it the head
  sits at the helper's eye height above the STAGE floor.
- **Frames.** `Session::StartStreamingIfNeeded` attaches the session's Metal
  device instead of starting the streaming server. `Session::EndFrame` hands
  the projection layer's snapshot images to a latest-frame-only queue; a
  compose thread waits on the snapshot's shared event on the GPU, draws both
  eyes side by side into a free IOSurface slot, waits for that GPU work, and
  sends the slot number. The helper warps the newest slot through the
  distortion mesh (`drivers/monado/wmr_panel.mm`) at the panel's refresh and
  returns superseded slots. `xrWaitFrame` paces at the panel's refresh rate.
  `xrEndFrame` never blocks on the GPU, the socket, or the display.
- **Lobby.** With no client, or half a second after the last frame, the
  helper renders a head-tracked room so the panel is never blank and a game
  launch or crash never leaves the headset showing the desktop.
- **Status.** `runtime_status.json` reports `state = "streaming"`,
  `transport = "wired"`, `device_type = "wmr"`.

### Deploying

Put `oxrsys-headset-helper` (from the native arm64 build,
`build/runtime/headset_helper/`) next to `liboxrsys-runtime.dylib` and ad-hoc
sign both, as for the encoder helper. The runtime starts it on demand; you
can also start it yourself (or from Home) to get the lobby before any game:

```bash
cp build/runtime/headset_helper/oxrsys-headset-helper ~/liboxrsys-runtime-1.1.0/
codesign --force --sign - ~/liboxrsys-runtime-1.1.0/oxrsys-headset-helper
~/liboxrsys-runtime-1.1.0/oxrsys-headset-helper &      # optional: lobby now
```

It logs to `~/Library/Application Support/OXRSys/oxrsys-headset-helper.log`
(and, when started without a terminal, Monado's own driver/SLAM messages to
`oxrsys-headset-helper-driver.log`)
and listens on `/tmp/oxrsys-headset-<uid>.sock`. Stop it before using the
probe or display tools, which need the headset for themselves. If it aborts
at start-up with `LIBUSB_ERROR_BUSY` (Monado asserts when the cameras cannot
start), another process still holds the headset's camera interface: find it
with `pgrep -fl oxrsys`, or unplug and replug the headset's USB.

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

### Camera monitor

To see what the headset's tracking cameras see, and what the trackers make
of it, start the helper with `--monitor` (or `OXRSYS_HEADSET_MONITOR=1` in
its environment). The runtime does this by default: `wired_camera_monitor =
true` in `[wired]` (Home: `Show the tracking cameras in a window`) makes it
pass `--monitor` whenever it starts the helper. By hand:

```bash
./build/runtime/headset_helper/oxrsys-headset-helper --monitor --vit-library /path/to/libbasalt.dylib
```

A normal window titled "OXRSys headset cameras" opens on a desktop screen
(never on the headset's captured display) with both 640x480 camera images
side by side at about 30 Hz, and over them:

- **Squares: the features Basalt is tracking**, at the image position it
  reports for the latest pose. The colour is the estimated depth, warm
  (orange) for near and cool (blue) for far, gray when unknown. A filled
  square is a feature that was already in the previous pose (tracked); a
  hollow square is new in this one. A healthy scene has a few dozen mostly
  filled squares per camera; all hollow means the tracker keeps losing and
  re-detecting, none at all means it is not getting frames or has no VIT
  library.
- **A circle in camera 0: the PS Move sphere** when the sphere tracker is
  active and reports a position, projected with the camera's pinhole
  parameters from the headset's calibration (distortion ignored, so it
  drifts a little towards the image edges).
- **WMR controllers** (with controller tracking running, see "WMR
  controller position from the headset cameras"): the LED blobs found in the
  latest short-exposure controller frame as small dots, a solid box labelled
  `L` (cyan) or `R` (orange) around the blobs the tracker assigned to each
  controller, and a dashed yellow `? N` box around each cluster of three or
  more blobs not assigned to a controller yet (a controller the tracker has
  not locked onto, or a stray light). The camera label adds the blob count.
- **Status lines**: the head tracking kind (`6DoF (Basalt)` or
  `3DoF (IMU)` with `no VIT library` / `IMU only` when SLAM is off), the head
  position, poses per second from the tracker, feature counts per camera,
  IMU and image samples pushed, and each camera's frame rate in its label.
  With controller tracking, a third line: LED frames per second and, per
  hand, `seen cam0/cam1` or `not seen`, `(LED sync waiting)` until timesync
  packets go out, and the head-relative position and poses per second while
  it is tracked, e.g. `L seen cam0 (0.12, -0.31, -0.38) m 42/s, R not seen`.

`OXRSYS_HEADSET_MONITOR_LED_FRAMES=1` shows the short-exposure controller
frames instead of the SLAM frames (at about 60 Hz; nearly black except for
the LEDs), which is the quickest way to see whether the controllers' LEDs
flash in sync.

Closing the window hides it; the helper keeps running. The frames come from
`oxrsys_wmr_camera_tap` (`drivers/monado/wmr_camera_tap.c`), which splits
them off next to the SLAM tracker without delaying it.

The features need `liboxrsys-vit-monitor.dylib` next to the helper (the
build puts it there, `runtime/headset_helper/`; deploy it with the helper).
It is a VIT plugin that forwards every call to the real library named by
`OXRSYS_VIT_REAL_LIBRARY` and records the pose features on the way
(`drivers/monado/vit_monitor.h`). With `--monitor`, the helper sets that
variable to the Basalt it found and loads the tracker through the shim;
without `--monitor` the tracker is loaded directly and nothing changes. When
the shim is missing the window still shows the camera images, only without
the squares.

## Controllers

Three kinds of controller work with the wired backend. The controller's IMU
gives its rotation. Position comes from the headset cameras when they see the
controller (1st-gen WMR controllers through their LEDs, one PS Move through its
sphere, both below); otherwise the runtime places the controller with a fixed
arm model around the head that follows head yaw.

| Controller | Link | Inputs mapped |
|---|---|---|
| WMR motion controllers (original, Odyssey, Reverb G2) | Bluetooth to the Mac, a USB Bluetooth adapter through `wmr_btstack` (1st-gen controllers), or the headset's own radio on Reverb G2 / Odyssey+ | trigger, squeeze (grip), menu, thumbstick and click, trackpad click as the lower face button; G2 A/B/X/Y and analog squeeze |
| PlayStation Move (ZCM1, ZCM2) | Bluetooth to the Mac | trigger, Move button (grip), Start (menu), Cross/Circle as the face buttons |

The runtime prefers the headset's controllers and falls back to PS Moves; with
Moves, the first one found is the right hand and the second the left. Inputs
land on the packet's Touch-style fields, so games see an Oculus Touch
profile.

### Pairing

- **WMR controllers.** Open the battery cover; hold the small pairing button
  inside until the LEDs flash. Controllers that macOS can pair (connect
  `Motion controller - Left` / `Motion controller - Right` in System
  Settings → Bluetooth) show up in `oxrsys_wmr_probe --list` as
  `Bluetooth motion controller`; watch them with
  `oxrsys_wmr_probe --controllers`. **1st-gen controllers (Acer, Dell, HP,
  Lenovo, Samsung Odyssey) do not pair with macOS 13 and later**: see the
  next section.
- **PS Move ZCM2** (PS4 era, micro-USB): hold PS until the LED blinks and
  pair it in System Settings → Bluetooth like any gamepad.
- **PS Move ZCM1** (PS3 era, mini-USB): it only pairs to the host whose
  Bluetooth address was written to it over USB. Pair it once with a tool that
  does that (for example `psmove pair` from psmoveapi), then unplug USB; the
  controller only streams sensor data over Bluetooth, and the runtime skips
  USB-attached Moves. `oxrsys_wmr_probe --psmove --list` shows the bus each
  Move is on; `oxrsys_wmr_probe --psmove` prints their state.

### WMR controllers through a USB Bluetooth adapter

macOS's Bluetooth daemon reads a controller's service records before it
starts pairing. A 1st-gen WMR controller in pairing mode stops answering
during that exchange (the link drops after the 5 s supervision timeout and the
LEDs go from flashing to solid), so pairing never begins; no IOBluetooth API
changes the order (`drivers/tools/wmr_bt_pair.swift` records what was tried).
Windows and BlueZ authenticate first, which is what the controllers expect.

`drivers/tools/wmr_btstack` does the same on a separate USB Bluetooth adapter
that it drives itself with [BTstack](https://github.com/bluekitchen/btstack)
over libusb, bypassing macOS's stack:

```
  controller ── Bluetooth ── USB adapter ── libusb ── wmr_btstack
                                                          │ Unix socket
                                                          ▼  (wmr_bt_bridge_protocol.h)
                          oxrsys-headset-helper: os_hid_wmr_bridge.c → Monado wmr_bt_controller
```

- **Adapter.** Any USB adapter with a Bluetooth HCI interface that works
  without a firmware upload: CSR8510 A10 (`0a12:0001`, verified) and Broadcom
  BCM20702. Realtek (`0bda:*`) and Intel (`8087:*`) adapters need vendor
  firmware that the tool doesn't load. On Apple Silicon macOS never uses
  external adapters, so nothing has to be detached; on Intel Macs the adapter
  must not be the system Bluetooth controller. While the tool runs it owns the
  adapter exclusively.
- **Build.** `brew install libusb`, clone BTstack to `~/src/btstack`, then
  `drivers/tools/wmr_btstack/build.sh` (`BTSTACK=` to point elsewhere). It
  compiles a Classic-only BTstack with a patched copy of the libusb transport:
  on macOS it skips `libusb_set_configuration` and `libusb_reset_device` when
  opening the adapter, and SCO (isochronous) transfers are compiled out. The
  unpatched transport panicked the macOS 27 beta kernel in IOUSBHostFamily.
- **Deploy.** Copy `drivers/tools/wmr_btstack/build/wmr_btstack` next to
  `oxrsys-headset-helper` and ad-hoc sign it, like the helper.
- **Enable.** `wired_controller_adapter = true` in `[wired]` (Home: Streaming
  → Headset → Motion Controllers). The runtime passes `--controller-adapter`
  to the helper, which starts `wmr_btstack -p 0` from next to itself when its
  socket doesn't answer, gives paired controllers up to five seconds to
  reconnect, and then opens whichever are connected through the socket. The
  Home app also keeps the tool running while the setting is on and an adapter
  is plugged in, so controllers can be paired before any game starts.
- **Pairing.** Pair each controller once: Home's `Pair Controller…` (or
  `wmr_btstack -p 120` from a terminal) opens a pairing window; put the
  controller in pairing mode and it bonds (the controllers ask for legacy PIN
  `0000`, which the tool answers) and connects. Afterwards pressing the
  controller's Windows button reconnects it: outside a pairing window the tool
  only page-scans, at a high duty cycle, so those reconnects get through.
  `Forget Paired Controllers` (or `wmr_btstack -r`) drops all bondings.
- **Standalone.** Without the runtime attached the tool prints each
  controller's accelerometer, gyro, stick, trigger, touchpad and buttons,
  which is a quick way to check a controller.
- **Files** (all in `~/Library/Application Support/OXRSys/`):
  `wmr_bt.sock` (bridge socket), `wmr_controllers_status.json` (for Home:
  `process_id`, `adapter_state` `ready`/`no_adapter`, `adapter_address`,
  `pairing_seconds_left`, `runtime_attached`, `controllers[]` with `hand`,
  `name`, `address`, `state` `pairing`/`connecting`/`connected` and
  `reports_per_second`, and `paired[]`), `wmr_btstack_<adapter>.tlv` (link
  keys), `wmr_btstack_names.txt`, `wmr_btstack.log`, and
  `wmr_btstack_hci.pklg`, a PacketLogger HCI trace of the last run.
- **Limits.** The helper opens controllers once, when it opens the headset;
  one switched on later is used after the helper restarts. The controllers
  send motion reports at roughly 50-65 Hz over this link. Rotation comes
  from the IMU; position from the headset cameras (below) when they see the
  controller.

### PS Move sphere position from the headset cameras

With OpenCV (`brew install opencv`, configure with `-DOXRSYS_WMR_OPENCV=ON`)
the runtime tracks one PS Move sphere positionally using the headset's two
640x480 monochrome head-tracking cameras. `drivers/monado/wmr_psmv_tracking.c`
builds a stereo calibration from the headset's own calibration blob, runs
Monado's PS Move sphere tracker on a side-by-side frame combined from both
cameras, and hands the PS Move driver a tracking factory, so the Move's
pose carries a position. The sphere is lit white and segmented by brightness
(the cameras have no colour), so only the first Move gets a position; the
second stays orientation-only with the arm model. Positions come out in the
camera frame, which moves with the head; the runtime rotates them by the head
orientation, treating the first camera as coincident with the head.

The tracker is created only when a Move is connected and torn down otherwise,
restarting the headset cameras each time (the driver starts them at open).
Camera exposure and gain are left on the driver's automatic control.
Not yet verified with a Move in hand: pair one and run
`oxrsys_wmr_probe --psmove` first (orientation only), then the runtime.

### WMR controller position from the headset cameras

With OpenCV (`-DOXRSYS_WMR_OPENCV=ON`) the helper tracks 1st-gen WMR motion
controllers' LED rings with the headset's own cameras
(`drivers/monado/wmr_controller_tracking.c`); `--no-controller-tracking`
turns it off.

- **Frames.** A WMR headset runs its cameras at 90 Hz in a SLAM, controller,
  controller cadence. The controller frames have a very short exposure in
  which only the controllers' infrared LEDs show. Monado's camera code already
  delivers them on the source's camera sinks 2 and 3; on a Dell Visor they
  arrive at 60 Hz with no controller connected.
- **LED sync.** The controllers pulse their LEDs only when the host keeps
  telling them, in the controller's own clock, when the next controller
  exposure starts. Upstream Monado does not send that yet, so the module
  wraps each controller's report handler: it tracks the controller clock from
  the tick counter in its IMU reports (1st-gen report layout only), and on the
  second controller frame of each cycle sends a timesync packet (report
  `0x03`: next exposure time, LED intensity 1..399, adjusted from the
  brightness of matched blobs, and 800 in the 11-bit field that follows, as
  Windows sends) plus a keepalive (`0x05`) every 125 ms. The
  protocol and timing follow Jan Schmidt's and Beyley Cardellio's
  `dev-constellation-controller-tracking` Monado branch.
- **Blobs and poses.** Monado's `t_rift_blobwatch` finds LED blobs in each
  camera (thresholds `OXRSYS_WMR_CT_PIXEL_THRESHOLD`, default `0x04`, and
  `OXRSYS_WMR_CT_BLOB_THRESHOLD`, default `0x10`), and Monado's generic
  constellation tracker (`src/xrt/tracking/constellation`, built from the
  pinned checkout) matches them against each controller's LED model from its
  calibration (32 LEDs on a 1st-gen controller), with the ring occlusion model
  from the same branch.
- **Left/right and bad solves.** The left and right rings are mirror images,
  so one controller's blobs also fit the other controller's model; on hardware
  both hands were often reported on the same controller. A pose is kept only
  when its implied gravity direction in the controller agrees with the
  controller's own low-passed accelerometer (Monado's fusion rotation while
  the controller accelerates hard) within `OXRSYS_WMR_CT_GRAVITY_MAX_DEG`
  (default 30). Measured on hardware the split is clean: right-hand matches
  come in under 10 degrees, mirrored ones over 60. Poses with a non-finite
  result, fewer than 4 matched LEDs, more than 1.5 m away or behind the head
  are dropped too; the tracker reports some unscored RANSAC recoveries with no
  matched LEDs and a NaN pose. When both hands still end up within 8 cm of
  each other at the same moment, the one whose gravity agrees better is kept.
  Only kept poses drive the LED brightness feedback, which never dims below
  intensity 40.
- **Prior.** Each controller's last kept pose (up to 120 ms old) is the
  tracker's prior, so it re-matches from labelled blobs and the predicted pose
  instead of searching from scratch. With a controller lying still in view of
  camera 0 this took kept poses from 1-4 per second (about 60 per second
  rejected, mostly with fewer than 4 matched LEDs) to 60 per second with none
  rejected. `OXRSYS_WMR_CT_PRIOR=0` turns it off.
- **Geometry.** Cameras are placed with the SLAM calibration's
  extrinsics in the frame the driver reports head poses in, and the tracker's
  world is the head orientation at the frame time (position ignored, so a
  wandering SLAM estimate costs nothing), which gives poses relative to the
  head.
- **Runtime.** A controller seen within the last 150 ms is placed at its
  optical position (the LED ring's centre), carried along with the current
  head pose; rotation stays the IMU's. Otherwise the arm model applies.
- **Diagnostics.** The helper logs a `controller tracking:` line every two
  seconds in the driver log (LED frame rate, blobs per camera, and per hand
  sync state, timesyncs sent, LED intensity, poses per second, last position,
  camera, matched LEDs, reprojection error, rejected poses and a histogram of
  the gravity agreement; rejections are split into NaN, fewer than 4 LEDs,
  gravity and duplicate of the other hand), plus the camera poses it
  derived at start. `OXRSYS_WMR_CT_LED_SYNC=0` stops the timesync packets,
  `OXRSYS_WMR_CT_TIME_OFFSET=N` delays them by N x 0.5 ms for tuning, and
  `OXRSYS_WMR_CT_WITHOUT_CONTROLLERS=1` runs the blob detector with no
  controller connected. `oxrsys_wmr_ct_selftest` (a CTest) renders a
  controller's constellation into the cameras and checks the tracker solves
  it, with no hardware.
- **Status.** Verified on a Dell Visor with both 1st-gen controllers over
  `wmr_btstack`: the LEDs flash in sync (blobs appear within a few seconds
  of starting), both controllers are identified in both cameras, and poses
  come at up to 60 per second per controller while in view (10-50 per second
  while being waved around before the prior was added), at
  plausible positions 0.2-0.6 m in front of and below the head with 4-12
  matched LEDs and 0.1-2 px reprojection error. The self-test solves synthetic
  views to about a millimetre. Accuracy against ground truth is not measured. Still missing: fusing optical and IMU rotation (the
  IMU yaw is not aligned with the head's), a grip offset from the ring centre,
  motion prediction, an IMU-predicted prior (the prior is the last pose, not
  where the controller has moved since), and a gyro consistency check against
  mirror fits that pass the gravity check.
  Reverb G2 / Odyssey controllers report a different IMU layout and get no
  LED sync.

### 6DoF head tracking with Basalt

Monado tracks position with an external visual-inertial system loaded at run
time through its VIT plugin interface; the supported one is Basalt (Monado's
fork). The driver library builds Monado's SLAM tracker whenever
`OXRSYS_WMR_OPENCV=ON` (a macOS `dlopen` loader replaces upstream's
Linux-only one), and the helper uses it when it finds a Basalt library:

1. Build Basalt once (needs Homebrew `eigen tbb fmt opencv cmake ninja`;
   a few minutes):

   ```bash
   drivers/tools/build_basalt.sh
   ```

   It clones the pinned commit into `~/Library/Caches/OXRSys/basalt`, fetches
   only the submodules the shared library needs, and builds
   `libbasalt.dylib` (no Pangolin, no ROS).
2. Put the library where the helper looks: next to `oxrsys-headset-helper`,
   or `~/Library/Application Support/OXRSys/libbasalt.dylib`. Or point at it
   explicitly: `wired_vit_library` in the runtime config, `--vit-library PATH`
   or `OXRSYS_VIT_LIBRARY` on the helper. `wired_position_tracking = false`
   (or `--vit-library none`) keeps IMU-only tracking even with a library
   installed.
3. Restart the helper. Its log says `6DoF head tracking through ...` and the
   open line reports `head 6DoF (Basalt)`; the status line every ten seconds
   prints the head position. `HeadsetInfo` carries a tracking description
   and a position flag to the runtime.

How it is wired: the driver can start SLAM itself, but then a missing plugin
makes headset creation fail, so the helper opens the headset with
`WMR_SLAM=false` and creates the tracker afterwards
(`drivers/monado/wmr_slam.c`) from the calibration the driver already
computed (`wh->tracking.slam_calib`), routes both cameras and the IMU into
it, and flips the driver to `slam_over_3dof`. The driver then applies
Basalt's pose (with its IMU-to-eye offset) in `xrt_device_get_tracked_pose`,
predicted with the IMU.

Pipeline settings: Basalt's generic defaults let the estimate run away
within seconds on a Visor (tens of metres in half a minute while the headset
sat still). Basalt ships a profile for WMR headsets (`msdmo`, tuned on the
Monado SLAM Datasets recorded with an Odyssey+; the differences that matter
are an image safe radius that keeps features out of the vignetted corners
and marginalising lost landmarks). The helper writes that profile, plus a
calibration file converted from the driver's calibration, to
`~/Library/Caches/OXRSys/basalt/` (`wmr.toml`, `wmr_vio_config.json`,
`wmr_calib.json`) at every start and hands the TOML to Monado as
`SLAM_CONFIG`. With a config file Monado does not send the calibration over
the plugin interface, which is why the file carries it. Edit
`wmr_vio_config.json` to experiment (it is regenerated at the next start, so
copy it and set `SLAM_CONFIG` to your own TOML to keep changes). Basalt
splits the config path as a command line, so it must not contain spaces.

Start-up: the estimate wanders during the first seconds (camera restart,
exposure settling, few landmarks), so the helper keeps the head at the fixed
eye height until the SLAM position has stayed within 5 cm for 2.5 s, then
takes that point as the origin and logs `6DoF position settled`. From then
on the reported position is the SLAM position relative to that origin, plus
the eye height. With the headset still on a desk the head then stayed within
6 cm over 50 s on a Dell Visor. `SLAM_LOG=trace` prints every frame and IMU
sample handed to Basalt; Monado's other `SLAM_*` variables
(`SLAM_PREDICTION_TYPE`, `SLAM_CONFIG`) work unchanged.

Camera restarts: the WMR source only takes new sinks by stopping and
restarting the cameras, and its stop merely requests cancellation of the USB
transfers; a start before libusb finishes cancelling fails with
`LIBUSB_ERROR_BUSY`, which the driver treats as fatal. Every consumer (SLAM,
the PS Move sphere tracker, the camera tap) therefore goes through
`oxrsys_wmr_camera_route()`, which waits in between, and later consumers
split the frames with earlier ones instead of replacing them.

What the cameras see: `oxrsys_wmr_probe --snapshot DIR` writes one frame of
each camera as `DIR/cam0.pgm` and `DIR/cam1.pgm` (8-bit grayscale, 640x480).
Basalt needs a textured scene at a reasonable distance; a headset lying in a
lap looking at a cable and a leg gives it nothing usable, and a
visual-inertial tracker without visual constraints drifts away quadratically
on the IMU alone. Verify with the headset worn, or on a desk facing the room.

### Known gaps in the runtime path

- Positional tracking needs Basalt (above); without it the head sits at a
  fixed height.
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
- Timewarp is still missing; see above.

### x86_64 build for the Wine bridge

The bridge loads the runtime inside an x86_64 (Rosetta) Wine process, so the
dylib must be x86_64. Homebrew's hidapi and libusb are arm64 only, so when
`CMAKE_OSX_ARCHITECTURES` differs from the host the driver builds both from
source (pinned hidapi 0.15.0 and libusb-cmake 1.0.30; `OXRSYS_WMR_BUNDLED_USB`
forces either behaviour). Monado's Eigen sources are compiled at `-O1` in that
configuration because Apple clang crashes on them at `-O3` for x86_64. OpenCV
sphere tracking stays off in the x86_64 build unless an x86_64 OpenCV is
available.

```bash
cmake -S . -B build-x86 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=x86_64
cmake --build build-x86 --target oxrsys_runtime
lipo -info build-x86/runtime/liboxrsys-runtime.dylib   # x86_64
```

Verified on the Dell Visor: the x86_64 probe runs under Rosetta and the
x86_64 runtime presents through the loader with `wired_headset = true`.

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
- **6DoF needs a separately built Basalt.** The SLAM tracker is built in,
  the visual-inertial system is not; see "6DoF head tracking with Basalt".
  Its quality on a Dell Visor is not yet characterised.
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
- **Controller hot-plug.** Controllers (Bluetooth, adapter or headset radio)
  are opened with the headset; ones connected later need a helper restart.
