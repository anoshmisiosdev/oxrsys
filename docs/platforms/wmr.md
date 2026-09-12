# Windows Mixed Reality Headsets (macOS)

Status: **driver bring-up**. The Monado WMR driver builds and links on macOS
and a probe tool can open a headset and print IMU orientation. Nothing is wired
into the OpenXR runtime yet; there is no display output and no positional
tracking.

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
    └── wmr_probe.c                # oxrsys_wmr_probe
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
- **No runtime integration yet.** The next step is a local wired backend in the
  runtime that feeds `xrt_device` poses into the tracking path and presents
  distortion-corrected frames on the headset's display.
- **Controller calibration cache path.** Monado writes cached controller
  calibration to `$XDG_CONFIG_HOME/monado/wmr/` or, failing that,
  `~/monado/wmr/`. That is Monado's choice on non-Linux platforms and may be
  redirected later.
- **Bluetooth controllers** (the ones not paired through the headset's own
  radio) are not enumerated yet; only Reverb G2 and Odyssey+ controllers, which
  talk through the HMD, are returned by the open call.
