# Windows Mixed Reality Headsets (macOS)

Status: **Home UI only, on this branch**. OXRSys Home can select a wired Windows Mixed Reality
headset, edit its settings, and manage the motion-controller Bluetooth adapter. The runtime-side
wired backend, the Monado-derived driver under `drivers/`, `oxrsys-headset-helper`, `wmr_btstack`
and `wmr_edid_override.py` land with the driver port; until then the runtime ignores the `[wired]`
config section and keeps using the streaming path.

## What This Is

Windows Mixed Reality headsets (HP Reverb G1/G2, Samsung Odyssey/Odyssey+, Lenovo Explorer, Dell
Visor, Acer AH100/AH101, Medion Erazer X1000, Fujitsu FMVHDS1, HP VR1000) have no tracking computer
on board. The host must talk to the headset over USB HID, switch the panel on, read the IMU, run
the cameras, and apply the per-device lens distortion itself.

[Monado](https://gitlab.freedesktop.org/monado/monado) has an open, permissively licensed (BSL-1.0)
driver for all of that, but its build only enables it on Linux and its HID layer is Linux `hidraw`
only. The driver port compiles that driver out of an unmodified, commit-pinned Monado checkout and
adds the small pieces macOS is missing.

Unlike a Quest or a Vision Pro, a WMR headset is not a streaming client: it is a panel and an IMU
on the end of a cable. With `wired.wired_headset = true` it replaces the streaming client entirely,
so nothing is encoded, sent over the network, or decoded.

## Configuration

Home writes the `[wired]` section of `~/Library/Application Support/OXRSys/oxrsys-runtime.toml`:

| Key | Default | Meaning |
| --- | --- | --- |
| `wired_headset` | `false` | Use a wired WMR headset instead of a streaming client. |
| `wired_display_id` | `0` | `CGDirectDisplayID` of the headset panel; `0` auto-detects. |
| `wired_eye_height_m` | `1.6` | Eye height above the floor (`1.0`-`2.2`): the fixed head height without positional tracking, and the starting height with it. |
| `wired_position_tracking` | `true` | 6DoF head tracking through Basalt when `libbasalt.dylib` is installed next to the headset helper; `false` keeps orientation-only tracking. |
| `wired_controller_adapter` | `false` | Drive WMR motion controllers from a separate USB Bluetooth adapter (below). |
| `wired_camera_monitor` | `true` | Open the headset helper's tracking-camera window on a desktop screen while the headset is in use. |

The UI for these is the Headset section at the top of Home's Streaming tab; see
[macos-home.md](macos-home.md).

## macOS hides the panel: the EDID override

This is the expected first failure, and it is a policy, not a fault. Every WMR headset's EDID
carries Microsoft's vendor block (OUI CA-12-5C) that declares the panel a non-desktop head-mounted
display. macOS reads it, builds a display pipe for the panel (visible in `ioreg` as a `dispext`
framebuffer with the right resolution), and then keeps the display off the desktop: it is never
returned by `CGGetOnlineDisplayList`, so nothing can be drawn on it.

Apple's display overrides can patch EDID bytes before they are interpreted (`edid-patches`, the same
mechanism Apple ships fixes with under `/System/Library/Displays`). `wmr_edid_override.py` reads the
EDID from IOKit, rewrites the Microsoft block's tag to a reserved value so it is skipped, fixes the
extension checksum, and writes the override:

```bash
python3 drivers/tools/wmr_edid_override.py                 # show the patch
sudo python3 drivers/tools/wmr_edid_override.py --install  # write it
```

Then unplug and replug the headset's video cable (or sleep and wake). To undo, delete the file the
tool printed under `/Library/Displays/Contents/Resources/Overrides/`.

Verified on a Dell Visor (EDID vendor `10ac`, product `7fce`): the block sits at byte 146 with usage
byte `0x07` (VR headset, no desktop use), and the patch changes two bytes.

## WMR controllers through a USB Bluetooth adapter

macOS's Bluetooth daemon reads a controller's service records before it starts pairing. A 1st-gen
WMR controller in pairing mode stops answering during that exchange (the link drops after the 5 s
supervision timeout and the LEDs go from flashing to solid), so pairing never begins; no
IOBluetooth API changes the order. Windows and BlueZ authenticate first, which is what the
controllers expect.

`wmr_btstack` does the same on a separate USB Bluetooth adapter that it drives itself with
[BTstack](https://github.com/bluekitchen/btstack) over libusb, bypassing macOS's stack:

```
  controller ── Bluetooth ── USB adapter ── libusb ── wmr_btstack
                                                          │ Unix socket
                                                          ▼
                                                 oxrsys-headset-helper
```

- **Adapter.** Any USB adapter with a Bluetooth HCI interface that works without a firmware upload:
  CSR8510 A10 (`0a12:0001`, verified) and Broadcom BCM20702. Realtek (`0bda:*`) and Intel
  (`8087:*`) adapters need vendor firmware that the tool doesn't load, and Home marks them as such.
  On Apple Silicon macOS never uses external adapters, so nothing has to be detached; on Intel Macs
  the adapter must not be the system Bluetooth controller. While the tool runs it owns the adapter
  exclusively.
- **Enable.** `wired_controller_adapter = true` in `[wired]` (Home: Streaming → Headset → Motion
  Controllers). Home keeps the tool running while the setting is on and a supported adapter is
  plugged in, so controllers can be paired before any game starts. It looks for `wmr_btstack` next
  to the selected runtime's dylib, then next to the registered one, and starts it detached with
  `-p 0` (reconnect only) at most once every ten seconds.
- **Pairing.** Pair each controller once: Home's `Pair Controller…` (or `wmr_btstack -p 120` from a
  terminal) opens a pairing window; take the battery cover off, hold the small pairing button until
  the lights flash, and it bonds (the controllers ask for legacy PIN `0000`, which the tool
  answers) and connects. Afterwards pressing the controller's Windows button reconnects it: outside
  a pairing window the tool only page-scans, at a high duty cycle, so those reconnects get through.
  `Forget Paired Controllers` (or `wmr_btstack -r`) drops all bondings.
- **Files** (all in `~/Library/Application Support/OXRSys/`): `wmr_bt.sock` (bridge socket),
  `wmr_controllers_status.json` (what Home reads: `process_id`, `adapter_state`
  `ready`/`no_adapter`, `adapter_address`, `pairing_seconds_left`, `runtime_attached`,
  `controllers[]` with `hand`, `name`, `address`, `state` `pairing`/`connecting`/`connected` and
  `reports_per_second`, and `paired[]`), `wmr_btstack_<adapter>.tlv` (link keys),
  `wmr_btstack_names.txt`, `wmr_btstack.log`, and `wmr_btstack_hci.pklg`, a PacketLogger HCI trace
  of the last run.
- **Limits.** The helper opens controllers once, when it opens the headset; one switched on later
  is used after the helper restarts. The controllers send motion reports at roughly 50-65 Hz over
  this link. Pose is 3DoF, from an arm model.

PlayStation Move controllers do not need any of this — they pair through macOS Bluetooth normally.

## Known Gaps

- The runtime-side wired backend is not on this branch, so `wired_headset = true` currently changes
  nothing outside Home.
- Controller pose is orientation-only plus an arm model.
- Home reports the first detected adapter only; it does not let you choose between several.
