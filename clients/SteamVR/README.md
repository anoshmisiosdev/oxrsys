<!-- SPDX-License-Identifier: MPL-2.0 -->

# OXRSys SteamVR driver

`driver_oxrsys.dll` — a SteamVR (OpenVR) driver that makes OXRSys appear to SteamVR as a
head-mounted display, so an unmodified SteamVR install running under CrossOver can drive
the runtime.

```bash
./build.sh
```

Cross-compiles to a Windows x86-64 DLL with mingw-w64 and lays out a driver directory
named `oxrsys`. See [../../docs/steamvr-driver.md](../../docs/steamvr-driver.md) for how it
works, why it uses direct mode, the MSVC/GCC vtable constraint, and how to install it into
a bottle.

`openvr/openvr_driver.h` is Valve's driver header, vendored unchanged under its own
license.
