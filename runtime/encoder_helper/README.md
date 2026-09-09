# Native-arm64 hardware HEVC encoder helper

## Why this exists

The OXRSys runtime dylib is `dlopen`ed **in-process by CrossOver's x86_64 Wine
host** (Rosetta). VideoToolbox **refuses the hardware HEVC encoder to an
x86_64/Rosetta process** — `VTCompressionSessionCreate` with
`RequireHardware=YES` fails `kVTCouldNotFindVideoEncoderErr (-12908)`, and with
`Require=NO` it silently falls back to the **software** encoder (~27–40 ms/frame,
which saturates the CPU at full resolution). Hardware **H.264** is granted under
Rosetta, but the Quest client only decodes HEVC, so that is not usable.

The only way to reach the hardware HEVC encoder is a **native-arm64 process**.
This helper is that process. All GPU composition (blit / downscale / foveation)
stays in the runtime; only the final `VTCompressionSessionEncodeFrame` moves
out-of-process. The runtime shares its compose IOSurfaces with the helper
**zero-copy**, so no frame pixels are ever copied between processes.

Measured on an M4 Pro (see `smoke_test.mm`): native-arm64 hardware HEVC encode
is **~8 ms/frame** at 2272×1264, vs ~27–40 ms for in-process software HEVC.

## Architecture

```
  x86_64 runtime dylib (Rosetta)                 native-arm64 helper process
  ------------------------------                 ---------------------------
  VideoEncoder                                    main.mm
    Metal compose -> IOSurface slot   ── mach ──►  IOSurfaceLookupFromMachPort
    HevcEncoderHelperClient                        CVPixelBufferCreateWithIOSurface
      posix_spawn(helper)                          VTCompressionSession
      bootstrap_check_in(name)                       RequireHardware = YES
      send Init         ───────── unix socket ─────► read Init
      recv child port   ◄──────── mach ────────────  send child port
      send N surfaces   ───────── mach ────────────► recv + wrap surfaces
      recv InitAck(hw)  ◄──────── unix socket ──────  create session, reply
      SubmitFrame(slot) ───────── unix socket ─────► VTCompressionSessionEncodeFrame
      onNal / onDone    ◄──────── unix socket ──────  Annex-B NAL units + metrics
```

* **Control + frame submission + encoded results:** a Unix stream socket
  (`EncoderHelperIpc.h`, explicit little-endian framing).
* **IOSurface transfer:** Mach send rights (`IOSurfaceCreateMachPort` →
  `IOSurfaceLookupFromMachPort`), transferred once at startup over a bootstrap
  rendezvous. The parent `bootstrap_check_in`s a per-spawn name *before*
  spawning; the child `bootstrap_look_up`s it (name passed via argv) and hands
  the parent a send right so surfaces can be pushed to it.
* **Timestamps** cross the boundary as int64 ns the parent supplies and the
  child echoes back; the child never generates a timestamp the parent compares
  (mach clocks are not comparable across the Rosetta boundary). Encode duration
  is measured entirely within the helper and reported as milliseconds.

> Do **not** switch the rendezvous to
> `posix_spawnattr_setspecialport_np(TASK_BOOTSTRAP_PORT)`: libxpc latches the
> bootstrap port during libSystem init, before `main()`, and the child's first
> XPC-touching call (IOSurface lookup / VideoToolbox) then hangs forever. The
> child must use the normally inherited bootstrap port.

## Build

The helper is arm64; the runtime dylib is x86_64. They are separate targets.

```sh
# Bulletproof single-file build (system frameworks only):
runtime/encoder_helper/build-helper.sh
# -> build/helper/oxrsys-encoder-helper   (arm64, ad-hoc signed)

# Or via CMake (own build dir, arm64):
cmake -S runtime/encoder_helper -B build/helper -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build build/helper
lipo -info build/helper/oxrsys-encoder-helper   # must say: arm64
```

The runtime-side glue (`../src/HevcEncoderHelperClient.mm`) builds as part of the
normal x86_64 `oxrsys_runtime` target.

## Enable

Off by default. In `~/Library/Application Support/OXRSys/oxrsys-runtime.toml`:

```toml
encoder_helper = true
# encoder_helper_path = ""   # empty = sibling of the runtime dylib
```

Deploy `oxrsys-encoder-helper` next to `liboxrsys-runtime.dylib` (or set
`encoder_helper_path` / `$OXRSYS_ENCODER_HELPER_PATH`). Both must be
`codesign --force --sign -` ad-hoc signed.

The runtime log then shows:
`VideoEncoder: hardware=YES (via native-arm64 out-of-process helper)` and
`EncoderHelper: ready — hardware HEVC encoder live in native-arm64 helper`.
On any failure it logs where it broke and continues on the software encoder.

## Offline test

`smoke_test.mm` reproduces the whole handoff without Wine/the game: an x86_64
parent (to mimic the Rosetta runtime) creates IOSurface-backed BGRA buffers,
spawns the arm64 helper, performs the mach rendezvous + surface transfer, and
encodes frames — printing `hardware=YES` and the per-frame encode time.

```sh
xcrun clang++ -arch x86_64 -std=c++17 -O2 runtime/encoder_helper/smoke_test.mm \
  -o build/helper/smoke_test_x64 \
  -framework Foundation -framework CoreFoundation -framework CoreVideo \
  -framework CoreMedia -framework IOSurface
codesign --force --sign - build/helper/smoke_test_x64
build/helper/smoke_test_x64 build/helper/oxrsys-encoder-helper
```
