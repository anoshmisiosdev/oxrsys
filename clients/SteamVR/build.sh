#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
#
# Cross-compiles the OXRSys SteamVR driver to a Windows x86-64 DLL.
#
# mingw-w64 GCC, not MSVC: the driver is loaded by Valve's MSVC-built
# vrserver.exe, and src/OpenVRMsAbi.h restates the affected interfaces with
# GCC-specific signatures so the two agree on the vtable. Building with a
# different compiler would need that header revisited.
#
# Usage: ./build.sh [output-directory]

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out_dir="${1:-${script_dir}/build}"
driver_dir="${out_dir}/oxrsys"
bin_dir="${driver_dir}/bin/win64"

cxx="${OXRSYS_MINGW_CXX:-x86_64-w64-mingw32-g++}"

# OpenXR headers. The runtime build fetches the SDK with CMake FetchContent, so
# reuse that checkout when one is present; OXRSYS_OPENXR_INCLUDE overrides it.
openxr_include="${OXRSYS_OPENXR_INCLUDE:-}"
if [ -z "${openxr_include}" ]; then
    for candidate in "${script_dir}"/../../build/*/_deps/openxr-src/include; do
        if [ -d "${candidate}" ]; then
            openxr_include="${candidate}"
            break
        fi
    done
fi
if [ ! -d "${openxr_include}" ]; then
    echo "error: OpenXR headers not found; configure the runtime once or set OXRSYS_OPENXR_INCLUDE" >&2
    exit 1
fi
if ! command -v "${cxx}" >/dev/null 2>&1; then
    echo "error: ${cxx} not found; install mingw-w64" >&2
    exit 1
fi

mkdir -p "${bin_dir}"

"${cxx}" \
    -std=c++20 \
    -O2 \
    -shared \
    -static-libgcc \
    -static-libstdc++ \
    -Wall \
    -Wextra \
    -Wno-unknown-pragmas \
    -fno-rtti \
    -isystem "${script_dir}/openvr" \
    -isystem "${openxr_include}" \
    -I"${script_dir}/src" \
    "${script_dir}/src/ServerDriver.cpp" \
    "${script_dir}/src/HmdDevice.cpp" \
    "${script_dir}/src/DirectModeComponent.cpp" \
    "${script_dir}/src/DriverLog.cpp" \
    "${script_dir}/src/OxrClient.cpp" \
    -o "${bin_dir}/driver_oxrsys.dll" \
    -static \
    -ld3d11 \
    -ldxgi \
    -Wl,--enable-stdcall-fixup

cp -R "${script_dir}/resources/." "${driver_dir}/"

echo "built ${bin_dir}/driver_oxrsys.dll"
echo "driver directory: ${driver_dir}"
