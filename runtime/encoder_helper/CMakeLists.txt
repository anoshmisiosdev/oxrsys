# SPDX-License-Identifier: MPL-2.0
#
# Standalone native-arm64 HEVC encoder helper.
#
# This is a SEPARATE project from the runtime dylib on purpose:
#  * The runtime dylib must be x86_64 (it is dlopen'd by the x86_64/Rosetta Wine
#    host). This helper must be arm64 (only a native-arm64 process is granted the
#    hardware HEVC encoder). Keeping it a standalone project lets it be configured
#    with CMAKE_OSX_ARCHITECTURES=arm64 independently, and built in its own build
#    directory so it never contends with the runtime's build/x86 tree.
#  * It links ONLY system frameworks — no spdlog/glm/OpenXR — so there is no risk
#    of pulling an x86_64 FetchContent artifact into an arm64 link.
#
# Configure + build (from the repo root):
#   cmake -S runtime/encoder_helper -B build/helper -G Ninja \
#         -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
#   cmake --build build/helper
#   lipo -info build/helper/oxrsys-encoder-helper   # must say: arm64

cmake_minimum_required(VERSION 3.20)
project(oxrsys_encoder_helper LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_OSX_ARCHITECTURES)
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Helper architecture" FORCE)
endif()

add_executable(oxrsys-encoder-helper
    main.mm
)

# arm64 is required for the hardware encoder; guard against an accidental
# universal/x86_64 configure.
set_target_properties(oxrsys-encoder-helper PROPERTIES
    OSX_ARCHITECTURES "arm64"
)

target_compile_options(oxrsys-encoder-helper PRIVATE -Wall -Wextra)

target_link_libraries(oxrsys-encoder-helper PRIVATE
    "-framework Foundation"
    "-framework CoreFoundation"
    "-framework CoreMedia"
    "-framework CoreVideo"
    "-framework VideoToolbox"
    "-framework IOSurface"
)
