# SPDX-License-Identifier: MPL-2.0
#
# oxrsys-headset-helper: owns a wired (Windows Mixed Reality) headset, its
# display, tracking and the idle lobby; the runtime connects to it over a
# Unix socket and shares frame surfaces. Native arm64 only.

add_executable(oxrsys-headset-helper
    main.mm
    CameraMonitor.mm
)
target_compile_options(oxrsys-headset-helper PRIVATE -Wall -Wextra -Wno-unused-parameter -fobjc-arc)
target_include_directories(oxrsys-headset-helper PRIVATE
    ${CMAKE_SOURCE_DIR}/common/protocol/include
)
target_link_libraries(oxrsys-headset-helper PRIVATE
    oxrsys_monado_wmr
    "-framework AppKit"
    "-framework CoreGraphics"
    "-framework Foundation"
    "-framework IOSurface"
    "-framework Metal"
    "-framework QuartzCore"
)

# The camera monitor (--monitor) loads the VIT monitor shim from next to the
# helper executable, so the shim is built with the helper and lands in the
# same directory. Deploy it next to the helper as well.
set_target_properties(oxrsys_vit_monitor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
add_dependencies(oxrsys-headset-helper oxrsys_vit_monitor)
# Deploy next to liboxrsys-runtime.dylib (the runtime looks for it there) and
# ad-hoc sign it like the encoder helper.
