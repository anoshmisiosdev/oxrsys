# SPDX-License-Identifier: MPL-2.0
#
# Wired-headset device drivers borrowed from Monado.
#
# Monado's own build gates the Windows Mixed Reality driver on Linux and drags in
# its compositor and IPC layers, so this file compiles the driver sources
# directly out of a pinned, unmodified Monado checkout instead of calling
# add_subdirectory() on it. Everything platform specific that Monado lacks for
# macOS (a HID backend, device enumeration) lives in drivers/monado/.

# The option itself is declared in the top-level CMakeLists.txt, before the
# runtime, so the runtime can test it.
if(NOT OXRSYS_BUILD_WMR_DRIVER)
    return()
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(OXRSYS_HIDAPI REQUIRED IMPORTED_TARGET hidapi)
pkg_check_modules(OXRSYS_LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

# Optional OpenCV, needed by Monado's camera-based (controller/SLAM) tracking
# code. Off by default so the plain headset build stays dependency-light.
# macOS: `brew install opencv`. Note that the x86_64 (Rosetta) runtime build
# used by the Wine bridge needs an x86_64 OpenCV, for example from an x86_64
# Homebrew under /usr/local, since Apple Silicon Homebrew libraries under
# /opt/homebrew are arm64 only.
option(OXRSYS_WMR_OPENCV "Link the Monado WMR driver against OpenCV" OFF)
if(OXRSYS_WMR_OPENCV)
    find_package(OpenCV REQUIRED)
endif()

include(FetchContent)

# Pinned Monado main (BSL-1.0). Bump deliberately: the file lists below track
# the driver's source layout at this revision. SOURCE_SUBDIR points at a
# directory that does not exist so FetchContent only populates the tree and
# never runs Monado's top-level CMakeLists.txt.
FetchContent_Declare(
    monado
    GIT_REPOSITORY https://gitlab.freedesktop.org/monado/monado.git
    GIT_TAG a6d31c6c1ce5ab54c6a897328f963571a757e4e7
    GIT_SHALLOW OFF
    SOURCE_SUBDIR oxrsys-does-not-add-subdirectory
)

# Eigen is header-only; Monado's math and IMU fusion code needs it.
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW ON
    SOURCE_SUBDIR oxrsys-does-not-add-subdirectory
)

FetchContent_MakeAvailable(monado eigen)

set(OXRSYS_MONADO_XRT "${monado_SOURCE_DIR}/src/xrt")
set(OXRSYS_MONADO_EXTERNAL "${monado_SOURCE_DIR}/src/external")
set(OXRSYS_MONADO_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/monado-generated")
file(MAKE_DIRECTORY "${OXRSYS_MONADO_GENERATED}/xrt")

# --- Configuration headers -------------------------------------------------
# Generated from Monado's own templates so the driver sees exactly the macros
# it expects. Only the features we build are switched on.
set(XRT_HAVE_LIBUSB ON)
set(XRT_HAVE_HIDAPI ON)
if(OXRSYS_WMR_OPENCV)
    set(XRT_HAVE_OPENCV ON)
endif()
set(FILE_CONTENTS "#define XRT_BUILD_DRIVER_WMR\n#define XRT_BUILD_DRIVER_PSMV\n")
foreach(oxrsys_monado_config IN ITEMS build have drivers vulkan android)
    configure_file(
        "${OXRSYS_MONADO_XRT}/include/xrt/xrt_config_${oxrsys_monado_config}.h.cmake_in"
        "${OXRSYS_MONADO_GENERATED}/xrt/xrt_config_${oxrsys_monado_config}.h"
        @ONLY
    )
endforeach()
configure_file(
    "${OXRSYS_MONADO_XRT}/include/xrt/xrt_openxr_config.h.cmake_in"
    "${OXRSYS_MONADO_GENERATED}/xrt/xrt_openxr_config.h"
    @ONLY
)

# X-macro enum lists used by u_pretty_print / u_str_to_enum.
include("${monado_SOURCE_DIR}/cmake/GenerateMacroLists.cmake")
generate_macro_lists(
    OUTPUT "${OXRSYS_MONADO_GENERATED}/xrt/xrt_macro_lists.h.inc"
    INPUTS
        "${OXRSYS_MONADO_XRT}/include/xrt/xrt_defines.h"
        "${OXRSYS_MONADO_XRT}/include/xrt/xrt_device.h"
        "${OXRSYS_MONADO_XRT}/include/xrt/xrt_results.h"
    ENUMS xrt_device_feature_type xrt_result xrt_device_name xrt_input_name xrt_output_name
    IGNORE_OTHER_ENUMS
    WORKING_DIRECTORY "${OXRSYS_MONADO_XRT}/include/xrt"
)
add_custom_target(oxrsys_monado_macro_lists
    DEPENDS "${OXRSYS_MONADO_GENERATED}/xrt/xrt_macro_lists.h.inc"
)

# --- Driver library --------------------------------------------------------
set(OXRSYS_MONADO_WMR_SOURCES
    # drivers/wmr: everything except wmr_prober.c, which needs Monado's
    # udev/libusb prober. drivers/monado/wmr_macos.c replaces it.
    drivers/wmr/wmr_bt_controller.c
    drivers/wmr/wmr_camera.c
    drivers/wmr/wmr_config.c
    drivers/wmr/wmr_controller.c
    drivers/wmr/wmr_controller_base.c
    drivers/wmr/wmr_controller_hp.c
    drivers/wmr/wmr_controller_og.c
    drivers/wmr/wmr_controller_protocol.c
    drivers/wmr/wmr_hmd.c
    drivers/wmr/wmr_hmd_controller.c
    drivers/wmr/wmr_protocol.c
    drivers/wmr/wmr_source.c

    # drivers/psmv: PlayStation Move over Bluetooth HID. Monado gates it on
    # Linux + hidapi; drivers/monado/psmv_macos.c does the enumeration here.
    drivers/psmv/psmv_driver.c

    auxiliary/os/os_time.cpp

    auxiliary/math/m_base.cpp
    auxiliary/math/m_clock_tracking.c
    auxiliary/math/m_filter_fifo.c
    auxiliary/math/m_filter_one_euro.c
    auxiliary/math/m_hash.cpp
    auxiliary/math/m_imu_3dof.c
    auxiliary/math/m_imu_pre.c
    auxiliary/math/m_lowpass_float.cpp
    auxiliary/math/m_lowpass_integer.cpp
    auxiliary/math/m_matrix_projection.cpp
    auxiliary/math/m_optics.c
    auxiliary/math/m_permutation.c
    auxiliary/math/m_predict.c
    auxiliary/math/m_quatexpmap.cpp
    auxiliary/math/m_relation_history.cpp
    auxiliary/math/m_space.cpp

    auxiliary/tracking/t_camera_models.cpp
    auxiliary/tracking/t_data_utils.c
    auxiliary/tracking/t_dead_reckoning.c
    auxiliary/tracking/t_imu.cpp

    auxiliary/util/u_autoexpgain.c
    auxiliary/util/u_bitwise.c
    auxiliary/util/u_config_json.c
    auxiliary/util/u_debug.c
    auxiliary/util/u_device.c
    auxiliary/util/u_device_id.c
    auxiliary/util/u_device_ni.c
    auxiliary/util/u_distortion.c
    auxiliary/util/u_distortion_mesh.c
    auxiliary/util/u_extension_list.cpp
    auxiliary/util/u_file.c
    auxiliary/util/u_file.cpp
    auxiliary/util/u_format.c
    auxiliary/util/u_frame.c
    auxiliary/util/u_hashmap.cpp
    auxiliary/util/u_hashset.cpp
    auxiliary/util/u_id_ringbuffer.cpp
    auxiliary/util/u_imu_sink_force_monotonic.c
    auxiliary/util/u_imu_sink_split.c
    auxiliary/util/u_json.c
    auxiliary/util/u_limited_unique_id.cpp
    auxiliary/util/u_logging.c
    auxiliary/util/u_misc.c
    auxiliary/util/u_pretty_print.c
    auxiliary/util/u_sink_blob_visualizer.c
    auxiliary/util/u_sink_combiner.c
    auxiliary/util/u_sink_converter.c
    auxiliary/util/u_sink_deinterleaver.c
    auxiliary/util/u_sink_force_genlock.c
    auxiliary/util/u_sink_queue.c
    auxiliary/util/u_sink_quirk.c
    auxiliary/util/u_sink_simple_queue.c
    auxiliary/util/u_sink_split.c
    auxiliary/util/u_sink_stereo_sbs_split.c
    auxiliary/util/u_str_to_enum.c
    auxiliary/util/u_time.cpp
    auxiliary/util/u_trace_marker.c
    auxiliary/util/u_tracked_imu_3dof.c
    auxiliary/util/u_var.cpp
    auxiliary/util/u_visibility_mask.c
)
list(TRANSFORM OXRSYS_MONADO_WMR_SOURCES PREPEND "${OXRSYS_MONADO_XRT}/")

# Camera-based tracking (Monado's aux_tracking OpenCV half plus the PS Move
# sphere tracker), only with OpenCV.
if(OXRSYS_WMR_OPENCV)
    set(OXRSYS_MONADO_OPENCV_SOURCES
        auxiliary/tracking/t_calibration.cpp
        auxiliary/tracking/t_convert.cpp
        auxiliary/tracking/t_debug_hsv_filter.cpp
        auxiliary/tracking/t_debug_hsv_picker.cpp
        auxiliary/tracking/t_debug_hsv_viewer.cpp
        auxiliary/tracking/t_file.cpp
        auxiliary/tracking/t_frame_cv_mat_wrapper.cpp
        auxiliary/tracking/t_hsv_filter.c
        auxiliary/tracking/t_kalman.cpp
        auxiliary/tracking/t_tracker_psmv.cpp
    )
    list(TRANSFORM OXRSYS_MONADO_OPENCV_SOURCES PREPEND "${OXRSYS_MONADO_XRT}/")
    list(APPEND OXRSYS_MONADO_WMR_SOURCES ${OXRSYS_MONADO_OPENCV_SOURCES})
endif()

set(OXRSYS_MONADO_PLATFORM_SOURCES
    monado/os_hid_hidapi.c
    monado/psmv_macos.c
    monado/t_euroc_recorder_stub.c
    monado/u_file_macos.c
    monado/wmr_macos.c
    monado/wmr_psmv_tracking.c
)
if(APPLE)
    # Objective-C++: presents warped eye images on the panel through Metal.
    list(APPEND OXRSYS_MONADO_PLATFORM_SOURCES monado/wmr_panel.mm)
    set_source_files_properties(monado/wmr_panel.mm PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-fobjc-arc")
endif()

add_library(oxrsys_monado_wmr STATIC
    ${OXRSYS_MONADO_WMR_SOURCES}
    ${OXRSYS_MONADO_PLATFORM_SOURCES}
    "${OXRSYS_MONADO_EXTERNAL}/cjson/cjson/cJSON.c"
)
add_dependencies(oxrsys_monado_wmr oxrsys_monado_macro_lists)

target_include_directories(oxrsys_monado_wmr
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/monado"
        "${OXRSYS_MONADO_GENERATED}"
        "${OXRSYS_MONADO_XRT}/include"
        "${OXRSYS_MONADO_XRT}/auxiliary"
        "${OXRSYS_MONADO_XRT}/drivers"
    PRIVATE
        "${OXRSYS_MONADO_EXTERNAL}/cjson"
        "${OXRSYS_MONADO_EXTERNAL}/flexkalman"
        "${OXRSYS_MONADO_EXTERNAL}/util-headers"
        "${OXRSYS_MONADO_EXTERNAL}/bcdec"
)
target_include_directories(oxrsys_monado_wmr SYSTEM PRIVATE "${eigen_SOURCE_DIR}")
if(OXRSYS_WMR_OPENCV)
    target_include_directories(oxrsys_monado_wmr SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

target_compile_definitions(oxrsys_monado_wmr
    PRIVATE
        CJSON_HIDE_SYMBOLS
        _GNU_SOURCE
)
# The runtime dylib is loaded into arbitrary apps; keep Monado's symbols out of
# its export table.
target_compile_options(oxrsys_monado_wmr PRIVATE -fvisibility=hidden)
target_compile_features(oxrsys_monado_wmr PRIVATE c_std_11 cxx_std_20)

# Third-party code is compiled warning-free rather than fixed up locally.
set_source_files_properties(
    ${OXRSYS_MONADO_WMR_SOURCES}
    "${OXRSYS_MONADO_EXTERNAL}/cjson/cjson/cJSON.c"
    PROPERTIES COMPILE_OPTIONS "-w"
)
set_source_files_properties(
    monado/os_hid_hidapi.c
    monado/psmv_macos.c
    monado/t_euroc_recorder_stub.c
    monado/u_file_macos.c
    monado/wmr_macos.c
    monado/wmr_psmv_tracking.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wno-unused-parameter"
)

target_link_libraries(oxrsys_monado_wmr
    PUBLIC
        PkgConfig::OXRSYS_HIDAPI
        PkgConfig::OXRSYS_LIBUSB
        Threads::Threads
)
if(OXRSYS_WMR_OPENCV)
    target_link_libraries(oxrsys_monado_wmr PUBLIC ${OpenCV_LIBS})
endif()
if(APPLE)
    target_link_libraries(oxrsys_monado_wmr
        PUBLIC
            "-framework AppKit"
            "-framework CoreGraphics"
            "-framework Metal"
            "-framework QuartzCore"
    )
endif()

# --- Probe tool ------------------------------------------------------------
# Opens the first Windows Mixed Reality headset on the USB bus and prints its
# IMU-driven orientation. This is the hardware bring-up check for the driver.
add_executable(oxrsys_wmr_probe tools/wmr_probe.c)
target_compile_features(oxrsys_wmr_probe PRIVATE c_std_11)
# -Wno-unused-parameter: Monado's inline os_time.h / os_threading.h helpers trip it.
target_compile_options(oxrsys_wmr_probe PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(oxrsys_wmr_probe PRIVATE oxrsys_monado_wmr)

# --- OpenXR smoke tool ------------------------------------------------------
# Native OpenXR client (loader -> runtime -> WiredHeadset): the end-to-end
# check that a wired headset works through the runtime.
if(APPLE)
    add_executable(oxrsys_wmr_xr_smoke tools/wmr_xr_smoke.mm)
    target_compile_options(oxrsys_wmr_xr_smoke PRIVATE -Wall -Wextra -fobjc-arc)
    target_compile_definitions(oxrsys_wmr_xr_smoke PRIVATE XR_USE_GRAPHICS_API_METAL)
    target_include_directories(oxrsys_wmr_xr_smoke PRIVATE "${openxr_SOURCE_DIR}/include")
    target_link_libraries(oxrsys_wmr_xr_smoke PRIVATE
        OpenXR::openxr_loader
        "-framework Foundation"
        "-framework Metal"
    )
endif()

# --- Display tool ----------------------------------------------------------
# Captures the headset's panel as a macOS display and renders a
# distortion-corrected stereo test scene onto it with Metal.
if(APPLE)
    add_executable(oxrsys_wmr_display tools/wmr_display.mm)
    # -Wno-unused-parameter: Monado's inline os_time.h helpers trip it.
    target_compile_options(oxrsys_wmr_display PRIVATE -Wall -Wextra -Wno-unused-parameter -fobjc-arc)
    target_link_libraries(oxrsys_wmr_display PRIVATE
        oxrsys_monado_wmr
        "-framework AppKit"
        "-framework CoreGraphics"
        "-framework ImageIO"
        "-framework Metal"
        "-framework QuartzCore"
    )
endif()
