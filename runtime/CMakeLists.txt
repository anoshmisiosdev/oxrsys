# SPDX-License-Identifier: MPL-2.0

set(OXRSYS_RUNTIME_SOURCES
    src/Config.cpp
    src/EntryPoint.cpp
    src/Runtime.cpp
    src/RuntimeStatus.cpp
    src/RuntimePlatform.cpp
    src/RuntimeSockets.cpp
    src/StreamingFrameQueue.cpp
    src/VideoBitstream.cpp
    src/VideoCodecSelection.cpp
    src/Instance.cpp
    src/Session.cpp
    src/Space.cpp
    src/ActionSet.cpp
    src/InputManager.cpp
    src/HandTracker.cpp
    src/StreamingServer.cpp
    src/TrackingReceiver.cpp
    src/VulkanDispatch.cpp
)

set(OXRSYS_VIDEO_ENCODER "AUTO" CACHE STRING "Runtime video encoder: AUTO, VIDEOTOOLBOX, MEDIAFOUNDATION, VAAPI, or STUB")
set_property(CACHE OXRSYS_VIDEO_ENCODER PROPERTY STRINGS AUTO VIDEOTOOLBOX MEDIAFOUNDATION VAAPI STUB)
string(TOUPPER "${OXRSYS_VIDEO_ENCODER}" OXRSYS_VIDEO_ENCODER_NORMALIZED)
if(NOT OXRSYS_VIDEO_ENCODER_NORMALIZED MATCHES "^(AUTO|VIDEOTOOLBOX|MEDIAFOUNDATION|VAAPI|STUB)$")
    message(FATAL_ERROR "OXRSYS_VIDEO_ENCODER must be AUTO, VIDEOTOOLBOX, MEDIAFOUNDATION, VAAPI, or STUB")
endif()

set(OXRSYS_VIDEO_ENCODER_SELECTED "${OXRSYS_VIDEO_ENCODER_NORMALIZED}")
if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "AUTO")
    if(APPLE)
        set(OXRSYS_VIDEO_ENCODER_SELECTED "VIDEOTOOLBOX")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(OXRSYS_VIDEO_ENCODER_SELECTED "VAAPI")
    elseif(WIN32)
        set(OXRSYS_VIDEO_ENCODER_SELECTED "MEDIAFOUNDATION")
    else()
        set(OXRSYS_VIDEO_ENCODER_SELECTED "STUB")
    endif()
endif()

set(OXRSYS_RUNTIME_ENABLE_OPENGL OFF)
set(OXRSYS_RUNTIME_NEEDS_VAAPI OFF)

if(APPLE)
    list(APPEND OXRSYS_RUNTIME_SOURCES
        src/SwapchainCommon.cpp
        src/SwapchainVulkan.cpp
        src/SwapchainMetal.mm
    )
    if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "VIDEOTOOLBOX")
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/VideoEncoder.mm
        )
    elseif(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "STUB")
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/StubVideoEncoder.cpp
        )
    else()
        message(FATAL_ERROR "OXRSYS_VIDEO_ENCODER=${OXRSYS_VIDEO_ENCODER_SELECTED} is not supported on Apple platforms")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "VIDEOTOOLBOX" OR
       OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "MEDIAFOUNDATION")
        message(FATAL_ERROR "OXRSYS_VIDEO_ENCODER=${OXRSYS_VIDEO_ENCODER_SELECTED} is not supported on Linux")
    endif()
    list(APPEND OXRSYS_RUNTIME_SOURCES
        src/SwapchainCommon.cpp
        src/SwapchainVulkan.cpp
        src/SwapchainOpenGL.cpp
    )
    set(OXRSYS_RUNTIME_ENABLE_OPENGL ON)
    if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "VAAPI")
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/VideoFramePreparation.cpp
            src/VaapiVideoEncoder.cpp
        )
        set(OXRSYS_RUNTIME_NEEDS_VAAPI ON)
    else()
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/StubVideoEncoder.cpp
        )
    endif()
elseif(WIN32)
    if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "VIDEOTOOLBOX" OR
       OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "VAAPI")
        message(FATAL_ERROR "OXRSYS_VIDEO_ENCODER=${OXRSYS_VIDEO_ENCODER_SELECTED} is not supported on Windows")
    endif()
    list(APPEND OXRSYS_RUNTIME_SOURCES
        src/SwapchainCommon.cpp
        src/SwapchainVulkan.cpp
        src/SwapchainD3D11.cpp
        src/SwapchainD3D12.cpp
    )
    if(OXRSYS_VIDEO_ENCODER_SELECTED STREQUAL "MEDIAFOUNDATION")
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/VideoFramePreparation.cpp
            src/MediaFoundationVideoEncoder.cpp
        )
    else()
        list(APPEND OXRSYS_RUNTIME_SOURCES
            src/StubVideoEncoder.cpp
        )
    endif()
else()
    message(FATAL_ERROR
        "The ${CMAKE_SYSTEM_NAME} runtime backend is not supported in this pass. "
        "Use macOS, Linux, or Windows for runtime builds."
    )
endif()

add_library(oxrsys_runtime SHARED
    ${OXRSYS_RUNTIME_SOURCES}
)

find_package(Vulkan REQUIRED)
if(OXRSYS_RUNTIME_ENABLE_OPENGL)
    find_package(OpenGL REQUIRED)
    find_package(X11 REQUIRED)
endif()

set(OXRSYS_RUNTIME_LINK_LIBRARIES
    spdlog::spdlog
    glm::glm
    ${CMAKE_DL_LIBS}
)

set(OXRSYS_LIBVA_PROVIDER "AUTO" CACHE STRING "Linux VA-API dependency provider: AUTO, SYSTEM, or FETCH_HEADERS")
set_property(CACHE OXRSYS_LIBVA_PROVIDER PROPERTY STRINGS AUTO SYSTEM FETCH_HEADERS)
string(TOUPPER "${OXRSYS_LIBVA_PROVIDER}" OXRSYS_LIBVA_PROVIDER_NORMALIZED)
if(NOT OXRSYS_LIBVA_PROVIDER_NORMALIZED MATCHES "^(AUTO|SYSTEM|FETCH_HEADERS)$")
    message(FATAL_ERROR "OXRSYS_LIBVA_PROVIDER must be AUTO, SYSTEM, or FETCH_HEADERS")
endif()

if(OXRSYS_RUNTIME_NEEDS_VAAPI)
    set(OXRSYS_VAAPI_TARGET)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND AND NOT OXRSYS_LIBVA_PROVIDER_NORMALIZED STREQUAL "FETCH_HEADERS")
        pkg_check_modules(OXRSYS_VAAPI_PKG QUIET IMPORTED_TARGET
            libva
            libva-drm
        )
        if(OXRSYS_VAAPI_PKG_FOUND)
            set(OXRSYS_VAAPI_TARGET PkgConfig::OXRSYS_VAAPI_PKG)
            message(STATUS "Using VA-API through pkg-config")
        endif()
    endif()

    if(NOT OXRSYS_VAAPI_TARGET)
        if(OXRSYS_LIBVA_PROVIDER_NORMALIZED STREQUAL "SYSTEM")
            message(FATAL_ERROR "VA-API development files were not found through pkg-config")
        endif()
        include(FetchContent)
        FetchContent_Declare(
            libva_headers
            GIT_REPOSITORY https://github.com/intel/libva.git
            GIT_TAG 2.24.0
            GIT_SHALLOW ON
        )
        FetchContent_GetProperties(libva_headers)
        if(NOT libva_headers_POPULATED)
            FetchContent_Populate(libva_headers)
        endif()
        find_library(OXRSYS_VA_LIBRARY
            NAMES va libva.so.2
            PATHS /usr/lib64 /usr/lib /lib64 /lib
        )
        find_library(OXRSYS_VA_DRM_LIBRARY
            NAMES va-drm libva-drm.so.2
            PATHS /usr/lib64 /usr/lib /lib64 /lib
        )
        if(NOT OXRSYS_VA_LIBRARY OR NOT OXRSYS_VA_DRM_LIBRARY)
            message(FATAL_ERROR
                "VA-API runtime libraries were not found. Install libva and libva-drm, "
                "or install libva development packages so pkg-config can resolve them."
            )
        endif()
        set(OXRSYS_LIBVA_GENERATED_INCLUDE_DIR
            "${CMAKE_CURRENT_BINARY_DIR}/generated/libva/include"
        )
        file(MAKE_DIRECTORY "${OXRSYS_LIBVA_GENERATED_INCLUDE_DIR}/va")
        set(VA_API_MAJOR_VERSION 1)
        set(VA_API_MINOR_VERSION 24)
        set(VA_API_MICRO_VERSION 0)
        set(VA_API_VERSION "1.24.0")
        configure_file(
            "${libva_headers_SOURCE_DIR}/va/va_version.h.in"
            "${OXRSYS_LIBVA_GENERATED_INCLUDE_DIR}/va/va_version.h"
            @ONLY
        )
        configure_file(
            "${libva_headers_SOURCE_DIR}/va/drm/va_drm.h"
            "${OXRSYS_LIBVA_GENERATED_INCLUDE_DIR}/va/va_drm.h"
            COPYONLY
        )
        add_library(oxrsys_vaapi_headers INTERFACE)
        target_include_directories(oxrsys_vaapi_headers INTERFACE
            ${OXRSYS_LIBVA_GENERATED_INCLUDE_DIR}
            ${libva_headers_SOURCE_DIR}
        )
        target_link_libraries(oxrsys_vaapi_headers INTERFACE
            ${OXRSYS_VA_LIBRARY}
            ${OXRSYS_VA_DRM_LIBRARY}
        )
        add_library(OXRSYS::VAAPI ALIAS oxrsys_vaapi_headers)
        set(OXRSYS_VAAPI_TARGET OXRSYS::VAAPI)
        message(STATUS "Using fetched libva headers with system VA-API runtime libraries")
    endif()

    list(APPEND OXRSYS_RUNTIME_LINK_LIBRARIES
        ${OXRSYS_VAAPI_TARGET}
    )
endif()

if(WIN32)
    list(APPEND OXRSYS_RUNTIME_LINK_LIBRARIES
        d3d11
        d3d12
        dxgi
        mf
        mfplat
        mfuuid
        wmcodecdspuuid
        propsys
        ole32
        ws2_32
    )
endif()

target_include_directories(oxrsys_runtime PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/common/protocol/include
    ${openxr_SOURCE_DIR}/include
    ${openxr_SOURCE_DIR}/src/common
    ${Vulkan_INCLUDE_DIRS}
)

target_compile_definitions(oxrsys_runtime PRIVATE
    XR_USE_GRAPHICS_API_VULKAN
    OXRSYS_USE_VULKAN
    OXRSYS_VIDEO_ENCODER_${OXRSYS_VIDEO_ENCODER_SELECTED}
    OXRSYS_VERSION_MAJOR=${OXRSYS_VERSION_MAJOR}
    OXRSYS_VERSION_MINOR=${OXRSYS_VERSION_MINOR}
    OXRSYS_VERSION_PATCH=${OXRSYS_VERSION_PATCH}
)

if(APPLE)
    target_compile_definitions(oxrsys_runtime PRIVATE
        XR_USE_GRAPHICS_API_METAL
        OXRSYS_USE_METAL
    )
endif()

if(OXRSYS_RUNTIME_ENABLE_OPENGL)
    target_compile_definitions(oxrsys_runtime PRIVATE
        XR_USE_GRAPHICS_API_OPENGL
        XR_USE_PLATFORM_XLIB
        OXRSYS_USE_OPENGL_XLIB
        GL_GLEXT_PROTOTYPES
    )
endif()

if(WIN32)
    target_compile_definitions(oxrsys_runtime PRIVATE
        XR_USE_PLATFORM_WIN32
        XR_USE_GRAPHICS_API_D3D11
        XR_USE_GRAPHICS_API_D3D12
        OXRSYS_USE_D3D11
        OXRSYS_USE_D3D12
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
endif()

# NOTE: We intentionally do NOT link against Vulkan::Vulkan.
# Linking the Vulkan loader would load the system MoltenVK ICD, which conflicts
# with app-embedded MoltenVK (e.g. Godot). Instead, all Vulkan functions are
# resolved dynamically through the app's pfnGetInstanceProcAddr.
target_link_libraries(oxrsys_runtime PRIVATE
    ${OXRSYS_RUNTIME_LINK_LIBRARIES}
)

if(OXRSYS_RUNTIME_ENABLE_OPENGL)
    target_link_libraries(oxrsys_runtime PRIVATE
        OpenGL::GL
        X11::X11
    )
endif()

if(APPLE)
    target_link_libraries(oxrsys_runtime PRIVATE
        "-framework Metal"
        "-framework MetalPerformanceShaders"
        "-framework Foundation"
        "-framework QuartzCore"
        "-framework AppKit"
        "-framework VideoToolbox"
        "-framework CoreMedia"
        "-framework CoreVideo"
    )
endif()

# Only export the negotiation entry point
set_target_properties(oxrsys_runtime PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    OUTPUT_NAME oxrsys-runtime
    PREFIX "lib"
)

if(APPLE)
    set_target_properties(oxrsys_runtime PROPERTIES
        OBJCXX_VISIBILITY_PRESET hidden
        SUFFIX ".dylib"
    )
endif()

if(WIN32)
    set_target_properties(oxrsys_runtime PROPERTIES
        PREFIX ""
    )
endif()

set(OXRSYS_RUNTIME_LIBRARY_FILENAME "$<TARGET_FILE_NAME:oxrsys_runtime>")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/oxrsys-runtime.json.in
    ${CMAKE_CURRENT_BINARY_DIR}/oxrsys-runtime.json.in.configured
    @ONLY
)

# Copy manifest JSON next to the dylib
file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/oxrsys-runtime.json
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/oxrsys-runtime.json.in.configured
)

# Copy default config file next to the dylib (if not already present)
add_custom_command(TARGET oxrsys_runtime POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/oxrsys-runtime.toml
        ${CMAKE_CURRENT_BINARY_DIR}/oxrsys-runtime.toml
    COMMENT "Copying oxrsys-runtime.toml to build directory"
)

get_directory_property(OXRSYS_RUNTIME_HAS_PARENT PARENT_DIRECTORY)
if(OXRSYS_RUNTIME_HAS_PARENT)
    set(OXRSYS_RUNTIME_SELECTED_VIDEO_ENCODER "${OXRSYS_VIDEO_ENCODER_SELECTED}" PARENT_SCOPE)
    if(OXRSYS_RUNTIME_NEEDS_VAAPI)
        set(OXRSYS_RUNTIME_VAAPI_TARGET "${OXRSYS_VAAPI_TARGET}" PARENT_SCOPE)
    endif()
endif()
