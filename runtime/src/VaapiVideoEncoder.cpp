// SPDX-License-Identifier: MPL-2.0

#include "VideoEncoder.h"

#include "Config.h"
#include "VideoBitstream.h"
#include "VideoFramePreparation.h"

#include <spdlog/spdlog.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iterator>
#include <memory>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;

constexpr size_t VaapiSurfaceCount = 3;
constexpr VAConfigID InvalidConfig = VA_INVALID_ID;
constexpr VAContextID InvalidContext = VA_INVALID_ID;
constexpr VASurfaceID InvalidSurface = VA_INVALID_SURFACE;

double ToMilliseconds(Clock::duration duration)
{
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration).count();
}

const char* VaStatusName(VAStatus status)
{
    const char* value = vaErrorStr(status);
    return value != nullptr ? value : "unknown VA-API error";
}

const char* CodecDisplayName(oxr::protocol::VideoCodec codec)
{
    switch (codec)
    {
        case oxr::protocol::VideoCodec::H264:
            return "H.264";
        case oxr::protocol::VideoCodec::AV1:
            return "AV1";
        case oxr::protocol::VideoCodec::H265:
        default:
            return "H.265";
    }
}

uint8_t H264LevelIdc(uint32_t width, uint32_t height, uint32_t fps)
{
    const uint64_t macroblocksPerFrame =
        ((static_cast<uint64_t>(width) + 15u) / 16u) *
        ((static_cast<uint64_t>(height) + 15u) / 16u);
    const uint64_t macroblocksPerSecond = macroblocksPerFrame * std::max(fps, 1u);
    if (macroblocksPerFrame > 22080 || macroblocksPerSecond > 589824)
    {
        return 52;
    }
    if (macroblocksPerFrame > 8704 || macroblocksPerSecond > 245760)
    {
        return 51;
    }
    if (macroblocksPerFrame > 8192 || macroblocksPerSecond > 216000)
    {
        return 50;
    }
    if (macroblocksPerFrame > 3600 || macroblocksPerSecond > 108000)
    {
        return 42;
    }
    return 40;
}

uint8_t H265LevelIdc(uint32_t width, uint32_t height, uint32_t fps)
{
    const uint64_t lumaSamples = static_cast<uint64_t>(width) * height;
    const uint64_t lumaSamplesPerSecond = lumaSamples * std::max(fps, 1u);
    if (lumaSamples > 8912896 || lumaSamplesPerSecond > 534773760)
    {
        return 186; // Level 6.2
    }
    if (lumaSamplesPerSecond > 267386880)
    {
        return 156; // Level 5.2
    }
    if (lumaSamplesPerSecond > 133693440)
    {
        return 153; // Level 5.1
    }
    if (lumaSamples > 2228224 || lumaSamplesPerSecond > 66846720)
    {
        return 150; // Level 5
    }
    return 120; // Level 4
}

struct VaapiSlot
{
    VASurfaceID inputSurface = InvalidSurface;
    VASurfaceID reconSurface = InvalidSurface;
    VABufferID codedBuffer = VA_INVALID_ID;
};

struct VaapiEncoderState
{
    int drmFd = -1;
    VADisplay display = nullptr;
    VAConfigID config = InvalidConfig;
    VAContextID context = InvalidContext;
    std::array<VaapiSlot, VaapiSurfaceCount> slots = {};
    size_t nextSlot = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t fps = 0;
    uint32_t bitrateMbps = 0;
    uint32_t frameIndex = 0;
    uint32_t keyframeInterval = 1;
    bool hasReference = false;
    VASurfaceID lastReconSurface = InvalidSurface;
    uint32_t lastReferenceFrameIndex = 0;
    int32_t lastReferencePoc = 0;
    oxr::protocol::VideoCodec codec = oxr::protocol::VideoCodec::H264;
    VideoFramePreparer preparer;
    PreparedVideoFrame preparedFrame;
};

VaapiEncoderState* State(void* ptr)
{
    return static_cast<VaapiEncoderState*>(ptr);
}

void DestroyBuffer(VADisplay display, VABufferID& buffer)
{
    if (display != nullptr && buffer != VA_INVALID_ID)
    {
        vaDestroyBuffer(display, buffer);
    }
    buffer = VA_INVALID_ID;
}

void DestroyVaapiState(VaapiEncoderState* state)
{
    if (state == nullptr)
    {
        return;
    }

    if (state->display != nullptr)
    {
        for (VaapiSlot& slot : state->slots)
        {
            DestroyBuffer(state->display, slot.codedBuffer);
        }

        std::array<VASurfaceID, VaapiSurfaceCount * 2> surfaces = {};
        uint32_t surfaceCount = 0;
        for (const VaapiSlot& slot : state->slots)
        {
            if (slot.inputSurface != InvalidSurface)
            {
                surfaces[surfaceCount++] = slot.inputSurface;
            }
            if (slot.reconSurface != InvalidSurface)
            {
                surfaces[surfaceCount++] = slot.reconSurface;
            }
        }
        if (surfaceCount > 0)
        {
            vaDestroySurfaces(state->display, surfaces.data(), surfaceCount);
        }

        if (state->context != InvalidContext)
        {
            vaDestroyContext(state->display, state->context);
        }
        if (state->config != InvalidConfig)
        {
            vaDestroyConfig(state->display, state->config);
        }
        vaTerminate(state->display);
    }

    if (state->drmFd >= 0)
    {
        close(state->drmFd);
    }

    delete state;
}

bool HasEntrypoint(VADisplay display, VAProfile profile, VAEntrypoint entrypoint)
{
    const int maxEntrypoints = vaMaxNumEntrypoints(display);
    if (maxEntrypoints <= 0)
    {
        return false;
    }

    std::vector<VAEntrypoint> entrypoints(static_cast<size_t>(maxEntrypoints));
    int entrypointCount = 0;
    if (vaQueryConfigEntrypoints(display, profile, entrypoints.data(), &entrypointCount) != VA_STATUS_SUCCESS)
    {
        return false;
    }
    return std::find(entrypoints.begin(), entrypoints.begin() + entrypointCount, entrypoint) !=
           entrypoints.begin() + entrypointCount;
}

bool OpenVaDisplay(VaapiEncoderState& state)
{
    std::vector<std::string> devicePaths;
    if (const char* overridePath = std::getenv("OXRSYS_VAAPI_DRM_DEVICE");
        overridePath != nullptr && overridePath[0] != '\0')
    {
        devicePaths.emplace_back(overridePath);
    }
    for (int index = 128; index < 144; ++index)
    {
        devicePaths.push_back("/dev/dri/renderD" + std::to_string(index));
    }
    devicePaths.emplace_back("/dev/dri/card0");

    for (const std::string& path : devicePaths)
    {
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0)
        {
            continue;
        }
        VADisplay display = vaGetDisplayDRM(fd);
        if (display == nullptr)
        {
            close(fd);
            continue;
        }

        int major = 0;
        int minor = 0;
        VAStatus status = vaInitialize(display, &major, &minor);
        if (status != VA_STATUS_SUCCESS)
        {
            spdlog::warn("VAAPI: failed to initialize display {}: {}", path, VaStatusName(status));
            close(fd);
            continue;
        }

        state.drmFd = fd;
        state.display = display;
        spdlog::info("VAAPI: initialized display {} (VA {}.{})", path, major, minor);
        return true;
    }

    spdlog::error("VAAPI: no DRM render node could be initialized");
    return false;
}

VAProfile VaProfileForCodec(oxr::protocol::VideoCodec codec)
{
    switch (codec)
    {
        case oxr::protocol::VideoCodec::H264:
            return VAProfileH264High;
        case oxr::protocol::VideoCodec::H265:
            return VAProfileHEVCMain;
        case oxr::protocol::VideoCodec::AV1:
        default:
            return VAProfileNone;
    }
}

bool CreateConfig(VaapiEncoderState& state)
{
    const VAProfile profile = VaProfileForCodec(state.codec);
    if (profile == VAProfileNone ||
        !HasEntrypoint(state.display, profile, VAEntrypointEncSlice))
    {
        spdlog::error("VAAPI: {} encode entrypoint is not supported by this driver",
                      CodecDisplayName(state.codec));
        return false;
    }

    std::array<VAConfigAttrib, 3> attribs = {};
    attribs[0].type = VAConfigAttribRTFormat;
    attribs[0].value = VA_RT_FORMAT_YUV420;
    attribs[1].type = VAConfigAttribRateControl;
    attribs[1].value = VA_RC_CBR;
    attribs[2].type = VAConfigAttribEncPackedHeaders;
    attribs[2].value = VA_ENC_PACKED_HEADER_NONE;

    VAStatus status = vaCreateConfig(
        state.display,
        profile,
        VAEntrypointEncSlice,
        attribs.data(),
        static_cast<int>(attribs.size()),
        &state.config);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::error("VAAPI: failed to create {} config: {}",
                      CodecDisplayName(state.codec), VaStatusName(status));
        return false;
    }
    return true;
}

bool CreateSurfaces(VaapiEncoderState& state)
{
    std::array<VASurfaceID, VaapiSurfaceCount * 2> surfaces = {};
    VASurfaceAttrib pixelFormat = {};
    pixelFormat.type = VASurfaceAttribPixelFormat;
    pixelFormat.flags = VA_SURFACE_ATTRIB_SETTABLE;
    pixelFormat.value.type = VAGenericValueTypeInteger;
    pixelFormat.value.value.i = VA_FOURCC_NV12;

    VAStatus status = vaCreateSurfaces(
        state.display,
        VA_RT_FORMAT_YUV420,
        state.width,
        state.height,
        surfaces.data(),
        static_cast<unsigned int>(surfaces.size()),
        &pixelFormat,
        1);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::error("VAAPI: failed to create NV12 surfaces: {}", VaStatusName(status));
        return false;
    }

    for (size_t i = 0; i < VaapiSurfaceCount; ++i)
    {
        state.slots[i].inputSurface = surfaces[i * 2u + 0u];
        state.slots[i].reconSurface = surfaces[i * 2u + 1u];
    }
    return true;
}

bool CreateContext(VaapiEncoderState& state)
{
    std::array<VASurfaceID, VaapiSurfaceCount> reconSurfaces = {};
    for (size_t i = 0; i < VaapiSurfaceCount; ++i)
    {
        reconSurfaces[i] = state.slots[i].reconSurface;
    }

    VAStatus status = vaCreateContext(
        state.display,
        state.config,
        state.width,
        state.height,
        VA_PROGRESSIVE,
        reconSurfaces.data(),
        static_cast<int>(reconSurfaces.size()),
        &state.context);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::error("VAAPI: failed to create context: {}", VaStatusName(status));
        return false;
    }
    return true;
}

bool EnsureCodedBuffer(VaapiEncoderState& state, VaapiSlot& slot)
{
    if (slot.codedBuffer != VA_INVALID_ID)
    {
        DestroyBuffer(state.display, slot.codedBuffer);
    }

    const unsigned int codedBufferSize = std::max(
        static_cast<unsigned int>(state.width * state.height),
        2u * 1024u * 1024u);
    VAStatus status = vaCreateBuffer(
        state.display,
        state.context,
        VAEncCodedBufferType,
        codedBufferSize,
        1,
        nullptr,
        &slot.codedBuffer);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to create coded buffer: {}", VaStatusName(status));
        return false;
    }
    return true;
}

bool UploadNv12(VaapiEncoderState& state, VASurfaceID surface, const PreparedVideoFrame& frame)
{
    VAImage image = {};
    VAStatus status = vaDeriveImage(state.display, surface, &image);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to derive input image: {}", VaStatusName(status));
        return false;
    }

    void* mapped = nullptr;
    status = vaMapBuffer(state.display, image.buf, &mapped);
    if (status != VA_STATUS_SUCCESS || mapped == nullptr)
    {
        spdlog::warn("VAAPI: failed to map input image: {}", VaStatusName(status));
        vaDestroyImage(state.display, image.image_id);
        return false;
    }

    auto* dst = static_cast<uint8_t*>(mapped);
    const uint8_t* srcY = frame.YPlane();
    const uint8_t* srcUV = frame.UVPlane();
    for (uint32_t y = 0; y < frame.height; ++y)
    {
        std::memcpy(dst + image.offsets[0] + static_cast<size_t>(image.pitches[0]) * y,
                    srcY + static_cast<size_t>(frame.yStride) * y,
                    frame.width);
    }
    for (uint32_t y = 0; y < frame.height / 2u; ++y)
    {
        std::memcpy(dst + image.offsets[1] + static_cast<size_t>(image.pitches[1]) * y,
                    srcUV + static_cast<size_t>(frame.uvStride) * y,
                    frame.width);
    }

    vaUnmapBuffer(state.display, image.buf);
    vaDestroyImage(state.display, image.image_id);
    return true;
}

VABufferID CreateBuffer(VaapiEncoderState& state,
                        VABufferType type,
                        unsigned int size,
                        void* data)
{
    VABufferID buffer = VA_INVALID_ID;
    VAStatus status = vaCreateBuffer(state.display, state.context, type, size, 1, data, &buffer);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to create render buffer type {}: {}",
                     static_cast<int>(type), VaStatusName(status));
        return VA_INVALID_ID;
    }
    return buffer;
}

VABufferID CreateMiscBuffer(VaapiEncoderState& state,
                            VAEncMiscParameterType type,
                            const void* payload,
                            size_t payloadSize)
{
    const size_t totalSize = sizeof(VAEncMiscParameterBuffer) + payloadSize;
    std::vector<uint8_t> storage(totalSize, 0);
    auto* header = reinterpret_cast<VAEncMiscParameterBuffer*>(storage.data());
    header->type = type;
    std::memcpy(header->data, payload, payloadSize);
    return CreateBuffer(state,
                        VAEncMiscParameterBufferType,
                        static_cast<unsigned int>(storage.size()),
                        storage.data());
}

void FillInvalidReferences(VAPictureH264* pictures, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        pictures[i].picture_id = VA_INVALID_SURFACE;
        pictures[i].flags = VA_PICTURE_H264_INVALID;
    }
}

void FillInvalidReferences(VAPictureHEVC* pictures, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        pictures[i].picture_id = VA_INVALID_SURFACE;
        pictures[i].pic_order_cnt = 0;
        pictures[i].flags = VA_PICTURE_HEVC_INVALID;
    }
}

bool EncodeH264Frame(VaapiEncoderState& state,
                     VaapiSlot& slot,
                     bool forceKeyframe,
                     std::vector<uint8_t>& encoded)
{
    if (!EnsureCodedBuffer(state, slot))
    {
        return false;
    }

    const bool keyframe =
        forceKeyframe || state.frameIndex == 0 || !state.hasReference ||
        (state.keyframeInterval > 0 && (state.frameIndex % state.keyframeInterval) == 0);
    const uint16_t mbWidth = static_cast<uint16_t>((state.width + 15u) / 16u);
    const uint16_t mbHeight = static_cast<uint16_t>((state.height + 15u) / 16u);
    const uint32_t frameNum = state.frameIndex & 0xffu;

    VAEncSequenceParameterBufferH264 seq = {};
    seq.seq_parameter_set_id = 0;
    seq.level_idc = H264LevelIdc(state.width, state.height, state.fps);
    seq.intra_period = state.keyframeInterval;
    seq.intra_idr_period = state.keyframeInterval;
    seq.ip_period = 1;
    seq.bits_per_second = state.bitrateMbps * 1000u * 1000u;
    seq.max_num_ref_frames = 1;
    seq.picture_width_in_mbs = mbWidth;
    seq.picture_height_in_mbs = mbHeight;
    seq.seq_fields.bits.chroma_format_idc = 1;
    seq.seq_fields.bits.frame_mbs_only_flag = 1;
    seq.seq_fields.bits.direct_8x8_inference_flag = 1;
    seq.seq_fields.bits.log2_max_frame_num_minus4 = 4;
    seq.seq_fields.bits.pic_order_cnt_type = 0;
    seq.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 = 4;
    const uint32_t paddedWidth = mbWidth * 16u;
    const uint32_t paddedHeight = mbHeight * 16u;
    if (paddedWidth != state.width || paddedHeight != state.height)
    {
        seq.frame_cropping_flag = 1;
        seq.frame_crop_right_offset = (paddedWidth - state.width) / 2u;
        seq.frame_crop_bottom_offset = (paddedHeight - state.height) / 2u;
    }
    seq.vui_parameters_present_flag = 1;
    seq.vui_fields.bits.timing_info_present_flag = 1;
    seq.vui_fields.bits.fixed_frame_rate_flag = 1;
    seq.vui_fields.bits.motion_vectors_over_pic_boundaries_flag = 1;
    seq.num_units_in_tick = 1;
    seq.time_scale = std::max(state.fps, 1u) * 2u;

    VAEncPictureParameterBufferH264 pic = {};
    pic.CurrPic.picture_id = slot.reconSurface;
    pic.CurrPic.frame_idx = frameNum;
    pic.CurrPic.flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
    FillInvalidReferences(pic.ReferenceFrames, std::size(pic.ReferenceFrames));
    if (!keyframe && state.lastReconSurface != InvalidSurface)
    {
        pic.ReferenceFrames[0].picture_id = state.lastReconSurface;
        pic.ReferenceFrames[0].frame_idx = state.lastReferenceFrameIndex;
        pic.ReferenceFrames[0].flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
    }
    pic.coded_buf = slot.codedBuffer;
    pic.pic_parameter_set_id = 0;
    pic.seq_parameter_set_id = 0;
    pic.frame_num = static_cast<uint16_t>(frameNum);
    pic.pic_init_qp = 26;
    pic.pic_fields.bits.idr_pic_flag = keyframe ? 1 : 0;
    pic.pic_fields.bits.reference_pic_flag = 1;
    pic.pic_fields.bits.deblocking_filter_control_present_flag = 1;

    VAEncSliceParameterBufferH264 slice = {};
    slice.macroblock_address = 0;
    slice.num_macroblocks = static_cast<uint32_t>(mbWidth) * mbHeight;
    slice.macroblock_info = VA_INVALID_ID;
    slice.slice_type = keyframe ? 2 : 0; // I or P, no B frames for low latency.
    slice.pic_parameter_set_id = 0;
    slice.idr_pic_id = static_cast<uint16_t>(frameNum);
    slice.pic_order_cnt_lsb = static_cast<uint16_t>((state.frameIndex * 2u) & 0xffu);
    slice.disable_deblocking_filter_idc = 0;
    FillInvalidReferences(slice.RefPicList0, std::size(slice.RefPicList0));
    FillInvalidReferences(slice.RefPicList1, std::size(slice.RefPicList1));
    if (!keyframe && state.lastReconSurface != InvalidSurface)
    {
        slice.RefPicList0[0].picture_id = state.lastReconSurface;
        slice.RefPicList0[0].frame_idx = state.lastReferenceFrameIndex;
        slice.RefPicList0[0].flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
    }

    VAEncMiscParameterRateControl rateControl = {};
    rateControl.bits_per_second = state.bitrateMbps * 1000u * 1000u;
    rateControl.target_percentage = 100;
    rateControl.window_size = 500;
    rateControl.rc_flags.bits.disable_frame_skip = 1;

    VAEncMiscParameterFrameRate frameRate = {};
    frameRate.framerate = std::max(state.fps, 1u);

    std::vector<VABufferID> buffers;
    buffers.reserve(5);
    buffers.push_back(CreateBuffer(state, VAEncSequenceParameterBufferType, sizeof(seq), &seq));
    buffers.push_back(CreateBuffer(state, VAEncPictureParameterBufferType, sizeof(pic), &pic));
    buffers.push_back(CreateBuffer(state, VAEncSliceParameterBufferType, sizeof(slice), &slice));
    buffers.push_back(CreateMiscBuffer(state, VAEncMiscParameterTypeRateControl,
                                       &rateControl, sizeof(rateControl)));
    buffers.push_back(CreateMiscBuffer(state, VAEncMiscParameterTypeFrameRate,
                                       &frameRate, sizeof(frameRate)));
    if (std::any_of(buffers.begin(), buffers.end(), [](VABufferID id) { return id == VA_INVALID_ID; }))
    {
        for (VABufferID& buffer : buffers)
        {
            DestroyBuffer(state.display, buffer);
        }
        return false;
    }

    VAStatus status = vaBeginPicture(state.display, state.context, slot.inputSurface);
    if (status == VA_STATUS_SUCCESS)
    {
        status = vaRenderPicture(
            state.display,
            state.context,
            buffers.data(),
            static_cast<int>(buffers.size()));
    }
    if (status == VA_STATUS_SUCCESS)
    {
        status = vaEndPicture(state.display, state.context);
    }
    for (VABufferID& buffer : buffers)
    {
        DestroyBuffer(state.display, buffer);
    }
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to submit H.264 frame: {}", VaStatusName(status));
        return false;
    }

    status = vaSyncSurface(state.display, slot.inputSurface);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to sync encoded surface: {}", VaStatusName(status));
        return false;
    }

    VACodedBufferSegment* segment = nullptr;
    status = vaMapBuffer(state.display, slot.codedBuffer, reinterpret_cast<void**>(&segment));
    if (status != VA_STATUS_SUCCESS || segment == nullptr)
    {
        spdlog::warn("VAAPI: failed to map coded buffer: {}", VaStatusName(status));
        return false;
    }

    encoded.clear();
    for (auto* current = segment; current != nullptr;
         current = static_cast<VACodedBufferSegment*>(current->next))
    {
        if (current->buf != nullptr && current->size > 0)
        {
            const auto* bytes = static_cast<const uint8_t*>(current->buf);
            encoded.insert(encoded.end(), bytes, bytes + current->size);
        }
    }
    vaUnmapBuffer(state.display, slot.codedBuffer);

    state.hasReference = true;
    state.lastReconSurface = slot.reconSurface;
    state.lastReferenceFrameIndex = frameNum;
    state.lastReferencePoc = static_cast<int32_t>(state.frameIndex * 2u);
    state.frameIndex++;
    return !encoded.empty();
}

bool EncodeH265Frame(VaapiEncoderState& state,
                     VaapiSlot& slot,
                     bool forceKeyframe,
                     std::vector<uint8_t>& encoded)
{
    if (!EnsureCodedBuffer(state, slot))
    {
        return false;
    }

    const bool keyframe =
        forceKeyframe || state.frameIndex == 0 || !state.hasReference ||
        (state.keyframeInterval > 0 && (state.frameIndex % state.keyframeInterval) == 0);
    const uint32_t ctbWidth = (state.width + 63u) / 64u;
    const uint32_t ctbHeight = (state.height + 63u) / 64u;
    const int32_t pictureOrderCount = static_cast<int32_t>(state.frameIndex * 2u);

    VAEncSequenceParameterBufferHEVC seq = {};
    seq.general_profile_idc = 1; // Main
    seq.general_level_idc = H265LevelIdc(state.width, state.height, state.fps);
    seq.general_tier_flag = 0;
    seq.intra_period = state.keyframeInterval;
    seq.intra_idr_period = state.keyframeInterval;
    seq.ip_period = 1;
    seq.bits_per_second = state.bitrateMbps * 1000u * 1000u;
    seq.pic_width_in_luma_samples = static_cast<uint16_t>(state.width);
    seq.pic_height_in_luma_samples = static_cast<uint16_t>(state.height);
    seq.seq_fields.bits.chroma_format_idc = 1;
    seq.seq_fields.bits.bit_depth_luma_minus8 = 0;
    seq.seq_fields.bits.bit_depth_chroma_minus8 = 0;
    seq.seq_fields.bits.strong_intra_smoothing_enabled_flag = 1;
    seq.seq_fields.bits.low_delay_seq = 1;
    seq.log2_min_luma_coding_block_size_minus3 = 0; // 8x8 minimum coding block.
    seq.log2_diff_max_min_luma_coding_block_size = 3; // 64x64 CTB.
    seq.log2_min_transform_block_size_minus2 = 0; // 4x4 minimum transform.
    seq.log2_diff_max_min_transform_block_size = 3; // 32x32 maximum transform.
    seq.max_transform_hierarchy_depth_inter = 2;
    seq.max_transform_hierarchy_depth_intra = 2;
    seq.vui_parameters_present_flag = 1;
    seq.vui_fields.bits.vui_timing_info_present_flag = 1;
    seq.vui_fields.bits.motion_vectors_over_pic_boundaries_flag = 1;
    seq.vui_num_units_in_tick = 1;
    seq.vui_time_scale = std::max(state.fps, 1u);

    VAEncPictureParameterBufferHEVC pic = {};
    pic.decoded_curr_pic.picture_id = slot.reconSurface;
    pic.decoded_curr_pic.pic_order_cnt = pictureOrderCount;
    pic.decoded_curr_pic.flags = 0;
    FillInvalidReferences(pic.reference_frames, std::size(pic.reference_frames));
    if (!keyframe && state.lastReconSurface != InvalidSurface)
    {
        pic.reference_frames[0].picture_id = state.lastReconSurface;
        pic.reference_frames[0].pic_order_cnt = state.lastReferencePoc;
        pic.reference_frames[0].flags = 0;
    }
    pic.coded_buf = slot.codedBuffer;
    pic.collocated_ref_pic_index = 0xff;
    pic.pic_init_qp = 26;
    pic.log2_parallel_merge_level_minus2 = 0;
    pic.ctu_max_bitsize_allowed = 0;
    pic.num_ref_idx_l0_default_active_minus1 = 0;
    pic.num_ref_idx_l1_default_active_minus1 = 0;
    pic.slice_pic_parameter_set_id = 0;
    pic.nal_unit_type = keyframe ? 19 : 1; // IDR_W_RADL or trailing non-IRAP.
    pic.pic_fields.bits.idr_pic_flag = keyframe ? 1 : 0;
    pic.pic_fields.bits.coding_type = keyframe ? 1 : 2; // I or P picture, no B frames.
    pic.pic_fields.bits.reference_pic_flag = 1;
    pic.pic_fields.bits.pps_loop_filter_across_slices_enabled_flag = 1;

    VAEncSliceParameterBufferHEVC slice = {};
    slice.slice_segment_address = 0;
    slice.num_ctu_in_slice = ctbWidth * ctbHeight;
    slice.slice_type = keyframe ? 2 : 1; // I or P slice.
    slice.slice_pic_parameter_set_id = 0;
    slice.num_ref_idx_l0_active_minus1 = 0;
    slice.num_ref_idx_l1_active_minus1 = 0;
    FillInvalidReferences(slice.ref_pic_list0, std::size(slice.ref_pic_list0));
    FillInvalidReferences(slice.ref_pic_list1, std::size(slice.ref_pic_list1));
    if (!keyframe && state.lastReconSurface != InvalidSurface)
    {
        slice.ref_pic_list0[0].picture_id = state.lastReconSurface;
        slice.ref_pic_list0[0].pic_order_cnt = state.lastReferencePoc;
        slice.ref_pic_list0[0].flags = 0;
    }
    slice.max_num_merge_cand = 5;
    slice.slice_qp_delta = 0;
    slice.slice_fields.bits.last_slice_of_pic_flag = 1;
    slice.slice_fields.bits.slice_loop_filter_across_slices_enabled_flag = 1;

    VAEncMiscParameterRateControl rateControl = {};
    rateControl.bits_per_second = state.bitrateMbps * 1000u * 1000u;
    rateControl.target_percentage = 100;
    rateControl.window_size = 500;
    rateControl.rc_flags.bits.disable_frame_skip = 1;

    VAEncMiscParameterFrameRate frameRate = {};
    frameRate.framerate = std::max(state.fps, 1u);

    std::vector<VABufferID> buffers;
    buffers.reserve(5);
    buffers.push_back(CreateBuffer(state, VAEncSequenceParameterBufferType, sizeof(seq), &seq));
    buffers.push_back(CreateBuffer(state, VAEncPictureParameterBufferType, sizeof(pic), &pic));
    buffers.push_back(CreateBuffer(state, VAEncSliceParameterBufferType, sizeof(slice), &slice));
    buffers.push_back(CreateMiscBuffer(state, VAEncMiscParameterTypeRateControl,
                                       &rateControl, sizeof(rateControl)));
    buffers.push_back(CreateMiscBuffer(state, VAEncMiscParameterTypeFrameRate,
                                       &frameRate, sizeof(frameRate)));
    if (std::any_of(buffers.begin(), buffers.end(), [](VABufferID id) { return id == VA_INVALID_ID; }))
    {
        for (VABufferID& buffer : buffers)
        {
            DestroyBuffer(state.display, buffer);
        }
        return false;
    }

    VAStatus status = vaBeginPicture(state.display, state.context, slot.inputSurface);
    if (status == VA_STATUS_SUCCESS)
    {
        status = vaRenderPicture(
            state.display,
            state.context,
            buffers.data(),
            static_cast<int>(buffers.size()));
    }
    if (status == VA_STATUS_SUCCESS)
    {
        status = vaEndPicture(state.display, state.context);
    }
    for (VABufferID& buffer : buffers)
    {
        DestroyBuffer(state.display, buffer);
    }
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to submit H.265 frame: {}", VaStatusName(status));
        return false;
    }

    status = vaSyncSurface(state.display, slot.inputSurface);
    if (status != VA_STATUS_SUCCESS)
    {
        spdlog::warn("VAAPI: failed to sync encoded surface: {}", VaStatusName(status));
        return false;
    }

    VACodedBufferSegment* segment = nullptr;
    status = vaMapBuffer(state.display, slot.codedBuffer, reinterpret_cast<void**>(&segment));
    if (status != VA_STATUS_SUCCESS || segment == nullptr)
    {
        spdlog::warn("VAAPI: failed to map coded buffer: {}", VaStatusName(status));
        return false;
    }

    encoded.clear();
    for (auto* current = segment; current != nullptr;
         current = static_cast<VACodedBufferSegment*>(current->next))
    {
        if (current->buf != nullptr && current->size > 0)
        {
            const auto* bytes = static_cast<const uint8_t*>(current->buf);
            encoded.insert(encoded.end(), bytes, bytes + current->size);
        }
    }
    vaUnmapBuffer(state.display, slot.codedBuffer);

    state.hasReference = true;
    state.lastReconSurface = slot.reconSurface;
    state.lastReferenceFrameIndex = state.frameIndex & 0xffu;
    state.lastReferencePoc = pictureOrderCount;
    state.frameIndex++;
    return !encoded.empty();
}

} // namespace

VideoEncoder::VideoEncoder() = default;

VideoEncoder::~VideoEncoder()
{
    Shutdown();
}

bool VideoEncoder::SupportsFoveatedEncoding(const GraphicsContext& /*graphicsContext*/)
{
    return false;
}

VideoEncoder::BackendCapabilities VideoEncoder::QueryBackendCapabilities(
    const GraphicsContext* /*graphicsContext*/)
{
    BackendCapabilities capabilities = {};
    capabilities.backendName = "VA-API";
    capabilities.hardwareEncoder = true;

    VaapiEncoderState state = {};
    if (!OpenVaDisplay(state))
    {
        capabilities.unsupportedReason = "no VA-API DRM render node could be initialized";
        if (state.display != nullptr)
        {
            vaTerminate(state.display);
        }
        if (state.drmFd >= 0)
        {
            close(state.drmFd);
        }
        return capabilities;
    }

    capabilities.supportsH264 =
        HasEntrypoint(state.display, VAProfileH264High, VAEntrypointEncSlice);
    capabilities.supportsH265 =
        HasEntrypoint(state.display, VAProfileHEVCMain, VAEntrypointEncSlice);
    if (!capabilities.supportsH264 && !capabilities.supportsH265)
    {
        capabilities.unsupportedReason = "driver exposes no H.264/H.265 encode entrypoint";
    }

    vaTerminate(state.display);
    if (state.drmFd >= 0)
    {
        close(state.drmFd);
    }
    return capabilities;
}

bool VideoEncoder::SupportsCodec(oxr::protocol::VideoCodec codec)
{
    return QueryBackendCapabilities(nullptr).SupportsCodec(codec);
}

bool VideoEncoder::Initialize(uint32_t width, uint32_t height, uint32_t fps,
                              uint32_t bitrateMbps, const GraphicsContext& graphicsContext,
                              oxr::protocol::VideoCodec codec)
{
    Shutdown();
    if (!SupportsCodec(codec))
    {
        spdlog::error("VAAPI: {} is not implemented in the VA-API backend", CodecDisplayName(codec));
        return false;
    }

    auto state = std::make_unique<VaapiEncoderState>();
    state->width = width;
    state->height = height;
    state->fps = std::max(fps, 1u);
    state->bitrateMbps = bitrateMbps;
    state->keyframeInterval = std::max(Config::Get().GetValues().keyframeIntervalSec * state->fps, 1u);
    state->codec = codec;
    for (VaapiSlot& slot : state->slots)
    {
        slot.inputSurface = InvalidSurface;
        slot.reconSurface = InvalidSurface;
        slot.codedBuffer = VA_INVALID_ID;
    }

    if ((width % 2u) != 0 || (height % 2u) != 0)
    {
        spdlog::error("VAAPI: NV12 encode dimensions must be even ({}x{})", width, height);
        return false;
    }
    if (!OpenVaDisplay(*state) || !CreateConfig(*state) || !CreateSurfaces(*state) || !CreateContext(*state))
    {
        DestroyVaapiState(state.release());
        return false;
    }

    width_ = width;
    height_ = height;
    eyeWidth_ = width / 2u;
    fps_ = state->fps;
    bitrateMbps_ = bitrateMbps;
    codec_ = codec;
    graphicsContext_ = graphicsContext;
    frameCount_ = 0;
    forceKeyframe_.store(false);
    shuttingDown_.store(false);
    droppedFrameCount_.store(0);
    inFlightFrameCount_.store(0);
    frameNumberCounter_.store(0);
    platformEncoder_ = state.release();
    initialized_ = true;

    spdlog::info("VAAPI: initialized {} encoder {}x{} @ {}Hz {}Mbps keyframe={}s",
                 CodecDisplayName(codec_), width_, height_, fps_, bitrateMbps_,
                 Config::Get().GetValues().keyframeIntervalSec);
    return true;
}

void VideoEncoder::Shutdown()
{
    shuttingDown_.store(true);
    initialized_ = false;
    auto* state = State(platformEncoder_);
    platformEncoder_ = nullptr;
    DestroyVaapiState(state);
    inFlightFrameCount_.store(0);
}

bool VideoEncoder::Encode(FrameImageSource imageSource, int64_t timestampNs,
                          OnNalUnitCallback callback,
                          OnFrameEncodedCallback frameCallback)
{
    FrameSource frameSource = {};
    frameSource.left = std::move(imageSource);
    return EncodeInternal(std::move(frameSource), false, timestampNs,
                          std::move(callback), std::move(frameCallback));
}

bool VideoEncoder::EncodeStereo(FrameSource frameSource, int64_t timestampNs,
                                OnNalUnitCallback callback,
                                OnFrameEncodedCallback frameCallback)
{
    return EncodeInternal(std::move(frameSource), true, timestampNs,
                          std::move(callback), std::move(frameCallback));
}

bool VideoEncoder::EncodeInternal(FrameSource frameSource, bool stereo,
                                  int64_t timestampNs, OnNalUnitCallback callback,
                                  OnFrameEncodedCallback frameCallback)
{
    auto* state = State(platformEncoder_);
    if (state == nullptr)
    {
        return false;
    }

    const auto encodeStart = Clock::now();
    inFlightFrameCount_.fetch_add(1);

    FrameMetrics metrics = {};
    metrics.frameNumber = frameNumberCounter_.fetch_add(1) + 1;
    metrics.timestampNs = timestampNs;

    auto finishDroppedFrame = [&](const char* reason) {
        droppedFrameCount_.fetch_add(1);
        inFlightFrameCount_.fetch_sub(1);
        metrics.frameDropped = true;
        metrics.totalLatencyMs = ToMilliseconds(Clock::now() - encodeStart);
        if (reason != nullptr && metrics.frameNumber <= 5)
        {
            spdlog::warn("VAAPI: dropping frame {}: {}", metrics.frameNumber, reason);
        }
        if (frameCallback)
        {
            frameCallback(metrics);
        }
        return false;
    };

    const auto copyStart = Clock::now();
    if (frameSource.left.IsValid() && (!stereo || frameSource.right.IsValid()))
    {
        if (!state->preparer.PrepareStereo(
                std::move(frameSource), stereo, graphicsContext_, width_, height_, state->preparedFrame))
        {
            return finishDroppedFrame("backend readback/NV12 conversion failed");
        }
    }
    else if (!VideoFramePreparer::FillBlack(width_, height_, state->preparedFrame))
    {
        return finishDroppedFrame("backend frame source unavailable");
    }
    metrics.gpuCopyMs = ToMilliseconds(Clock::now() - copyStart);

    VaapiSlot& slot = state->slots[state->nextSlot];
    state->nextSlot = (state->nextSlot + 1u) % state->slots.size();
    if (!UploadNv12(*state, slot.inputSurface, state->preparedFrame))
    {
        return finishDroppedFrame("NV12 surface upload failed");
    }

    const bool requestedKeyframe = forceKeyframe_.exchange(false);
    std::vector<uint8_t> encoded;
    const auto submitStart = Clock::now();
    bool encodedFrame = false;
    if (codec_ == oxr::protocol::VideoCodec::H265)
    {
        encodedFrame = EncodeH265Frame(*state, slot, requestedKeyframe, encoded);
    }
    else
    {
        encodedFrame = EncodeH264Frame(*state, slot, requestedKeyframe, encoded);
    }
    if (!encodedFrame)
    {
        return finishDroppedFrame("VA-API encode failed");
    }
    metrics.encodeSubmitMs = ToMilliseconds(Clock::now() - submitStart);
    const bool expectedKeyframe = requestedKeyframe || frameCount_ == 0 ||
        (state->keyframeInterval > 0 && (frameCount_ % state->keyframeInterval) == 0);

    bool emittedOutput = false;
    bool emittedKeyframe = false;
    if (callback && !encoded.empty())
    {
        emittedOutput = oxrsys::video_bitstream::VisitAnnexBUnits(
            codec_, encoded.data(), encoded.size(), expectedKeyframe,
            [&](const uint8_t* data, size_t size, bool keyframe) {
                emittedKeyframe = emittedKeyframe || keyframe;
                callback(data, size, keyframe, timestampNs);
            });
    }
    else
    {
        emittedOutput = !encoded.empty();
    }
    metrics.keyframe = emittedKeyframe || expectedKeyframe;

    frameCount_++;
    metrics.totalLatencyMs = ToMilliseconds(Clock::now() - encodeStart);
    inFlightFrameCount_.fetch_sub(1);
    if (!emittedOutput)
    {
        droppedFrameCount_.fetch_add(1);
        metrics.frameDropped = true;
    }
    if (frameCallback)
    {
        frameCallback(metrics);
    }
    return emittedOutput;
}

void VideoEncoder::ForceKeyframe()
{
    forceKeyframe_.store(true);
}

void VideoEncoder::SetBitrate(uint32_t bitrateMbps)
{
    bitrateMbps_ = bitrateMbps;
    if (auto* state = State(platformEncoder_))
    {
        state->bitrateMbps = bitrateMbps;
    }
}

bool VideoEncoder::AcquireSlot(size_t& outSlotIndex)
{
    outSlotIndex = 0;
    return false;
}

void VideoEncoder::ReleaseSlot(size_t /*slotIndex*/)
{
}

void VideoEncoder::DestroySlots()
{
}
