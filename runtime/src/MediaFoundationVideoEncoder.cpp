// SPDX-License-Identifier: MPL-2.0

#include "VideoEncoder.h"

#include "Config.h"
#include "VideoFramePreparation.h"

#include <spdlog/spdlog.h>

#include <windows.h>
#include <codecapi.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mftransform.h>
#include <wmcodecdsp.h>
#include <wrl/client.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
using Microsoft::WRL::ComPtr;

#ifndef RETURN_IF_FAILED
#define RETURN_IF_FAILED(expr) \
    do \
    { \
        const HRESULT _oxrsys_hr = (expr); \
        if (FAILED(_oxrsys_hr)) \
        { \
            return _oxrsys_hr; \
        } \
    } while (false)
#endif

double ToMilliseconds(Clock::duration duration)
{
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration).count();
}

std::string HResultString(HRESULT result)
{
    char buffer[32] = {};
    std::snprintf(buffer, sizeof(buffer), "HRESULT 0x%08lx", static_cast<unsigned long>(result));
    return buffer;
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

GUID MediaFoundationSubtype(oxr::protocol::VideoCodec codec)
{
    switch (codec)
    {
        case oxr::protocol::VideoCodec::H264:
            return MFVideoFormat_H264;
        case oxr::protocol::VideoCodec::H265:
            return MFVideoFormat_HEVC;
        case oxr::protocol::VideoCodec::AV1:
        default:
            return GUID_NULL;
    }
}

struct MediaFoundationEncoderState
{
    ComPtr<IMFTransform> transform;
    ComPtr<ICodecAPI> codecApi;
    DWORD inputStreamId = 0;
    DWORD outputStreamId = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t fps = 0;
    uint32_t bitrateMbps = 0;
    uint32_t frameIndex = 0;
    bool comInitialized = false;
    bool mfStarted = false;
    VideoFramePreparer preparer;
    PreparedVideoFrame preparedFrame;
};

MediaFoundationEncoderState* State(void* ptr)
{
    return static_cast<MediaFoundationEncoderState*>(ptr);
}

void DestroyState(MediaFoundationEncoderState* state)
{
    if (state == nullptr)
    {
        return;
    }
    if (state->transform)
    {
        state->transform->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0);
        state->transform->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0);
    }
    state->codecApi.Reset();
    state->transform.Reset();
    if (state->mfStarted)
    {
        MFShutdown();
    }
    if (state->comInitialized)
    {
        CoUninitialize();
    }
    delete state;
}

bool SetCodecApiBoolean(ICodecAPI* codecApi, const GUID& key, bool value)
{
    if (codecApi == nullptr)
    {
        return false;
    }
    VARIANT variant = {};
    variant.vt = VT_BOOL;
    variant.boolVal = value ? VARIANT_TRUE : VARIANT_FALSE;
    const HRESULT result = codecApi->SetValue(&key, &variant);
    return SUCCEEDED(result);
}

bool SetCodecApiUInt32(ICodecAPI* codecApi, const GUID& key, uint32_t value)
{
    if (codecApi == nullptr)
    {
        return false;
    }
    VARIANT variant = {};
    variant.vt = VT_UI4;
    variant.ulVal = value;
    const HRESULT result = codecApi->SetValue(&key, &variant);
    return SUCCEEDED(result);
}

HRESULT SetVideoTypeCommon(IMFMediaType* type,
                           const GUID& subtype,
                           uint32_t width,
                           uint32_t height,
                           uint32_t fps,
                           uint32_t bitrateMbps)
{
    RETURN_IF_FAILED(type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    RETURN_IF_FAILED(type->SetGUID(MF_MT_SUBTYPE, subtype));
    RETURN_IF_FAILED(type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    RETURN_IF_FAILED(MFSetAttributeSize(type, MF_MT_FRAME_SIZE, width, height));
    RETURN_IF_FAILED(MFSetAttributeRatio(type, MF_MT_FRAME_RATE, std::max(fps, 1u), 1));
    RETURN_IF_FAILED(MFSetAttributeRatio(type, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
    if (bitrateMbps > 0)
    {
        RETURN_IF_FAILED(type->SetUINT32(MF_MT_AVG_BITRATE, bitrateMbps * 1000u * 1000u));
    }
    return S_OK;
}

HRESULT CreateMediaTypes(MediaFoundationEncoderState& state,
                         oxr::protocol::VideoCodec codec,
                         IMFMediaType** inputType,
                         IMFMediaType** outputType)
{
    ComPtr<IMFMediaType> input;
    ComPtr<IMFMediaType> output;
    RETURN_IF_FAILED(MFCreateMediaType(&input));
    RETURN_IF_FAILED(MFCreateMediaType(&output));
    RETURN_IF_FAILED(SetVideoTypeCommon(
        input.Get(), MFVideoFormat_NV12, state.width, state.height, state.fps, 0));
    RETURN_IF_FAILED(SetVideoTypeCommon(
        output.Get(), MediaFoundationSubtype(codec), state.width, state.height,
        state.fps, state.bitrateMbps));
    *inputType = input.Detach();
    *outputType = output.Detach();
    return S_OK;
}

HRESULT ActivateEncoder(MediaFoundationEncoderState& state,
                        oxr::protocol::VideoCodec codec)
{
    MFT_REGISTER_TYPE_INFO inputInfo = {};
    inputInfo.guidMajorType = MFMediaType_Video;
    inputInfo.guidSubtype = MFVideoFormat_NV12;

    MFT_REGISTER_TYPE_INFO outputInfo = {};
    outputInfo.guidMajorType = MFMediaType_Video;
    outputInfo.guidSubtype = MediaFoundationSubtype(codec);

    IMFActivate** activates = nullptr;
    UINT32 activateCount = 0;
    HRESULT result = MFTEnumEx(
        MFT_CATEGORY_VIDEO_ENCODER,
        MFT_ENUM_FLAG_HARDWARE | MFT_ENUM_FLAG_SORTANDFILTER,
        &inputInfo,
        &outputInfo,
        &activates,
        &activateCount);
    if (FAILED(result) || activateCount == 0)
    {
        if (activates != nullptr)
        {
            CoTaskMemFree(activates);
            activates = nullptr;
        }
        result = MFTEnumEx(
            MFT_CATEGORY_VIDEO_ENCODER,
            MFT_ENUM_FLAG_ALL,
            &inputInfo,
            &outputInfo,
            &activates,
            &activateCount);
    }
    if (FAILED(result) || activateCount == 0 || activates == nullptr)
    {
        return FAILED(result) ? result : MF_E_TOPO_CODEC_NOT_FOUND;
    }

    result = activates[0]->ActivateObject(IID_PPV_ARGS(&state.transform));
    for (UINT32 i = 0; i < activateCount; ++i)
    {
        activates[i]->Release();
    }
    CoTaskMemFree(activates);
    if (FAILED(result))
    {
        return result;
    }

    state.transform.As(&state.codecApi);
    return S_OK;
}

HRESULT ConfigureEncoder(MediaFoundationEncoderState& state,
                         oxr::protocol::VideoCodec codec)
{
    ComPtr<IMFMediaType> inputType;
    ComPtr<IMFMediaType> outputType;
    RETURN_IF_FAILED(CreateMediaTypes(state, codec, &inputType, &outputType));

    RETURN_IF_FAILED(state.transform->SetOutputType(state.outputStreamId, outputType.Get(), 0));
    RETURN_IF_FAILED(state.transform->SetInputType(state.inputStreamId, inputType.Get(), 0));

    if (state.codecApi)
    {
        SetCodecApiBoolean(state.codecApi.Get(), CODECAPI_AVLowLatencyMode, true);
        SetCodecApiUInt32(state.codecApi.Get(), CODECAPI_AVEncCommonMeanBitRate,
                          state.bitrateMbps * 1000u * 1000u);
        SetCodecApiUInt32(state.codecApi.Get(), CODECAPI_AVEncMPVGOPSize,
                          std::max(Config::Get().GetValues().keyframeIntervalSec * state.fps, 1u));
    }

    state.transform->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    state.transform->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
    return S_OK;
}

HRESULT CopyNv12ToSample(const PreparedVideoFrame& frame, IMFSample** sampleOut)
{
    const DWORD length = static_cast<DWORD>(frame.nv12.size());
    ComPtr<IMFMediaBuffer> buffer;
    RETURN_IF_FAILED(MFCreateMemoryBuffer(length, &buffer));

    BYTE* destination = nullptr;
    DWORD maxLength = 0;
    RETURN_IF_FAILED(buffer->Lock(&destination, &maxLength, nullptr));
    if (maxLength < length)
    {
        buffer->Unlock();
        return E_FAIL;
    }
    std::memcpy(destination, frame.nv12.data(), frame.nv12.size());
    buffer->Unlock();
    RETURN_IF_FAILED(buffer->SetCurrentLength(length));

    ComPtr<IMFSample> sample;
    RETURN_IF_FAILED(MFCreateSample(&sample));
    RETURN_IF_FAILED(sample->AddBuffer(buffer.Get()));
    *sampleOut = sample.Detach();
    return S_OK;
}

bool DrainOutput(MediaFoundationEncoderState& state,
                 bool keyframe,
                 int64_t timestampNs,
                 const VideoEncoder::OnNalUnitCallback& callback,
                 bool& emittedOutput)
{
    emittedOutput = false;
    MFT_OUTPUT_STREAM_INFO streamInfo = {};
    HRESULT result = state.transform->GetOutputStreamInfo(state.outputStreamId, &streamInfo);
    if (FAILED(result))
    {
        spdlog::warn("MediaFoundation: failed to query output stream info: {}", HResultString(result));
        return false;
    }

    for (;;)
    {
        ComPtr<IMFSample> sample;
        if ((streamInfo.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES) == 0)
        {
            result = MFCreateSample(&sample);
            if (FAILED(result))
            {
                return false;
            }
            ComPtr<IMFMediaBuffer> buffer;
            result = MFCreateMemoryBuffer(std::max<DWORD>(streamInfo.cbSize, 1024u * 1024u), &buffer);
            if (FAILED(result))
            {
                return false;
            }
            sample->AddBuffer(buffer.Get());
        }

        MFT_OUTPUT_DATA_BUFFER output = {};
        output.dwStreamID = state.outputStreamId;
        output.pSample = sample.Get();
        DWORD status = 0;
        result = state.transform->ProcessOutput(0, 1, &output, &status);
        if (output.pEvents != nullptr)
        {
            output.pEvents->Release();
        }
        if (result == MF_E_TRANSFORM_NEED_MORE_INPUT)
        {
            return true;
        }
        if (result == MF_E_TRANSFORM_STREAM_CHANGE)
        {
            continue;
        }
        if (FAILED(result))
        {
            spdlog::warn("MediaFoundation: ProcessOutput failed: {}", HResultString(result));
            return false;
        }
        if (output.pSample == nullptr)
        {
            continue;
        }

        ComPtr<IMFMediaBuffer> contiguous;
        result = output.pSample->ConvertToContiguousBuffer(&contiguous);
        if (FAILED(result))
        {
            return false;
        }
        BYTE* data = nullptr;
        DWORD maxLength = 0;
        DWORD currentLength = 0;
        result = contiguous->Lock(&data, &maxLength, &currentLength);
        if (SUCCEEDED(result) && data != nullptr && currentLength > 0)
        {
            if (callback)
            {
                callback(data, currentLength, keyframe, timestampNs);
            }
            emittedOutput = true;
        }
        if (SUCCEEDED(result))
        {
            contiguous->Unlock();
        }
    }
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

bool VideoEncoder::SupportsCodec(oxr::protocol::VideoCodec codec)
{
    return codec == oxr::protocol::VideoCodec::H264 ||
           codec == oxr::protocol::VideoCodec::H265;
}

bool VideoEncoder::Initialize(uint32_t width, uint32_t height, uint32_t fps,
                              uint32_t bitrateMbps, const GraphicsContext& graphicsContext,
                              oxr::protocol::VideoCodec codec)
{
    Shutdown();
    if (MediaFoundationSubtype(codec) == GUID_NULL)
    {
        spdlog::error("MediaFoundation: {} is not implemented", CodecDisplayName(codec));
        return false;
    }
    if ((width % 2u) != 0 || (height % 2u) != 0)
    {
        spdlog::error("MediaFoundation: NV12 encode dimensions must be even ({}x{})", width, height);
        return false;
    }

    auto state = std::make_unique<MediaFoundationEncoderState>();
    HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (SUCCEEDED(result))
    {
        state->comInitialized = true;
    }
    else if (result != RPC_E_CHANGED_MODE)
    {
        spdlog::error("MediaFoundation: CoInitializeEx failed: {}", HResultString(result));
        return false;
    }

    result = MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
    if (FAILED(result))
    {
        spdlog::error("MediaFoundation: MFStartup failed: {}", HResultString(result));
        DestroyState(state.release());
        return false;
    }
    state->mfStarted = true;
    state->width = width;
    state->height = height;
    state->fps = std::max(fps, 1u);
    state->bitrateMbps = bitrateMbps;

    result = ActivateEncoder(*state, codec);
    if (FAILED(result))
    {
        spdlog::error("MediaFoundation: no {} encoder MFT available: {}",
                      CodecDisplayName(codec), HResultString(result));
        DestroyState(state.release());
        return false;
    }
    result = ConfigureEncoder(*state, codec);
    if (FAILED(result))
    {
        spdlog::error("MediaFoundation: failed to configure {} encoder: {}",
                      CodecDisplayName(codec), HResultString(result));
        DestroyState(state.release());
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

    spdlog::info("MediaFoundation: initialized {} encoder {}x{} @ {}Hz {}Mbps",
                 CodecDisplayName(codec_), width_, height_, fps_, bitrateMbps_);
    return true;
}

void VideoEncoder::Shutdown()
{
    shuttingDown_.store(true);
    initialized_ = false;
    auto* state = State(platformEncoder_);
    platformEncoder_ = nullptr;
    DestroyState(state);
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
            spdlog::warn("MediaFoundation: dropping frame {}: {}", metrics.frameNumber, reason);
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

    ComPtr<IMFSample> sample;
    HRESULT result = CopyNv12ToSample(state->preparedFrame, &sample);
    if (FAILED(result))
    {
        return finishDroppedFrame("failed to allocate Media Foundation input sample");
    }

    const uint32_t frameDuration = std::max(fps_, 1u);
    const LONGLONG sampleTime = static_cast<LONGLONG>((frameCount_ * 10000000ull) / frameDuration);
    const LONGLONG sampleDuration = static_cast<LONGLONG>(10000000ull / frameDuration);
    sample->SetSampleTime(sampleTime);
    sample->SetSampleDuration(sampleDuration);

    const bool requestedKeyframe = forceKeyframe_.exchange(false);
    if (requestedKeyframe && state->codecApi)
    {
        SetCodecApiUInt32(state->codecApi.Get(), CODECAPI_AVEncVideoForceKeyFrame, 1);
    }

    const auto submitStart = Clock::now();
    result = state->transform->ProcessInput(state->inputStreamId, sample.Get(), 0);
    metrics.encodeSubmitMs = ToMilliseconds(Clock::now() - submitStart);
    if (FAILED(result))
    {
        return finishDroppedFrame(HResultString(result).c_str());
    }

    bool emittedOutput = false;
    if (!DrainOutput(*state, requestedKeyframe || frameCount_ == 0, timestampNs, callback, emittedOutput))
    {
        return finishDroppedFrame("Media Foundation output drain failed");
    }

    frameCount_++;
    metrics.keyframe = requestedKeyframe || frameCount_ == 1;
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
    auto* state = State(platformEncoder_);
    if (state != nullptr)
    {
        state->bitrateMbps = bitrateMbps;
        if (state->codecApi)
        {
            SetCodecApiUInt32(state->codecApi.Get(), CODECAPI_AVEncCommonMeanBitRate,
                              bitrateMbps * 1000u * 1000u);
        }
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
