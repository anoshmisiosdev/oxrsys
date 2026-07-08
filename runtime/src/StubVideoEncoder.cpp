// SPDX-License-Identifier: MPL-2.0

#include "VideoEncoder.h"

#include <spdlog/spdlog.h>

#include <utility>

VideoEncoder::VideoEncoder() = default;

VideoEncoder::~VideoEncoder()
{
    Shutdown();
}

bool VideoEncoder::SupportsFoveatedEncoding(const GraphicsContext& /*graphicsContext*/)
{
    return false;
}

bool VideoEncoder::SupportsCodec(oxr::protocol::VideoCodec /*codec*/)
{
    return false;
}

bool VideoEncoder::Initialize(uint32_t /*width*/, uint32_t /*height*/, uint32_t /*fps*/,
                              uint32_t /*bitrateMbps*/, const GraphicsContext& /*graphicsContext*/,
                              oxr::protocol::VideoCodec /*codec*/)
{
    spdlog::error("VideoEncoder: no platform encoder backend was built");
    initialized_ = false;
    return false;
}

void VideoEncoder::Shutdown()
{
    initialized_ = false;
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

bool VideoEncoder::EncodeInternal(FrameSource /*frameSource*/, bool /*stereo*/,
                                  int64_t timestampNs, OnNalUnitCallback /*callback*/,
                                  OnFrameEncodedCallback frameCallback)
{
    droppedFrameCount_.fetch_add(1);
    FrameMetrics metrics = {};
    metrics.frameNumber = frameNumberCounter_.fetch_add(1) + 1;
    metrics.timestampNs = timestampNs;
    metrics.frameDropped = true;
    if (frameCallback)
    {
        frameCallback(metrics);
    }
    return false;
}

void VideoEncoder::ForceKeyframe()
{
    forceKeyframe_.store(true);
}

void VideoEncoder::SetBitrate(uint32_t bitrateMbps)
{
    bitrateMbps_ = bitrateMbps;
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
