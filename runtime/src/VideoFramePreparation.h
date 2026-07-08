// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "GraphicsTypes.h"

#include <cstddef>
#include <cstdint>
#include <vector>

struct PreparedVideoFrame
{
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t yStride = 0;
    uint32_t uvStride = 0;
    std::vector<uint8_t> nv12;

    uint8_t* YPlane() { return nv12.data(); }
    const uint8_t* YPlane() const { return nv12.data(); }

    uint8_t* UVPlane() { return nv12.data() + static_cast<size_t>(yStride) * height; }
    const uint8_t* UVPlane() const { return nv12.data() + static_cast<size_t>(yStride) * height; }
};

enum class VideoSourcePixelLayout
{
    Rgba,
    Bgra,
};

struct VideoSourcePixels
{
    const uint8_t* pixels = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t strideBytes = 0;
    VideoSourcePixelLayout layout = VideoSourcePixelLayout::Rgba;
};

class VideoFramePreparer
{
public:
    bool PrepareStereo(FrameSource frameSource,
                       bool stereo,
                       const GraphicsContext& graphicsContext,
                       uint32_t outputWidth,
                       uint32_t outputHeight,
                       PreparedVideoFrame& output);

    static bool FillBlack(uint32_t outputWidth,
                          uint32_t outputHeight,
                          PreparedVideoFrame& output);

    static bool PrepareStereoPixels(const VideoSourcePixels& leftPixels,
                                    const FrameImageSource& leftMetadata,
                                    const VideoSourcePixels* rightPixels,
                                    const FrameImageSource* rightMetadata,
                                    bool stereo,
                                    uint32_t outputWidth,
                                    uint32_t outputHeight,
                                    PreparedVideoFrame& output);
};
