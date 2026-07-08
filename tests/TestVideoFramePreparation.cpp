// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "VideoFramePreparation.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace
{

struct Yuv
{
    uint8_t y = 16;
    uint8_t u = 128;
    uint8_t v = 128;
};

uint8_t ClampByte(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

Yuv RgbToNv12(uint8_t r, uint8_t g, uint8_t b)
{
    Yuv sample;
    sample.y = ClampByte(16 + ((47 * r + 157 * g + 16 * b + 128) >> 8));
    sample.u = ClampByte(128 + ((-26 * r - 87 * g + 112 * b + 128) >> 8));
    sample.v = ClampByte(128 + ((112 * r - 102 * g - 10 * b + 128) >> 8));
    return sample;
}

std::array<uint8_t, 4> Rgba(uint8_t r, uint8_t g, uint8_t b)
{
    return {r, g, b, 255};
}

std::array<uint8_t, 4> Bgra(uint8_t r, uint8_t g, uint8_t b)
{
    return {b, g, r, 255};
}

void PutPixel(std::vector<uint8_t>& pixels,
              uint32_t strideBytes,
              uint32_t x,
              uint32_t y,
              const std::array<uint8_t, 4>& rgba)
{
    const size_t offset = static_cast<size_t>(y) * strideBytes + static_cast<size_t>(x) * 4u;
    pixels[offset + 0] = rgba[0];
    pixels[offset + 1] = rgba[1];
    pixels[offset + 2] = rgba[2];
    pixels[offset + 3] = rgba[3];
}

} // namespace

TEST_CASE("VideoFramePreparer fills black NV12 frames", "[video][frame-prep]")
{
    PreparedVideoFrame frame;
    REQUIRE(VideoFramePreparer::FillBlack(4, 4, frame));
    CHECK(frame.width == 4);
    CHECK(frame.height == 4);
    CHECK(frame.yStride == 4);
    CHECK(frame.uvStride == 4);

    const size_t yBytes = static_cast<size_t>(frame.yStride) * frame.height;
    const size_t uvBytes = static_cast<size_t>(frame.uvStride) * (frame.height / 2u);
    CHECK(std::all_of(frame.YPlane(), frame.YPlane() + yBytes, [](uint8_t value) {
        return value == 16;
    }));
    CHECK(std::all_of(frame.UVPlane(), frame.UVPlane() + uvBytes, [](uint8_t value) {
        return value == 128;
    }));

    CHECK_FALSE(VideoFramePreparer::FillBlack(3, 4, frame));
}

TEST_CASE("VideoFramePreparer crops and scales RGBA pixels to NV12", "[video][frame-prep]")
{
    constexpr uint32_t width = 4;
    constexpr uint32_t height = 2;
    constexpr uint32_t stride = width * 4u + 8u;
    std::vector<uint8_t> pixels(static_cast<size_t>(stride) * height, 0xff);

    PutPixel(pixels, stride, 0, 0, Rgba(0, 0, 0));
    PutPixel(pixels, stride, 1, 0, Rgba(255, 0, 0));
    PutPixel(pixels, stride, 2, 0, Rgba(0, 255, 0));
    PutPixel(pixels, stride, 3, 0, Rgba(0, 0, 255));
    PutPixel(pixels, stride, 0, 1, Rgba(255, 255, 255));
    PutPixel(pixels, stride, 1, 1, Rgba(0, 255, 255));
    PutPixel(pixels, stride, 2, 1, Rgba(255, 0, 255));
    PutPixel(pixels, stride, 3, 1, Rgba(255, 255, 0));

    VideoSourcePixels source;
    source.pixels = pixels.data();
    source.width = width;
    source.height = height;
    source.strideBytes = stride;
    source.layout = VideoSourcePixelLayout::Rgba;

    FrameImageSource metadata;
    metadata.sourceX = 2;
    metadata.sourceY = 0;
    metadata.sourceWidth = 2;
    metadata.sourceHeight = 2;

    PreparedVideoFrame frame;
    REQUIRE(VideoFramePreparer::PrepareStereoPixels(
        source, metadata, nullptr, nullptr, false, 4, 2, frame));

    const Yuv green = RgbToNv12(0, 255, 0);
    const Yuv blue = RgbToNv12(0, 0, 255);
    const Yuv magenta = RgbToNv12(255, 0, 255);
    const Yuv yellow = RgbToNv12(255, 255, 0);

    CHECK(frame.YPlane()[0] == green.y);
    CHECK(frame.YPlane()[1] == green.y);
    CHECK(frame.YPlane()[2] == blue.y);
    CHECK(frame.YPlane()[3] == blue.y);
    CHECK(frame.YPlane()[4] == magenta.y);
    CHECK(frame.YPlane()[5] == magenta.y);
    CHECK(frame.YPlane()[6] == yellow.y);
    CHECK(frame.YPlane()[7] == yellow.y);

    CHECK(frame.UVPlane()[0] == static_cast<uint8_t>((green.u + green.u + magenta.u + magenta.u) / 4u));
    CHECK(frame.UVPlane()[1] == static_cast<uint8_t>((green.v + green.v + magenta.v + magenta.v) / 4u));
    CHECK(frame.UVPlane()[2] == static_cast<uint8_t>((blue.u + blue.u + yellow.u + yellow.u) / 4u));
    CHECK(frame.UVPlane()[3] == static_cast<uint8_t>((blue.v + blue.v + yellow.v + yellow.v) / 4u));
}

TEST_CASE("VideoFramePreparer handles BGRA and RGBA stereo inputs", "[video][frame-prep]")
{
    std::vector<uint8_t> leftPixels(2u * 2u * 4u);
    std::vector<uint8_t> rightPixels(2u * 2u * 4u);
    for (uint32_t y = 0; y < 2; ++y)
    {
        for (uint32_t x = 0; x < 2; ++x)
        {
            PutPixel(leftPixels, 2u * 4u, x, y, Bgra(255, 0, 0));
            PutPixel(rightPixels, 2u * 4u, x, y, Rgba(0, 0, 255));
        }
    }

    const VideoSourcePixels left = {
        leftPixels.data(),
        2,
        2,
        2u * 4u,
        VideoSourcePixelLayout::Bgra,
    };
    const VideoSourcePixels right = {
        rightPixels.data(),
        2,
        2,
        2u * 4u,
        VideoSourcePixelLayout::Rgba,
    };

    FrameImageSource metadata;
    PreparedVideoFrame frame;
    REQUIRE(VideoFramePreparer::PrepareStereoPixels(
        left, metadata, &right, &metadata, true, 4, 2, frame));

    const Yuv red = RgbToNv12(255, 0, 0);
    const Yuv blue = RgbToNv12(0, 0, 255);
    CHECK(frame.YPlane()[0] == red.y);
    CHECK(frame.YPlane()[1] == red.y);
    CHECK(frame.YPlane()[2] == blue.y);
    CHECK(frame.YPlane()[3] == blue.y);
    CHECK(frame.YPlane()[4] == red.y);
    CHECK(frame.YPlane()[5] == red.y);
    CHECK(frame.YPlane()[6] == blue.y);
    CHECK(frame.YPlane()[7] == blue.y);
    CHECK(frame.UVPlane()[0] == red.u);
    CHECK(frame.UVPlane()[1] == red.v);
    CHECK(frame.UVPlane()[2] == blue.u);
    CHECK(frame.UVPlane()[3] == blue.v);
}

TEST_CASE("VideoFramePreparer rejects invalid crop rectangles", "[video][frame-prep]")
{
    std::vector<uint8_t> pixels(2u * 2u * 4u, 0);
    const VideoSourcePixels source = {
        pixels.data(),
        2,
        2,
        2u * 4u,
        VideoSourcePixelLayout::Rgba,
    };

    FrameImageSource metadata;
    metadata.sourceX = 1;
    metadata.sourceY = 0;
    metadata.sourceWidth = 2;
    metadata.sourceHeight = 2;

    PreparedVideoFrame frame;
    CHECK_FALSE(VideoFramePreparer::PrepareStereoPixels(
        source, metadata, nullptr, nullptr, false, 2, 2, frame));
}
