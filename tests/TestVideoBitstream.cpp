// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "VideoBitstream.h"

#include <array>
#include <cstdint>
#include <vector>

using oxr::protocol::VideoCodec;

namespace
{

std::vector<uint8_t> LengthPrefixed(const std::vector<std::vector<uint8_t>>& nalUnits)
{
    std::vector<uint8_t> bytes;
    for (const auto& nal : nalUnits)
    {
        const uint32_t size = static_cast<uint32_t>(nal.size());
        bytes.push_back(static_cast<uint8_t>((size >> 24u) & 0xffu));
        bytes.push_back(static_cast<uint8_t>((size >> 16u) & 0xffu));
        bytes.push_back(static_cast<uint8_t>((size >> 8u) & 0xffu));
        bytes.push_back(static_cast<uint8_t>(size & 0xffu));
        bytes.insert(bytes.end(), nal.begin(), nal.end());
    }
    return bytes;
}

bool HasFourByteStartCode(const std::vector<uint8_t>& bytes)
{
    return bytes.size() >= 4 &&
        bytes[0] == 0x00 &&
        bytes[1] == 0x00 &&
        bytes[2] == 0x00 &&
        bytes[3] == 0x01;
}

} // namespace

TEST_CASE("VideoBitstream normalizes Annex B H.264 and detects headers", "[video][bitstream]")
{
    const std::vector<uint8_t> sample = {
        0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x1f,
        0x00, 0x00, 0x00, 0x01, 0x68, 0xce, 0x06,
        0x00, 0x00, 0x01, 0x65, 0x88, 0x84,
    };

    CHECK(oxrsys::video_bitstream::IsAnnexB(sample.data(), sample.size()));

    const auto units = oxrsys::video_bitstream::NormalizeToAnnexB(
        VideoCodec::H264, sample.data(), sample.size(), false);
    REQUIRE(units.size() == 3);
    CHECK(HasFourByteStartCode(units[0].bytes));
    CHECK(units[0].parameterSet);
    CHECK(units[1].parameterSet);
    CHECK_FALSE(units[0].keyframe);
    CHECK(units[2].keyframe);

    size_t visited = 0;
    size_t keyframes = 0;
    CHECK(oxrsys::video_bitstream::VisitAnnexBUnits(
        VideoCodec::H264,
        sample.data(),
        sample.size(),
        false,
        [&](const uint8_t* data, size_t size, bool keyframe) {
            CHECK(oxrsys::video_bitstream::IsAnnexB(data, size));
            ++visited;
            if (keyframe)
            {
                ++keyframes;
            }
        }));
    CHECK(visited == 3);
    CHECK(keyframes == 1);
}

TEST_CASE("VideoBitstream normalizes length-prefixed H.265 samples", "[video][bitstream]")
{
    const std::vector<uint8_t> sample = LengthPrefixed({
        {0x40, 0x01, 0x0c}, // VPS, nal_unit_type 32
        {0x42, 0x01, 0x01}, // SPS, nal_unit_type 33
        {0x26, 0x01, 0xaa}, // IDR_W_RADL, nal_unit_type 19
    });

    const auto units = oxrsys::video_bitstream::NormalizeToAnnexB(
        VideoCodec::H265, sample.data(), sample.size(), false);
    REQUIRE(units.size() == 3);
    CHECK(HasFourByteStartCode(units[0].bytes));
    CHECK(units[0].parameterSet);
    CHECK(units[1].parameterSet);
    CHECK_FALSE(units[0].keyframe);
    CHECK(units[2].keyframe);
}

TEST_CASE("VideoBitstream can mark opaque samples as keyframes on request", "[video][bitstream]")
{
    const std::array<uint8_t, 3> pFrame = {0x41, 0x9a, 0x22};

    const auto units = oxrsys::video_bitstream::NormalizeToAnnexB(
        VideoCodec::H264, pFrame.data(), pFrame.size(), true);
    REQUIRE(units.size() == 1);
    CHECK(HasFourByteStartCode(units[0].bytes));
    CHECK(units[0].keyframe);
}
