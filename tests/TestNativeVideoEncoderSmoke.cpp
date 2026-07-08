// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "VideoBitstream.h"
#include "VideoEncoder.h"

#include <cstddef>
#include <cstdint>
#include <vector>

using oxr::protocol::VideoCodec;

namespace
{

struct EncodedStats
{
    size_t units = 0;
    size_t keyframes = 0;
    size_t parameterSets = 0;
    size_t nonKeyframeSlices = 0;
};

EncodedStats EncodeOne(VideoEncoder& encoder, VideoCodec codec, int64_t timestampNs)
{
    EncodedStats stats;
    const bool ok = encoder.Encode(
        {},
        timestampNs,
        [&](const uint8_t* data, size_t size, bool keyframe, int64_t /*timestamp*/) {
            ++stats.units;
            CHECK(oxrsys::video_bitstream::IsAnnexB(data, size));
            if (keyframe)
            {
                ++stats.keyframes;
            }
            if (oxrsys::video_bitstream::IsParameterSetNal(codec, data, size))
            {
                ++stats.parameterSets;
            }
            if (!keyframe &&
                !oxrsys::video_bitstream::IsParameterSetNal(codec, data, size))
            {
                ++stats.nonKeyframeSlices;
            }
        });
    REQUIRE(ok);
    CHECK(stats.units > 0);
    return stats;
}

void SmokeEncodeCodec(VideoCodec codec)
{
    VideoEncoder encoder;
    GraphicsContext graphicsContext = {};
    REQUIRE(encoder.Initialize(128, 128, 60, 10, graphicsContext, codec));

    encoder.ForceKeyframe();
    const EncodedStats first = EncodeOne(encoder, codec, 1000);
    CHECK(first.keyframes > 0);
    CHECK(first.parameterSets > 0);

    EncodedStats later = {};
    for (int frame = 0; frame < 3; ++frame)
    {
        const EncodedStats current = EncodeOne(encoder, codec, 2000 + frame);
        later.units += current.units;
        later.keyframes += current.keyframes;
        later.parameterSets += current.parameterSets;
        later.nonKeyframeSlices += current.nonKeyframeSlices;
    }
    CHECK(later.units > 0);
    CHECK(later.nonKeyframeSlices > 0);
}

} // namespace

TEST_CASE("Native video encoder emits Annex B keyframes and P-frames when hardware is available",
          "[video][encoder][hardware]")
{
    const VideoEncoder::BackendCapabilities capabilities =
        VideoEncoder::QueryBackendCapabilities(nullptr);
    if (!capabilities.supportsH264 && !capabilities.supportsH265)
    {
        SKIP(capabilities.unsupportedReason.empty()
                 ? "no hardware H.264/H.265 encoder backend available"
                 : capabilities.unsupportedReason);
    }

    if (capabilities.supportsH265)
    {
        DYNAMIC_SECTION(capabilities.backendName << " H.265")
        {
            SmokeEncodeCodec(VideoCodec::H265);
        }
    }
    if (capabilities.supportsH264)
    {
        DYNAMIC_SECTION(capabilities.backendName << " H.264")
        {
            SmokeEncodeCodec(VideoCodec::H264);
        }
    }
}
