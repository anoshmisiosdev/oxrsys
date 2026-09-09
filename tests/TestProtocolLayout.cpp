// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include <oxrsys/protocol/Foveation.h>
#include <oxrsys/protocol/Protocol.h>

#include <cstddef>

using namespace oxr::protocol;

TEST_CASE("C++ protocol layouts match the documented wire format", "[protocol]")
{
    STATIC_REQUIRE(SERVER_ANNOUNCE_BASE_SIZE == 92);
    STATIC_REQUIRE(CLIENT_CONNECT_BASE_SIZE == 80);
    STATIC_REQUIRE(LATENCY_REPORT_BASE_SIZE == 20);
    STATIC_REQUIRE(sizeof(ServerAnnounce) == 144);
    STATIC_REQUIRE(offsetof(ServerAnnounce, serverFeatures) == SERVER_ANNOUNCE_BASE_SIZE);
    STATIC_REQUIRE(sizeof(ClientConnect) == 96);
    STATIC_REQUIRE(offsetof(ClientConnect, clientCapabilities) == CLIENT_CONNECT_BASE_SIZE);
    STATIC_REQUIRE(sizeof(VideoPacketHeader) == 24);
    STATIC_REQUIRE(offsetof(VideoPacketHeader, fecGroupLastPacketPayloadSize) == 12);
    STATIC_REQUIRE(offsetof(VideoPacketHeader, reserved) == 14);
    STATIC_REQUIRE(offsetof(VideoPacketHeader, presentationTimeNs) == 16);
    STATIC_REQUIRE(sizeof(TcpRecordHeader) == 12);
    STATIC_REQUIRE(sizeof(TcpVideoNalHeader) == 24);
    STATIC_REQUIRE(sizeof(TcpRenderPose) == 48);
    STATIC_REQUIRE(sizeof(TcpAudioHeader) == 24);
    STATIC_REQUIRE(sizeof(AudioPacketHeader) == 32);
    STATIC_REQUIRE(TCP_RECORD_MAGIC == 0x4f585255);
    STATIC_REQUIRE(STREAMING_MIN_BITRATE_MBPS == 1);
    STATIC_REQUIRE(STREAMING_MAX_BITRATE_MBPS == 200);
    STATIC_REQUIRE(CLIENT_MAX_BITRATE_USE_SERVER_CONFIG == 0);

    STATIC_REQUIRE(sizeof(LatencyReport) == 40);
    STATIC_REQUIRE(sizeof(RequestKeyframe) == 12);
    STATIC_REQUIRE(sizeof(HapticsCommand) == 16);
    STATIC_REQUIRE(sizeof(NackRequest) == 24);

    STATIC_REQUIRE(sizeof(TrackingPacket) == 1008);
    STATIC_REQUIRE(offsetof(TrackingPacket, headLinearVelocity) == 152);
    STATIC_REQUIRE(offsetof(TrackingPacket, headAngularVelocity) == 164);
    STATIC_REQUIRE(offsetof(TrackingPacket, leftHandJoints) == 176);
    STATIC_REQUIRE(offsetof(TrackingPacket, rightHandJoints) == 592);
    STATIC_REQUIRE(TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE == 0x0004);
    STATIC_REQUIRE(TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE == 0x0008);
}

TEST_CASE("Foveated encoding presets calculate ALVR-style optimized eye sizes", "[protocol][foveation]")
{
    const FoveationLayout light =
        CalculateFoveationLayout(2144, 2144, FoveationPreset::Light);
    CHECK(light.optimizedEyeWidth == 1728);
    CHECK(light.optimizedEyeHeight == 1504);
    CHECK(light.parameters.edgeRatioX == 2.0f);
    CHECK(light.parameters.edgeRatioY == 3.0f);

    const FoveationLayout medium =
        CalculateFoveationLayout(2144, 2144, FoveationPreset::Medium);
    CHECK(medium.optimizedEyeWidth == 1280);
    CHECK(medium.optimizedEyeHeight == 1120);
    CHECK(medium.eyeWidthRatio > 0.98f);
    CHECK(medium.eyeHeightRatio > 0.97f);

    const FoveationLayout high =
        CalculateFoveationLayout(2144, 2144, FoveationPreset::High);
    CHECK(high.optimizedEyeWidth == 992);
    CHECK(high.optimizedEyeHeight == 896);

    const FoveationLayout off =
        CalculateFoveationLayout(2144, 2144, FoveationPreset::Off);
    CHECK(off.optimizedEyeWidth == 2144);
    CHECK(off.optimizedEyeHeight == 2144);
}

TEST_CASE("Quest foveated layout stays aligned and rejects incoherent targets", "[protocol][foveation]")
{
    const FoveationPreset presets[] = {
        FoveationPreset::Light,
        FoveationPreset::Medium,
        FoveationPreset::High,
    };

    for (FoveationPreset preset : presets)
    {
        const FoveationLayout layout =
            CalculateFoveationLayout(1512, 1680, preset);
        CHECK((layout.optimizedEyeWidth % 32) == 0);
        CHECK((layout.optimizedEyeHeight % 32) == 0);
        CHECK(layout.optimizedEyeWidth < layout.targetEyeWidth);
        CHECK(layout.optimizedEyeHeight < layout.targetEyeHeight);
        CHECK(layout.eyeWidthRatio > 0.0f);
        CHECK(layout.eyeWidthRatio <= 1.0f);
        CHECK(layout.eyeHeightRatio > 0.0f);
        CHECK(layout.eyeHeightRatio <= 1.0f);
        CHECK(IsFoveatedEncodingLayoutUsable(layout, 1512, 1680));
        CHECK_FALSE(IsFoveatedEncodingLayoutUsable(layout, 1520, 1680));
    }

    const FoveationLayout scaledLayout =
        CalculateFoveationLayout(1136, 1264, FoveationPreset::Light);
    CHECK_FALSE(IsFoveatedEncodingLayoutUsable(scaledLayout, 1512, 1680));
}

// Regression guard for the FFE-at-scale fix: StreamingServer now always
// computes the foveation layout from the FULL render eye (so it stays coherent
// with the encoder's source-texture validation and the client's un-warp, which
// both use renderWidth), and engages foveation whenever the optimized eye fits
// within the pixel budget the user chose via resolution_scale. At
// resolution_scale=0.75 (SUPERHOT: 1512x1680 render per eye -> 1136x1264
// scaled) the Medium/High presets must be usable against the full render eye
// AND land within that scaled budget while encoding fewer pixels than a uniform
// 0.75 downscale — that reduction is what relieves the software-HEVC bottleneck.
TEST_CASE("Foveation from full render coheres and fits the resolution_scale budget", "[protocol][foveation]")
{
    // Full render eye (SUPERHOT VR per-eye) and the 0.75 uniform-scale budget.
    const uint32_t renderEyeW = 1512;
    const uint32_t renderEyeH = 1680;
    const uint32_t scaledEyeW = 1136; // round(1512*0.75) aligned up to 16
    const uint32_t scaledEyeH = 1264; // round(1680*0.75) aligned up to 16

    const uint64_t uniformEncodedPixels =
        static_cast<uint64_t>(scaledEyeW) * 2ull * scaledEyeH;

    for (FoveationPreset preset : {FoveationPreset::Medium, FoveationPreset::High})
    {
        const FoveationLayout layout =
            CalculateFoveationLayout(renderEyeW, renderEyeH, preset);

        // Coherent with the encoder (targetEye == source render eye) and client.
        CHECK(IsFoveatedEncodingLayoutUsable(layout, renderEyeW, renderEyeH));

        // Fits within the user's resolution_scale=0.75 pixel budget, so
        // StreamingServer will engage it instead of the uniform downscale.
        CHECK(layout.optimizedEyeWidth <= scaledEyeW);
        CHECK(layout.optimizedEyeHeight <= scaledEyeH);

        // Center stays at (or above) the uniform-scale sampling density while
        // the whole frame encodes fewer pixels than uniform 0.75.
        const uint64_t foveatedEncodedPixels =
            static_cast<uint64_t>(layout.optimizedEyeWidth) * 2ull *
            layout.optimizedEyeHeight;
        CHECK(foveatedEncodedPixels < uniformEncodedPixels);
    }
}
