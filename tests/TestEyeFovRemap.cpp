// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "EyeFovRemap.h"

#include <cmath>

using Catch::Matchers::WithinAbs;
using namespace oxrsys::video;

namespace
{

FrameFov Fov(float left, float right, float up, float down)
{
    FrameFov fov;
    fov.angleLeft = left;
    fov.angleRight = right;
    fov.angleUp = up;
    fov.angleDown = down;
    return fov;
}

// Where a display-space UV lands in the submitted image.
float SourceU(const FovRemap& remap, float u) { return u * remap.uScale + remap.uOffset; }
float SourceV(const FovRemap& remap, float v) { return v * remap.vScale + remap.vOffset; }

} // namespace

TEST_CASE("Eye fov remap is the identity for matching fovs", "[video][fov]")
{
    const FrameFov fov = Fov(-0.9f, 0.75f, 0.84f, -0.87f);
    CHECK_FALSE(FovNeedsRemap(fov, fov));
    const FovRemap remap = ComputeFovRemap(fov, fov);
    CHECK_THAT(remap.uScale, WithinAbs(1.0, 1e-6));
    CHECK_THAT(remap.uOffset, WithinAbs(0.0, 1e-6));
    CHECK_THAT(remap.vScale, WithinAbs(1.0, 1e-6));
    CHECK_THAT(remap.vOffset, WithinAbs(0.0, 1e-6));
}

TEST_CASE("Eye fov remap maps directions, not pixels", "[video][fov]")
{
    // HITMAN 3 through OpenComposite rendered with the placeholder symmetric view while
    // a Quest 2 displayed its real asymmetric left-eye fov.
    const FrameFov rendered = Fov(-0.8204f, 0.8204f, 0.8727f, -0.8727f);
    const FrameFov display = Fov(-0.9076f, 0.7330f, 0.8378f, -0.8727f);
    REQUIRE(FovNeedsRemap(rendered, display));
    const FovRemap remap = ComputeFovRemap(rendered, display);

    // Straight ahead (tangent 0) is at the centre of the submitted image; in the display it
    // sits where tangent 0 falls within [tan(left), tan(right)].
    const float straightAheadU =
        -std::tan(display.angleLeft) / (std::tan(display.angleRight) - std::tan(display.angleLeft));
    CHECK_THAT(SourceU(remap, straightAheadU), WithinAbs(0.5, 1e-5));
    const float straightAheadV =
        std::tan(display.angleUp) / (std::tan(display.angleUp) - std::tan(display.angleDown));
    CHECK_THAT(SourceV(remap, straightAheadV), WithinAbs(0.5, 1e-5));

    // The display's left edge looks further out than the application rendered: no source.
    CHECK(SourceU(remap, 0.0f) < 0.0f);
    // Its right edge is inside the rendered image.
    CHECK(SourceU(remap, 1.0f) < 1.0f);
    CHECK(SourceU(remap, 1.0f) > 0.9f);
    // Down matches, so the bottom row maps to the bottom row.
    CHECK_THAT(SourceV(remap, 1.0f), WithinAbs(1.0, 1e-5));
}

TEST_CASE("Eye fov remap ignores unusable fovs", "[video][fov]")
{
    const FrameFov good = Fov(-0.8f, 0.8f, 0.8f, -0.8f);
    CHECK_FALSE(FovNeedsRemap(FrameFov{}, good));
    CHECK_FALSE(FovNeedsRemap(good, Fov(-0.8f, 0.8f, NAN, -0.8f)));
    CHECK_FALSE(FovNeedsRemap(good, Fov(0.8f, -0.8f, 0.8f, -0.8f)));
}
