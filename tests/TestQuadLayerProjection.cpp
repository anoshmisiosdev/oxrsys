// SPDX-License-Identifier: MPL-2.0
//
// The CPU half of quad-layer compositing: quad centre pose + metric size +
// view pose/FOV -> screen-space rect, and the OpenXR flag/enum translation.
//
// These run without a GPU on purpose. When the Metal pixel tests in
// TestQuadLayerMetal.mm fail, a green run here says the matrices are fine and
// the fault is in the shader or the blend state; a red run here says the
// opposite. Sign errors in the projection (a Y flip, a handedness slip) are
// exactly the failure these are shaped to catch.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "CompositionLayerQuad.h"
#include "QuadLayerProjection.h"

using Catch::Approx;
using namespace oxrsys::quad;

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kImageWidth = 200;
constexpr uint32_t kImageHeight = 100;

// A viewer at the origin looking down -Z with a symmetric 90-degree frustum.
// tan(45 degrees) == 1, so the projection's X and Y scales are both 1 and a
// point at (x, y, -1) lands at NDC (x, y) -- the arithmetic stays checkable by
// hand.
FrameEyeView SymmetricView()
{
    FrameEyeView view = {};
    view.pose.orientation[0] = 0.0f;
    view.pose.orientation[1] = 0.0f;
    view.pose.orientation[2] = 0.0f;
    view.pose.orientation[3] = 1.0f;
    view.pose.position[0] = 0.0f;
    view.pose.position[1] = 0.0f;
    view.pose.position[2] = 0.0f;
    view.fov.angleLeft = -kPi / 4.0f;
    view.fov.angleRight = kPi / 4.0f;
    view.fov.angleUp = kPi / 4.0f;
    view.fov.angleDown = -kPi / 4.0f;
    view.valid = true;
    return view;
}

FramePose PoseAt(float x, float y, float z)
{
    FramePose pose = {};
    pose.orientation[3] = 1.0f;
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;
    return pose;
}

ScreenRect RectFor(const FrameEyeView& view, const FramePose& pose, float width, float height)
{
    const QuadCorners corners = ProjectQuad(view, pose, width, height);
    REQUIRE(corners.valid);
    ScreenRect rect = {};
    REQUIRE(ComputeScreenRect(corners, kImageWidth, kImageHeight, rect));
    return rect;
}

} // namespace

TEST_CASE("A quad filling the frustum projects to the whole eye image")
{
    // 2m x 2m at 1m in front of a 90-degree frustum exactly fills it.
    const ScreenRect rect = RectFor(SymmetricView(), PoseAt(0.0f, 0.0f, -1.0f), 2.0f, 2.0f);

    CHECK(rect.minX == Approx(0.0f).margin(1e-3));
    CHECK(rect.maxX == Approx(static_cast<float>(kImageWidth)).margin(1e-3));
    CHECK(rect.minY == Approx(0.0f).margin(1e-3));
    CHECK(rect.maxY == Approx(static_cast<float>(kImageHeight)).margin(1e-3));
}

TEST_CASE("Quad size in metres scales the projected rect")
{
    // Half the metric size at the same distance covers half the frustum, so it
    // occupies the middle half of each axis.
    const ScreenRect rect = RectFor(SymmetricView(), PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);

    CHECK(rect.minX == Approx(kImageWidth * 0.25f).margin(1e-3));
    CHECK(rect.maxX == Approx(kImageWidth * 0.75f).margin(1e-3));
    CHECK(rect.minY == Approx(kImageHeight * 0.25f).margin(1e-3));
    CHECK(rect.maxY == Approx(kImageHeight * 0.75f).margin(1e-3));
}

TEST_CASE("Doubling the distance halves the projected rect")
{
    const FrameEyeView view = SymmetricView();
    const ScreenRect near = RectFor(view, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    const ScreenRect far = RectFor(view, PoseAt(0.0f, 0.0f, -2.0f), 1.0f, 1.0f);

    CHECK((far.maxX - far.minX) == Approx((near.maxX - near.minX) * 0.5f).margin(1e-3));
    CHECK((far.maxY - far.minY) == Approx((near.maxY - near.minY) * 0.5f).margin(1e-3));
    // Both stay centred.
    CHECK((far.minX + far.maxX) == Approx(static_cast<float>(kImageWidth)).margin(1e-3));
}

TEST_CASE("A quad to the viewer's right projects to the right of the image")
{
    // +X is right in OpenXR's right-handed, -Z-forward convention, and larger
    // pixel X is right in the image.
    const ScreenRect rect = RectFor(SymmetricView(), PoseAt(0.5f, 0.0f, -1.0f), 1.0f, 1.0f);

    CHECK(rect.minX == Approx(kImageWidth * 0.5f).margin(1e-3));
    CHECK(rect.maxX == Approx(static_cast<float>(kImageWidth)).margin(1e-3));
    // Unmoved vertically.
    CHECK(rect.minY == Approx(kImageHeight * 0.25f).margin(1e-3));
    CHECK(rect.maxY == Approx(kImageHeight * 0.75f).margin(1e-3));
}

TEST_CASE("A quad above the viewer projects to the top of the image")
{
    // The one that catches a Y-flip: +Y is up in OpenXR but y=0 is the *top*
    // row of a Metal render target, so "up" must mean a smaller pixel Y.
    const ScreenRect rect = RectFor(SymmetricView(), PoseAt(0.0f, 0.5f, -1.0f), 1.0f, 1.0f);

    CHECK(rect.minY == Approx(0.0f).margin(1e-3));
    CHECK(rect.maxY == Approx(kImageHeight * 0.5f).margin(1e-3));
    CHECK(rect.minX == Approx(kImageWidth * 0.25f).margin(1e-3));
}

TEST_CASE("A quad behind the viewer has no screen rect")
{
    const QuadCorners corners = ProjectQuad(SymmetricView(), PoseAt(0.0f, 0.0f, 1.0f), 1.0f, 1.0f);
    // The corners are still well-formed clip positions; it is the *rect* that
    // is meaningless, because every corner has w < 0.
    REQUIRE(corners.valid);
    ScreenRect rect = {};
    CHECK_FALSE(ComputeScreenRect(corners, kImageWidth, kImageHeight, rect));
}

TEST_CASE("A quad straddling the near plane is left to the rasteriser")
{
    // Rotated so one edge is behind the eye: no finite axis-aligned rect, and
    // the renderer must hand the homogeneous corners to the GPU clipper.
    FramePose pose = PoseAt(0.0f, 0.0f, -0.05f);
    // 90-degree yaw: the quad's plane now contains the view direction.
    const float halfAngle = kPi / 4.0f;
    pose.orientation[1] = std::sin(halfAngle);
    pose.orientation[3] = std::cos(halfAngle);

    const QuadCorners corners = ProjectQuad(SymmetricView(), pose, 4.0f, 1.0f);
    REQUIRE(corners.valid);
    ScreenRect rect = {};
    CHECK_FALSE(ComputeScreenRect(corners, kImageWidth, kImageHeight, rect));
}

TEST_CASE("The view pose moves the quad in the opposite direction")
{
    FrameEyeView view = SymmetricView();
    view.pose.position[0] = 0.5f;

    // Viewer steps right; a world-fixed quad slides left in the image.
    const ScreenRect rect = RectFor(view, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    CHECK(rect.minX == Approx(0.0f).margin(1e-3));
    CHECK(rect.maxX == Approx(kImageWidth * 0.5f).margin(1e-3));
}

TEST_CASE("An asymmetric FOV shifts the projected centre")
{
    // A frustum whose right half-angle is wider than its left puts the forward
    // axis left of the image centre.
    FrameEyeView view = SymmetricView();
    view.fov.angleLeft = -std::atan(1.0f);
    view.fov.angleRight = std::atan(3.0f);

    const ScreenRect rect = RectFor(view, PoseAt(0.0f, 0.0f, -1.0f), 0.5f, 0.5f);
    const float centreX = (rect.minX + rect.maxX) * 0.5f;
    // tanLeft = -1, tanRight = 3: the forward ray sits a quarter of the way in.
    CHECK(centreX == Approx(kImageWidth * 0.25f).margin(1e-3));
}

TEST_CASE("Degenerate quads and views are rejected before reaching the GPU")
{
    const FrameEyeView view = SymmetricView();

    CHECK_FALSE(ProjectQuad(view, PoseAt(0.0f, 0.0f, -1.0f), 0.0f, 1.0f).valid);
    CHECK_FALSE(ProjectQuad(view, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, -1.0f).valid);
    CHECK_FALSE(ProjectQuad(view, PoseAt(0.0f, 0.0f, -1.0f), 1.0f,
                            std::numeric_limits<float>::quiet_NaN())
                    .valid);

    FrameEyeView invalidView = view;
    invalidView.valid = false;
    CHECK_FALSE(ProjectQuad(invalidView, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f).valid);

    FrameEyeView zeroFov = view;
    zeroFov.fov.angleLeft = 0.0f;
    zeroFov.fov.angleRight = 0.0f;
    CHECK_FALSE(ProjectQuad(zeroFov, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f).valid);

    FramePose zeroQuaternion = PoseAt(0.0f, 0.0f, -1.0f);
    zeroQuaternion.orientation[3] = 0.0f;
    CHECK_FALSE(ProjectQuad(view, zeroQuaternion, 1.0f, 1.0f).valid);
}

TEST_CASE("The subImage rect becomes normalised UVs")
{
    FrameImageSource image = {};
    image.sourceX = 256;
    image.sourceY = 64;
    image.sourceWidth = 512;
    image.sourceHeight = 128;

    const QuadUvRect uv = MakeUvRect(image, 1024, 256);
    CHECK(uv.offsetU == Approx(0.25f));
    CHECK(uv.offsetV == Approx(0.25f));
    CHECK(uv.scaleU == Approx(0.5f));
    CHECK(uv.scaleV == Approx(0.5f));
}

TEST_CASE("A quad without a subImage rect samples the whole image")
{
    const QuadUvRect uv = MakeUvRect(FrameImageSource{}, 1024, 256);
    CHECK(uv.offsetU == Approx(0.0f));
    CHECK(uv.offsetV == Approx(0.0f));
    CHECK(uv.scaleU == Approx(1.0f));
    CHECK(uv.scaleV == Approx(1.0f));
}

TEST_CASE("Eye visibility selects which eyes a quad is drawn into")
{
    FrameQuadLayer quad = {};
    quad.eyeVisibility = FrameEyeVisibility::Both;
    CHECK(quad.IsVisibleForEye(true));
    CHECK(quad.IsVisibleForEye(false));

    quad.eyeVisibility = FrameEyeVisibility::Left;
    CHECK(quad.IsVisibleForEye(true));
    CHECK_FALSE(quad.IsVisibleForEye(false));

    quad.eyeVisibility = FrameEyeVisibility::Right;
    CHECK_FALSE(quad.IsVisibleForEye(true));
    CHECK(quad.IsVisibleForEye(false));
}

TEST_CASE("XrEyeVisibility maps onto the compositor's eye selection")
{
    using oxrsys::runtime::ToFrameEyeVisibility;
    CHECK(ToFrameEyeVisibility(XR_EYE_VISIBILITY_BOTH) == FrameEyeVisibility::Both);
    CHECK(ToFrameEyeVisibility(XR_EYE_VISIBILITY_LEFT) == FrameEyeVisibility::Left);
    CHECK(ToFrameEyeVisibility(XR_EYE_VISIBILITY_RIGHT) == FrameEyeVisibility::Right);
}

TEST_CASE("Composition layer flags select the blend mode")
{
    using oxrsys::runtime::ToFrameQuadBlend;

    CHECK(ToFrameQuadBlend(0) == FrameQuadBlend::Opaque);
    CHECK(ToFrameQuadBlend(XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT) ==
          FrameQuadBlend::PremultipliedAlpha);
    CHECK(ToFrameQuadBlend(XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                           XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT) ==
          FrameQuadBlend::UnpremultipliedAlpha);
    // UNPREMULTIPLIED on its own is meaningless: without the source-alpha bit
    // the spec says alpha is ignored outright.
    CHECK(ToFrameQuadBlend(XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT) ==
          FrameQuadBlend::Opaque);
    // An unrelated flag must not turn blending on.
    CHECK(ToFrameQuadBlend(XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT) ==
          FrameQuadBlend::Opaque);
}

TEST_CASE("FrameSource reports quad work only with both eye views")
{
    FrameSource frameSource = {};
    CHECK_FALSE(frameSource.HasQuadLayers());

    FrameQuadLayer quad = {};
    quad.widthMeters = 1.0f;
    quad.heightMeters = 1.0f;
    frameSource.quads.push_back(quad);
    // No projection layer was submitted, so there is nothing to project against.
    CHECK_FALSE(frameSource.HasQuadLayers());

    frameSource.views[0].valid = true;
    CHECK_FALSE(frameSource.HasQuadLayers());
    frameSource.views[1].valid = true;
    CHECK(frameSource.HasQuadLayers());

    frameSource.Reset();
    CHECK_FALSE(frameSource.HasQuadLayers());
    CHECK(frameSource.quads.empty());
    CHECK_FALSE(frameSource.views[0].valid);
}

TEST_CASE("Composing poses applies the parent's rotation to the child's offset")
{
    FramePose parent = PoseAt(1.0f, 0.0f, 0.0f);
    // 90 degrees of yaw about +Y.
    const float halfAngle = kPi / 4.0f;
    parent.orientation[1] = std::sin(halfAngle);
    parent.orientation[3] = std::cos(halfAngle);

    const FramePose child = PoseAt(0.0f, 0.0f, -2.0f);
    const FramePose composed = ComposePose(parent, child);

    // A +Y yaw of 90 degrees maps -Z onto -X in OpenXR's right-handed frame.
    CHECK(composed.position[0] == Approx(-1.0f).margin(1e-5));
    CHECK(composed.position[1] == Approx(0.0f).margin(1e-5));
    CHECK(composed.position[2] == Approx(0.0f).margin(1e-5));
}

TEST_CASE("Composing with identity is a no-op")
{
    const FramePose identity = PoseAt(0.0f, 0.0f, 0.0f);
    const FramePose pose = PoseAt(1.0f, 2.0f, 3.0f);

    const FramePose left = ComposePose(identity, pose);
    CHECK(left.position[0] == Approx(1.0f));
    CHECK(left.position[1] == Approx(2.0f));
    CHECK(left.position[2] == Approx(3.0f));

    const FramePose right = ComposePose(pose, identity);
    CHECK(right.position[0] == Approx(1.0f));
    CHECK(right.position[2] == Approx(3.0f));
}

TEST_CASE("The render-time VIEW space sits midway between the submitted eyes")
{
    FrameEyeView views[2] = {SymmetricView(), SymmetricView()};
    // A 64mm IPD straddling the origin, and the head pushed forward a metre.
    views[0].pose.position[0] = -0.032f;
    views[1].pose.position[0] = 0.032f;
    views[0].pose.position[2] = -1.0f;
    views[1].pose.position[2] = -1.0f;

    FramePose viewPose = {};
    REQUIRE(MakeRenderViewPose(views, viewPose));
    CHECK(viewPose.position[0] == Approx(0.0f).margin(1e-6));
    CHECK(viewPose.position[2] == Approx(-1.0f).margin(1e-6));
    CHECK(viewPose.orientation[3] == Approx(1.0f));
}

TEST_CASE("The render-time VIEW space needs both eyes")
{
    FrameEyeView views[2] = {SymmetricView(), SymmetricView()};
    FramePose viewPose = {};

    views[1].valid = false;
    CHECK_FALSE(MakeRenderViewPose(views, viewPose));

    views[1].valid = true;
    views[0].pose.position[0] = std::numeric_limits<float>::infinity();
    CHECK_FALSE(MakeRenderViewPose(views, viewPose));
}

TEST_CASE("A head-locked quad projects to the same place whatever the head did")
{
    // The point of pinning head-locked quads to the submitted views: a quad
    // 1m in front of VIEW space lands dead centre of the eye image whether the
    // head is at the origin or halfway across the room.
    const FramePose quadInViewSpace = PoseAt(0.0f, 0.0f, -1.0f);

    auto centreFor = [&](float headX, float headZ)
    {
        FrameEyeView views[2] = {SymmetricView(), SymmetricView()};
        for (FrameEyeView& view : views)
        {
            view.pose.position[0] = headX;
            view.pose.position[2] = headZ;
        }

        FramePose renderViewPose = {};
        REQUIRE(MakeRenderViewPose(views, renderViewPose));
        const FramePose quadPose = ComposePose(renderViewPose, quadInViewSpace);

        const ScreenRect rect = RectFor(views[0], quadPose, 1.0f, 1.0f);
        return std::make_pair((rect.minX + rect.maxX) * 0.5f, (rect.minY + rect.maxY) * 0.5f);
    };

    const auto atOrigin = centreFor(0.0f, 0.0f);
    const auto movedAway = centreFor(3.0f, -7.0f);

    CHECK(atOrigin.first == Approx(kImageWidth * 0.5f).margin(1e-3));
    CHECK(movedAway.first == Approx(atOrigin.first).margin(1e-3));
    CHECK(movedAway.second == Approx(atOrigin.second).margin(1e-3));
}
