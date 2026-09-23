// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "GraphicsTypes.h"
#include "SwapchainRect.h"

namespace
{

XrSwapchainSubImage MakeSubImage(int32_t x, int32_t y, int32_t width, int32_t height,
                                 uint32_t arrayIndex = 0)
{
    XrSwapchainSubImage subImage = {};
    subImage.swapchain = XR_NULL_HANDLE;
    subImage.imageRect.offset.x = x;
    subImage.imageRect.offset.y = y;
    subImage.imageRect.extent.width = width;
    subImage.imageRect.extent.height = height;
    subImage.imageArrayIndex = arrayIndex;
    return subImage;
}

} // namespace

TEST_CASE("FrameImageRect carries XrSwapchainSubImage::imageRect verbatim")
{
    const FrameImageRect rect = FrameImageRectFromSubImage(MakeSubImage(1512, 0, 1512, 1680));

    CHECK(rect.offsetX == 1512);
    CHECK(rect.offsetY == 0);
    CHECK(rect.width == 1512);
    CHECK(rect.height == 1680);
    CHECK_FALSE(rect.IsEmpty());
}

TEST_CASE("FrameImageRect conversion is independent of imageArrayIndex")
{
    // imageArrayIndex selects the texture slice; the rect must not change with it.
    const FrameImageRect slice0 = FrameImageRectFromSubImage(MakeSubImage(10, 20, 30, 40, 0));
    const FrameImageRect slice1 = FrameImageRectFromSubImage(MakeSubImage(10, 20, 30, 40, 1));

    CHECK(slice0.offsetX == slice1.offsetX);
    CHECK(slice0.offsetY == slice1.offsetY);
    CHECK(slice0.width == slice1.width);
    CHECK(slice0.height == slice1.height);
}

TEST_CASE("A default FrameImageSource still covers the whole image")
{
    FrameImageSource source = {};

    CHECK(source.Rect().IsEmpty());
    CHECK(source.Rect().CoversFullImage(1512, 1680));

    const FrameImageRect resolved = source.GetRect(1512, 1680);
    CHECK(resolved.offsetX == 0);
    CHECK(resolved.offsetY == 0);
    CHECK(resolved.width == 1512);
    CHECK(resolved.height == 1680);
}

TEST_CASE("A full-image rect resolves to the full image")
{
    FrameImageSource source = {};
    source.SetRect(FrameImageRectFromSubImage(MakeSubImage(0, 0, 1512, 1680)));

    CHECK(source.Rect().CoversFullImage(1512, 1680));

    const FrameImageRect resolved = source.GetRect(1512, 1680);
    CHECK(resolved.offsetX == 0);
    CHECK(resolved.offsetY == 0);
    CHECK(resolved.width == 1512);
    CHECK(resolved.height == 1680);
}

TEST_CASE("An offset rect resolves to the requested sub-region")
{
    // The BasaultVR (UE4) case: one 3024x1680 side-by-side swapchain image
    // submitted twice, the left eye taking uMin 0..0.5 and the right 0.5..1.0.
    FrameImageSource left = {};
    left.SetRect(FrameImageRectFromSubImage(MakeSubImage(0, 0, 1512, 1680)));
    FrameImageSource right = {};
    right.SetRect(FrameImageRectFromSubImage(MakeSubImage(1512, 0, 1512, 1680)));

    CHECK_FALSE(left.Rect().CoversFullImage(3024, 1680));
    CHECK_FALSE(right.Rect().CoversFullImage(3024, 1680));

    const FrameImageRect leftResolved = left.GetRect(3024, 1680);
    CHECK(leftResolved.offsetX == 0);
    CHECK(leftResolved.offsetY == 0);
    CHECK(leftResolved.width == 1512);
    CHECK(leftResolved.height == 1680);

    const FrameImageRect rightResolved = right.GetRect(3024, 1680);
    CHECK(rightResolved.offsetX == 1512);
    CHECK(rightResolved.offsetY == 0);
    CHECK(rightResolved.width == 1512);
    CHECK(rightResolved.height == 1680);
}

TEST_CASE("Resolving clips a rect that reaches outside the image")
{
    FrameImageRect rect = {};
    rect.offsetX = 100;
    rect.offsetY = 100;
    rect.width = 400;
    rect.height = 400;

    const FrameImageRect resolved = ResolveFrameImageRect(rect, 256, 256);
    CHECK(resolved.offsetX == 100);
    CHECK(resolved.offsetY == 100);
    CHECK(resolved.width == 156);
    CHECK(resolved.height == 156);
}

TEST_CASE("Resolving falls back to the full image for degenerate input")
{
    FrameImageRect empty = {};
    const FrameImageRect fromEmpty = ResolveFrameImageRect(empty, 64, 32);
    CHECK(fromEmpty.offsetX == 0);
    CHECK(fromEmpty.offsetY == 0);
    CHECK(fromEmpty.width == 64);
    CHECK(fromEmpty.height == 32);

    FrameImageRect negative = {};
    negative.offsetX = -10;
    negative.offsetY = -10;
    negative.width = 16;
    negative.height = 16;
    const FrameImageRect fromNegative = ResolveFrameImageRect(negative, 64, 32);
    CHECK(fromNegative.offsetX == 0);
    CHECK(fromNegative.offsetY == 0);

    FrameImageRect offImage = {};
    offImage.offsetX = 1000;
    offImage.offsetY = 0;
    offImage.width = 16;
    offImage.height = 16;
    const FrameImageRect fromOffImage = ResolveFrameImageRect(offImage, 64, 32);
    CHECK(fromOffImage.width == 64);
    CHECK(fromOffImage.height == 32);

    // A zero-sized image has no usable region; resolving must not divide by it.
    const FrameImageRect fromZeroImage = ResolveFrameImageRect(negative, 0, 0);
    CHECK(fromZeroImage.width == 0);
    CHECK(fromZeroImage.height == 0);
}

TEST_CASE("Resetting a FrameImageSource clears the rect")
{
    FrameImageSource source = {};
    source.SetRect(FrameImageRectFromSubImage(MakeSubImage(1512, 0, 1512, 1680)));
    source.Reset();

    CHECK(source.Rect().IsEmpty());
    CHECK(source.GetRect(3024, 1680).width == 3024);
}

TEST_CASE("The uv transform is identity for a full-image rect")
{
    const FrameImageRect resolved = ResolveFrameImageRect({}, 1512, 1680);
    const FrameImageUvTransform transform = MakeFrameImageUvTransform(resolved, 1512, 1680);

    CHECK(transform.scaleX == Catch::Approx(1.0f));
    CHECK(transform.scaleY == Catch::Approx(1.0f));
    CHECK(transform.offsetX == Catch::Approx(0.0f));
    CHECK(transform.offsetY == Catch::Approx(0.0f));
}

TEST_CASE("The uv transform maps a cropped rect onto the unit square")
{
    FrameImageRect rect = {};
    rect.offsetX = 1512;
    rect.offsetY = 0;
    rect.width = 1512;
    rect.height = 1680;

    const FrameImageRect resolved = ResolveFrameImageRect(rect, 3024, 1680);
    const FrameImageUvTransform transform = MakeFrameImageUvTransform(resolved, 3024, 1680);

    CHECK(transform.scaleX == Catch::Approx(0.5f));
    CHECK(transform.scaleY == Catch::Approx(1.0f));
    CHECK(transform.offsetX == Catch::Approx(0.5f));
    CHECK(transform.offsetY == Catch::Approx(0.0f));

    // uv 0 and 1 land on the two edges of the right half of the image.
    CHECK(transform.offsetX + 0.0f * transform.scaleX == Catch::Approx(0.5f));
    CHECK(transform.offsetX + 1.0f * transform.scaleX == Catch::Approx(1.0f));
}

TEST_CASE("The scale transform maps a full-image rect to a plain downscale")
{
    const FrameImageRect resolved = ResolveFrameImageRect({}, 2048, 2048);
    const FrameImageScaleTransform mapping = MakeFrameImageScaleTransform(resolved, 1024, 1024);

    CHECK(mapping.scaleX == Catch::Approx(0.5));
    CHECK(mapping.scaleY == Catch::Approx(0.5));
    CHECK(mapping.translateX == Catch::Approx(0.0));
    CHECK(mapping.translateY == Catch::Approx(0.0));
}

TEST_CASE("The scale transform maps a cropped rect onto the destination")
{
    FrameImageRect rect = {};
    rect.offsetX = 1512;
    rect.offsetY = 0;
    rect.width = 1512;
    rect.height = 1680;

    const FrameImageRect resolved = ResolveFrameImageRect(rect, 3024, 1680);
    const FrameImageScaleTransform mapping = MakeFrameImageScaleTransform(resolved, 756, 840);

    CHECK(mapping.scaleX == Catch::Approx(0.5));
    CHECK(mapping.scaleY == Catch::Approx(0.5));
    CHECK(mapping.translateX == Catch::Approx(-756.0));
    CHECK(mapping.translateY == Catch::Approx(0.0));

    // dst = src * scale + translate: the rect's two edges land on the
    // destination's two edges, so only the requested half is encoded.
    const double leftEdge = resolved.offsetX * mapping.scaleX + mapping.translateX;
    const double rightEdge = (resolved.offsetX + resolved.width) * mapping.scaleX + mapping.translateX;
    CHECK(leftEdge == Catch::Approx(0.0));
    CHECK(rightEdge == Catch::Approx(756.0));
}

TEST_CASE("A cropped rect with no scaling maps 1:1 from its offset")
{
    FrameImageRect rect = {};
    rect.offsetX = 1512;
    rect.offsetY = 0;
    rect.width = 1512;
    rect.height = 1680;

    const FrameImageRect resolved = ResolveFrameImageRect(rect, 3024, 1680);
    const FrameImageScaleTransform mapping = MakeFrameImageScaleTransform(resolved, 1512, 1680);

    CHECK(mapping.scaleX == Catch::Approx(1.0));
    CHECK(mapping.scaleY == Catch::Approx(1.0));
    CHECK(mapping.translateX == Catch::Approx(-1512.0));
    CHECK(mapping.translateY == Catch::Approx(0.0));
}
