// SPDX-License-Identifier: MPL-2.0
//
// The imageRect crop is ultimately a GPU operation, so these tests run the two
// primitives the encoder uses — a blit that takes its source origin from the
// rect, and an MPSImageBilinearScale driven by MakeFrameImageScaleTransform —
// against real textures and check which pixels came out.

#import <Metal/Metal.h>
#import <MetalPerformanceShaders/MetalPerformanceShaders.h>

#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "GraphicsTypes.h"

namespace
{

constexpr uint32_t kSourceWidth = 8;   // a "double-wide" side-by-side image
constexpr uint32_t kSourceHeight = 4;
constexpr uint32_t kEyeWidth = kSourceWidth / 2;

struct Rgba
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t a = 0;

    bool operator==(const Rgba& other) const
    {
        return r == other.r && g == other.g && b == other.b && a == other.a;
    }
};

id<MTLTexture> MakeTexture(id<MTLDevice> device, uint32_t width, uint32_t height)
{
    MTLTextureDescriptor* desc =
        [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                           width:width
                                                          height:height
                                                       mipmapped:NO];
    desc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite | MTLTextureUsageRenderTarget;
    desc.storageMode = MTLStorageModeManaged;
    return [device newTextureWithDescriptor:desc];
}

// Left half red, right half green, so a crop that reads the wrong half is
// unmistakable rather than a subtle offset.
id<MTLTexture> MakeSideBySideSource(id<MTLDevice> device)
{
    id<MTLTexture> texture = MakeTexture(device, kSourceWidth, kSourceHeight);
    if (texture == nil)
    {
        return nil;
    }

    std::vector<Rgba> pixels(kSourceWidth * kSourceHeight);
    for (uint32_t y = 0; y < kSourceHeight; y++)
    {
        for (uint32_t x = 0; x < kSourceWidth; x++)
        {
            Rgba& pixel = pixels[y * kSourceWidth + x];
            pixel = (x < kEyeWidth) ? Rgba{255, 0, 0, 255} : Rgba{0, 255, 0, 255};
        }
    }

    [texture replaceRegion:MTLRegionMake2D(0, 0, kSourceWidth, kSourceHeight)
               mipmapLevel:0
                 withBytes:pixels.data()
               bytesPerRow:kSourceWidth * sizeof(Rgba)];
    return texture;
}

std::vector<Rgba> ReadBack(id<MTLCommandQueue> queue, id<MTLTexture> texture)
{
    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    [blit synchronizeResource:texture];
    [blit endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    const uint32_t width = (uint32_t)texture.width;
    const uint32_t height = (uint32_t)texture.height;
    std::vector<Rgba> pixels(width * height);
    [texture getBytes:pixels.data()
          bytesPerRow:width * sizeof(Rgba)
           fromRegion:MTLRegionMake2D(0, 0, width, height)
          mipmapLevel:0];
    return pixels;
}

bool AllPixelsAre(const std::vector<Rgba>& pixels, Rgba expected)
{
    for (const Rgba& pixel : pixels)
    {
        if (!(pixel == expected))
        {
            return false;
        }
    }
    return !pixels.empty();
}

} // namespace

TEST_CASE("A blit from the rect's origin copies only that sub-region")
{
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (device == nil)
    {
        SUCCEED("No Metal device available");
        return;
    }
    id<MTLCommandQueue> queue = [device newCommandQueue];
    id<MTLTexture> source = MakeSideBySideSource(device);
    REQUIRE(source != nil);

    // The right eye of a side-by-side submission: offset x = half the image.
    FrameImageRect rect = {};
    rect.offsetX = (int32_t)kEyeWidth;
    rect.offsetY = 0;
    rect.width = (int32_t)kEyeWidth;
    rect.height = (int32_t)kSourceHeight;
    const FrameImageRect resolved = ResolveFrameImageRect(rect, kSourceWidth, kSourceHeight);

    id<MTLTexture> destination = MakeTexture(device, kEyeWidth, kSourceHeight);
    REQUIRE(destination != nil);

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    [blit copyFromTexture:source
              sourceSlice:0
              sourceLevel:0
             sourceOrigin:MTLOriginMake(resolved.offsetX, resolved.offsetY, 0)
               sourceSize:MTLSizeMake(resolved.width, resolved.height, 1)
                toTexture:destination
         destinationSlice:0
         destinationLevel:0
        destinationOrigin:MTLOriginMake(0, 0, 0)];
    [blit endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    // Green is the right half. Red here would mean the rect was ignored.
    CHECK(AllPixelsAre(ReadBack(queue, destination), Rgba{0, 255, 0, 255}));
}

TEST_CASE("A full-image rect blits the whole image as before")
{
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (device == nil)
    {
        SUCCEED("No Metal device available");
        return;
    }
    id<MTLCommandQueue> queue = [device newCommandQueue];
    id<MTLTexture> source = MakeSideBySideSource(device);
    REQUIRE(source != nil);

    const FrameImageRect resolved = ResolveFrameImageRect({}, kSourceWidth, kSourceHeight);
    REQUIRE(resolved.offsetX == 0);
    REQUIRE(resolved.width == (int32_t)kSourceWidth);

    id<MTLTexture> destination = MakeTexture(device, kSourceWidth, kSourceHeight);
    REQUIRE(destination != nil);

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    [blit copyFromTexture:source
              sourceSlice:0
              sourceLevel:0
             sourceOrigin:MTLOriginMake(resolved.offsetX, resolved.offsetY, 0)
               sourceSize:MTLSizeMake(resolved.width, resolved.height, 1)
                toTexture:destination
         destinationSlice:0
         destinationLevel:0
        destinationOrigin:MTLOriginMake(0, 0, 0)];
    [blit endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    const std::vector<Rgba> pixels = ReadBack(queue, destination);
    REQUIRE(pixels.size() == kSourceWidth * kSourceHeight);
    for (uint32_t y = 0; y < kSourceHeight; y++)
    {
        for (uint32_t x = 0; x < kSourceWidth; x++)
        {
            const Rgba expected = (x < kEyeWidth) ? Rgba{255, 0, 0, 255} : Rgba{0, 255, 0, 255};
            CHECK(pixels[y * kSourceWidth + x] == expected);
        }
    }
}

TEST_CASE("MakeFrameImageScaleTransform drives MPS to crop and downscale")
{
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (device == nil)
    {
        SUCCEED("No Metal device available");
        return;
    }
    id<MTLCommandQueue> queue = [device newCommandQueue];
    id<MTLTexture> source = MakeSideBySideSource(device);
    REQUIRE(source != nil);

    FrameImageRect rect = {};
    rect.offsetX = (int32_t)kEyeWidth;
    rect.offsetY = 0;
    rect.width = (int32_t)kEyeWidth;
    rect.height = (int32_t)kSourceHeight;
    const FrameImageRect resolved = ResolveFrameImageRect(rect, kSourceWidth, kSourceHeight);

    // Half the rect's size, so this exercises crop and scale together.
    const uint32_t destWidth = kEyeWidth / 2;
    const uint32_t destHeight = kSourceHeight / 2;
    id<MTLTexture> destination = MakeTexture(device, destWidth, destHeight);
    REQUIRE(destination != nil);

    const FrameImageScaleTransform mapping =
        MakeFrameImageScaleTransform(resolved, destWidth, destHeight);
    MPSScaleTransform transform = {};
    transform.scaleX = mapping.scaleX;
    transform.scaleY = mapping.scaleY;
    transform.translateX = mapping.translateX;
    transform.translateY = mapping.translateY;

    MPSImageBilinearScale* scaler = [[MPSImageBilinearScale alloc] initWithDevice:device];
    REQUIRE(scaler != nil);
    scaler.scaleTransform = &transform;
    scaler.clipRect = MTLRegionMake2D(0, 0, destWidth, destHeight);

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    [scaler encodeToCommandBuffer:cmd sourceTexture:source destinationTexture:destination];
    [cmd commit];
    [cmd waitUntilCompleted];

    // Only the green half was asked for. If the translate had the wrong sign we
    // would be looking at red, and with no transform at all at a red/green mix.
    CHECK(AllPixelsAre(ReadBack(queue, destination), Rgba{0, 255, 0, 255}));

    [scaler release];
}

TEST_CASE("MPS with no scale transform still downscales the whole image")
{
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (device == nil)
    {
        SUCCEED("No Metal device available");
        return;
    }
    id<MTLCommandQueue> queue = [device newCommandQueue];
    id<MTLTexture> source = MakeSideBySideSource(device);
    REQUIRE(source != nil);

    id<MTLTexture> destination = MakeTexture(device, kSourceWidth / 2, kSourceHeight / 2);
    REQUIRE(destination != nil);

    MPSImageBilinearScale* scaler = [[MPSImageBilinearScale alloc] initWithDevice:device];
    REQUIRE(scaler != nil);

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    [scaler encodeToCommandBuffer:cmd sourceTexture:source destinationTexture:destination];
    [cmd commit];
    [cmd waitUntilCompleted];

    // The whole 8-wide image squeezed into 4: still red on the left, green on
    // the right, which is the pre-imageRect behaviour a full-image rect keeps.
    const std::vector<Rgba> pixels = ReadBack(queue, destination);
    REQUIRE(pixels.size() == (kSourceWidth / 2) * (kSourceHeight / 2));
    CHECK(pixels[0].r > pixels[0].g);
    CHECK(pixels[(kSourceWidth / 2) - 1].g > pixels[(kSourceWidth / 2) - 1].r);

    [scaler release];
}
