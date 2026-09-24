// SPDX-License-Identifier: MPL-2.0
//
// EyeFovReprojector against real Metal textures: an eye image whose columns encode
// their own index is reprojected onto a different fov, read back, and every column is
// checked against the source column EyeFovRemap.h says it must show (or black).

#import <Metal/Metal.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "EyeFovReprojector.h"

using namespace oxrsys::video;

namespace
{

constexpr uint32_t kWidth = 200;
constexpr uint32_t kHeight = 120;

id<MTLTexture> MakeTexture(id<MTLDevice> device, MTLStorageMode storage)
{
    MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                 width:kWidth
                                                                                height:kHeight
                                                                             mipmapped:NO];
    d.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite | MTLTextureUsageRenderTarget;
    d.storageMode = storage;
    return [device newTextureWithDescriptor:d];
}

FrameFov Fov(float left, float right, float up, float down)
{
    FrameFov fov;
    fov.angleLeft = left;
    fov.angleRight = right;
    fov.angleUp = up;
    fov.angleDown = down;
    return fov;
}

} // namespace

TEST_CASE("EyeFovReprojector redraws an eye onto the display fov", "[metal][fov]")
{
    @autoreleasepool
    {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil)
        {
            SKIP("no Metal device");
        }
        id<MTLCommandQueue> queue = [device newCommandQueue];

        // Red = column index, green = row index, blue = 255 (never black).
        id<MTLTexture> source = MakeTexture(device, MTLStorageModeManaged);
        std::vector<uint8_t> pixels(kWidth * kHeight * 4);
        for (uint32_t y = 0; y < kHeight; ++y)
        {
            for (uint32_t x = 0; x < kWidth; ++x)
            {
                uint8_t* p = &pixels[(y * kWidth + x) * 4];
                p[0] = static_cast<uint8_t>(x);
                p[1] = static_cast<uint8_t>(y);
                p[2] = 255;
                p[3] = 255;
            }
        }
        [source replaceRegion:MTLRegionMake2D(0, 0, kWidth, kHeight) mipmapLevel:0 withBytes:pixels.data()
                  bytesPerRow:kWidth * 4];

        const FrameFov rendered = Fov(-0.8204f, 0.8204f, 0.8727f, -0.8727f);
        const FrameFov display = Fov(-0.9076f, 0.7330f, 0.8378f, -0.8727f);
        const FovRemap remap = ComputeFovRemap(rendered, display);

        EyeFovReprojector reprojector;
        void* cached = nullptr;
        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        id<MTLTexture> result = (__bridge id<MTLTexture>)reprojector.Reproject(
            (__bridge void*)cmd, (__bridge void*)source, remap, &cached);
        REQUIRE(result != nil);
        CHECK(result.width == kWidth);
        CHECK(result.height == kHeight);

        id<MTLTexture> readback = MakeTexture(device, MTLStorageModeManaged);
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit copyFromTexture:result toTexture:readback];
        [blit synchronizeResource:readback];
        [blit endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];

        std::vector<uint8_t> out(kWidth * kHeight * 4);
        [readback getBytes:out.data() bytesPerRow:kWidth * 4 fromRegion:MTLRegionMake2D(0, 0, kWidth, kHeight)
               mipmapLevel:0];

        const uint32_t row = kHeight / 2;
        int blackColumns = 0;
        for (uint32_t x = 0; x < kWidth; ++x)
        {
            const float u = (x + 0.5f) / kWidth;
            const float sourceU = u * remap.uScale + remap.uOffset;
            const uint8_t* p = &out[(row * kWidth + x) * 4];
            if (sourceU < -0.01f)
            {
                CHECK(p[0] == 0);
                CHECK(p[2] == 0);
                ++blackColumns;
            }
            else if (sourceU > 0.01f && sourceU < 0.99f)
            {
                // Linear filtering between neighbouring columns: within one column.
                const float expected = sourceU * kWidth - 0.5f;
                CHECK(std::fabs(p[0] - expected) <= 1.0f);
                CHECK(p[2] == 255);
            }
        }
        // tan(0.9076) is beyond tan(0.8204): the outer ~9% of the display was never rendered.
        CHECK(blackColumns >= 17);
        CHECK(blackColumns <= 20);

        [(id<MTLTexture>)cached release];
        [readback release];
        [source release];
        [queue release];
    }
}
