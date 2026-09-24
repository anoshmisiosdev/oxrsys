// SPDX-License-Identifier: MPL-2.0
//
// The GPU half of quad-layer compositing, run against real Metal textures.
//
// TestQuadLayerProjection.cpp proves the matrices; this proves the pixels. A
// known-colour quad is composited over a known-colour eye image and the result
// is read back and checked texel by texel -- where the quad landed, which eye
// it landed in, and what each of the two OpenXR alpha flags did to it.
//
// Every expectation here is derived from the projection maths independently
// (a 1m quad 1m away in a 90-degree frustum covers the middle half of the
// image), so a shader that draws the quad in the wrong place fails rather than
// agreeing with a bug in the maths.

#import <Metal/Metal.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "GraphicsTypes.h"
#include "QuadLayerRenderer.h"

using namespace oxrsys::quad;

namespace
{

constexpr uint32_t kEyeSize = 64;
constexpr float kPi = 3.14159265358979323846f;

// The eye image's background, and the quad's colour. Chosen far apart so a
// misplaced quad cannot be mistaken for a filtering artefact.
constexpr uint8_t kBackgroundBlue = 255;

struct Rgba
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t a = 0;
};

id<MTLTexture> MakeTexture(id<MTLDevice> device, uint32_t width, uint32_t height,
                           MTLStorageMode storageMode = MTLStorageModeManaged)
{
    MTLTextureDescriptor* descriptor =
        [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                           width:width
                                                          height:height
                                                       mipmapped:NO];
    descriptor.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite |
                       MTLTextureUsageRenderTarget;
    descriptor.storageMode = storageMode;
    return [device newTextureWithDescriptor:descriptor];
}

id<MTLTexture> MakeSolidTexture(id<MTLDevice> device, uint32_t width, uint32_t height, Rgba colour)
{
    id<MTLTexture> texture = MakeTexture(device, width, height);
    if (texture == nil)
    {
        return nil;
    }
    std::vector<Rgba> pixels(static_cast<size_t>(width) * height, colour);
    [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
               mipmapLevel:0
                 withBytes:pixels.data()
               bytesPerRow:width * sizeof(Rgba)];
    return texture;
}

// Left half `left`, right half `right` -- for the subImage rect test, where
// sampling the wrong half is unmistakable.
id<MTLTexture> MakeSplitTexture(id<MTLDevice> device, uint32_t width, uint32_t height, Rgba left,
                                Rgba right)
{
    id<MTLTexture> texture = MakeTexture(device, width, height);
    if (texture == nil)
    {
        return nil;
    }
    std::vector<Rgba> pixels(static_cast<size_t>(width) * height);
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            pixels[y * width + x] = (x < width / 2) ? left : right;
        }
    }
    [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
               mipmapLevel:0
                 withBytes:pixels.data()
               bytesPerRow:width * sizeof(Rgba)];
    return texture;
}

// Four distinct quadrants, so that a flipped or transposed UV mapping shows up
// as the wrong colour rather than as no change at all. Row 0 is the texture's
// *top*, which OpenXR maps to the quad's upper (+Y) edge.
id<MTLTexture> MakeQuadrantTexture(id<MTLDevice> device, uint32_t size, Rgba topLeft,
                                   Rgba topRight, Rgba bottomLeft, Rgba bottomRight)
{
    id<MTLTexture> texture = MakeTexture(device, size, size);
    if (texture == nil)
    {
        return nil;
    }
    std::vector<Rgba> pixels(static_cast<size_t>(size) * size);
    for (uint32_t y = 0; y < size; ++y)
    {
        for (uint32_t x = 0; x < size; ++x)
        {
            const bool top = y < size / 2;
            const bool left = x < size / 2;
            pixels[y * size + x] = top ? (left ? topLeft : topRight)
                                       : (left ? bottomLeft : bottomRight);
        }
    }
    [texture replaceRegion:MTLRegionMake2D(0, 0, size, size)
               mipmapLevel:0
                 withBytes:pixels.data()
               bytesPerRow:size * sizeof(Rgba)];
    return texture;
}

// ComposeEye allocates its destination private, so stage it through a managed
// copy to get the bytes back on every GPU.
std::vector<Rgba> ReadBack(id<MTLDevice> device, id<MTLCommandQueue> queue, id<MTLTexture> texture)
{
    const uint32_t width = static_cast<uint32_t>(texture.width);
    const uint32_t height = static_cast<uint32_t>(texture.height);
    id<MTLTexture> staging = MakeTexture(device, width, height);

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    [blit copyFromTexture:texture
              sourceSlice:0
              sourceLevel:0
             sourceOrigin:MTLOriginMake(0, 0, 0)
               sourceSize:MTLSizeMake(width, height, 1)
                toTexture:staging
         destinationSlice:0
         destinationLevel:0
        destinationOrigin:MTLOriginMake(0, 0, 0)];
    [blit synchronizeResource:staging];
    [blit endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    std::vector<Rgba> pixels(static_cast<size_t>(width) * height);
    [staging getBytes:pixels.data()
          bytesPerRow:width * sizeof(Rgba)
           fromRegion:MTLRegionMake2D(0, 0, width, height)
          mipmapLevel:0];
    [staging release];
    return pixels;
}

const Rgba& At(const std::vector<Rgba>& pixels, uint32_t x, uint32_t y)
{
    return pixels[static_cast<size_t>(y) * kEyeSize + x];
}

bool NearlyEqual(uint8_t actual, int expected, int tolerance = 2)
{
    return std::abs(static_cast<int>(actual) - expected) <= tolerance;
}

// A viewer at the origin looking down -Z with a symmetric 90-degree frustum, so
// a 1m quad 1m away covers exactly the middle half of the eye image.
FrameEyeView SymmetricView()
{
    FrameEyeView view = {};
    view.pose.orientation[3] = 1.0f;
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

FrameSource MakeFrameSource()
{
    FrameSource frameSource = {};
    frameSource.views[0] = SymmetricView();
    frameSource.views[1] = SymmetricView();
    return frameSource;
}

// Wraps an already-created texture as a quad layer's image without taking
// ownership; the test keeps the texture alive for the call's duration.
FrameQuadLayer MakeQuad(id<MTLTexture> texture, FramePose pose, float width, float height)
{
    FrameQuadLayer quad = {};
    quad.image.api = GraphicsApi::Metal;
    quad.image.image = std::shared_ptr<void>((__bridge void*)texture, [](void*) {});
    quad.pose = pose;
    quad.widthMeters = width;
    quad.heightMeters = height;
    return quad;
}

struct MetalFixture
{
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;

    bool Available() const { return device != nil && queue != nil; }
};

MetalFixture MakeFixture()
{
    MetalFixture fixture = {};
    fixture.device = MTLCreateSystemDefaultDevice();
    if (fixture.device != nil)
    {
        fixture.queue = [fixture.device newCommandQueue];
    }
    return fixture;
}

// Runs one ComposeEye and reads back the result. `cached` persists the
// renderer's destination texture the way the encoder's per-slot cache does.
std::vector<Rgba> Compose(const MetalFixture& fixture, QuadLayerRenderer& renderer,
                          id<MTLTexture> background, const FrameSource& frameSource, bool leftEye,
                          void** cached, id<MTLTexture>* outComposed = nullptr)
{
    id<MTLCommandBuffer> cmd = [fixture.queue commandBuffer];
    void* composed = renderer.ComposeEye((__bridge void*)cmd, (__bridge void*)background,
                                         frameSource, leftEye, cached);
    [cmd commit];
    [cmd waitUntilCompleted];

    REQUIRE(composed != nullptr);
    id<MTLTexture> composedTexture = (__bridge id<MTLTexture>)composed;
    if (outComposed != nullptr)
    {
        *outComposed = composedTexture;
    }
    return ReadBack(fixture.device, fixture.queue, composedTexture);
}

void ReleaseCached(void* cached)
{
    if (cached != nullptr)
    {
        [(id<MTLTexture>)cached release];
    }
}

} // namespace

TEST_CASE("A quad composites where the projection maths puts it")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture =
        MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});
    REQUIRE(background != nil);
    REQUIRE(quadTexture != nil);

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(
        MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // A 1m quad at 1m in a 90-degree frustum spans NDC [-0.5, 0.5] on both
    // axes: pixels [16, 48) of a 64-pixel eye.
    CHECK(At(pixels, 32, 32).r == 255);
    CHECK(At(pixels, 32, 32).b == 0);
    CHECK(At(pixels, 17, 32).r == 255);
    CHECK(At(pixels, 46, 32).r == 255);
    CHECK(At(pixels, 32, 17).r == 255);
    CHECK(At(pixels, 32, 46).r == 255);

    // ... and nothing outside it.
    CHECK(At(pixels, 14, 32).b == kBackgroundBlue);
    CHECK(At(pixels, 14, 32).r == 0);
    CHECK(At(pixels, 49, 32).b == kBackgroundBlue);
    CHECK(At(pixels, 32, 14).b == kBackgroundBlue);
    CHECK(At(pixels, 32, 49).b == kBackgroundBlue);
    CHECK(At(pixels, 0, 0).b == kBackgroundBlue);
    CHECK(At(pixels, 63, 63).b == kBackgroundBlue);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("A quad above the viewer composites into the top of the eye image")
{
    // The pixel-level counterpart of the Y-flip unit test: raising the quad in
    // OpenXR's +Y must darken the *upper* rows of the Metal target.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});
    REQUIRE(background != nil);

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(MakeQuad(quadTexture, PoseAt(0.0f, 0.5f, -1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // NDC y in [0, 1] -> the top half: pixel rows [0, 32).
    CHECK(At(pixels, 32, 4).r == 255);
    CHECK(At(pixels, 32, 28).r == 255);
    CHECK(At(pixels, 32, 40).b == kBackgroundBlue);
    CHECK(At(pixels, 32, 60).b == kBackgroundBlue);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("eyeVisibility keeps a quad out of the eye it was not meant for")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});

    FrameSource frameSource = MakeFrameSource();
    FrameQuadLayer quad = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quad.eyeVisibility = FrameEyeVisibility::Left;
    frameSource.quads.push_back(quad);

    void* leftCached = nullptr;
    const std::vector<Rgba> leftPixels =
        Compose(fixture, renderer, background, frameSource, true, &leftCached);
    CHECK(At(leftPixels, 32, 32).r == 255);

    // The right eye has no work at all, so ComposeEye must hand back the
    // untouched background rather than a copy with the quad on it.
    void* rightCached = nullptr;
    CHECK_FALSE(QuadLayerRenderer::HasWorkForEye(frameSource, false));
    const std::vector<Rgba> rightPixels =
        Compose(fixture, renderer, background, frameSource, false, &rightCached);
    CHECK(At(rightPixels, 32, 32).b == kBackgroundBlue);
    CHECK(At(rightPixels, 32, 32).r == 0);

    ReleaseCached(leftCached);
    ReleaseCached(rightCached);
    [background release];
    [quadTexture release];
}

TEST_CASE("Premultiplied source alpha blends the quad over the eye image")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    // Half-transparent red, already multiplied by alpha: (128, 0, 0, 128).
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{128, 0, 0, 128});

    FrameSource frameSource = MakeFrameSource();
    FrameQuadLayer quad = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quad.blend = FrameQuadBlend::PremultipliedAlpha;
    frameSource.quads.push_back(quad);

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // src + dst * (1 - srcAlpha): red 128 + 0, blue 0 + 255 * (1 - 128/255).
    const Rgba& centre = At(pixels, 32, 32);
    CHECK(NearlyEqual(centre.r, 128));
    CHECK(NearlyEqual(centre.b, 127));
    // Outside the quad the background is untouched.
    CHECK(At(pixels, 4, 4).b == kBackgroundBlue);
    CHECK(At(pixels, 4, 4).r == 0);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("Unpremultiplied source alpha scales the quad's colour before blending")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    // The same colour as the premultiplied case, stored unpremultiplied:
    // full-strength red at half alpha.
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 128});

    FrameSource frameSource = MakeFrameSource();
    FrameQuadLayer quad = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quad.blend = FrameQuadBlend::UnpremultipliedAlpha;
    frameSource.quads.push_back(quad);

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // srcAlpha * src + (1 - srcAlpha) * dst: the same result as the
    // premultiplied case above, which is the whole point of the flag.
    const Rgba& centre = At(pixels, 32, 32);
    CHECK(NearlyEqual(centre.r, 128));
    CHECK(NearlyEqual(centre.b, 127));

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("The two alpha flags are not interchangeable")
{
    // If UNPREMULTIPLIED were ignored, this unpremultiplied texture would go
    // through the premultiplied pipeline and come out over-bright (255 red
    // instead of 128), so this is the test that fails if the flag is dropped.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 128});

    FrameSource premultiplied = MakeFrameSource();
    FrameQuadLayer quadA = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quadA.blend = FrameQuadBlend::PremultipliedAlpha;
    premultiplied.quads.push_back(quadA);

    void* cachedA = nullptr;
    const std::vector<Rgba> premultipliedPixels =
        Compose(fixture, renderer, background, premultiplied, true, &cachedA);
    CHECK(NearlyEqual(At(premultipliedPixels, 32, 32).r, 255));

    FrameSource unpremultiplied = MakeFrameSource();
    FrameQuadLayer quadB = quadA;
    quadB.blend = FrameQuadBlend::UnpremultipliedAlpha;
    unpremultiplied.quads.push_back(quadB);

    void* cachedB = nullptr;
    const std::vector<Rgba> unpremultipliedPixels =
        Compose(fixture, renderer, background, unpremultiplied, true, &cachedB);
    CHECK(NearlyEqual(At(unpremultipliedPixels, 32, 32).r, 128));

    ReleaseCached(cachedA);
    ReleaseCached(cachedB);
    [background release];
    [quadTexture release];
}

TEST_CASE("Without the source-alpha flag a transparent quad is drawn opaque")
{
    // XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT unset means the
    // texture's alpha channel is ignored, not that the quad disappears.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 0});

    FrameSource frameSource = MakeFrameSource();
    FrameQuadLayer quad = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quad.blend = FrameQuadBlend::Opaque;
    frameSource.quads.push_back(quad);

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    CHECK(At(pixels, 32, 32).r == 255);
    CHECK(At(pixels, 32, 32).b == 0);
    // The composited eye image must stay opaque for the encoder.
    CHECK(At(pixels, 32, 32).a == 255);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("Quads composite in submission order")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> firstTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});
    id<MTLTexture> secondTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{0, 255, 0, 255});

    FrameSource frameSource = MakeFrameSource();
    // A large red quad, then a smaller green one on top of it.
    frameSource.quads.push_back(MakeQuad(firstTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f));
    frameSource.quads.push_back(MakeQuad(secondTexture, PoseAt(0.0f, 0.0f, -1.0f), 0.5f, 0.5f));

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // Centre: the later, smaller quad wins.
    CHECK(At(pixels, 32, 32).g == 255);
    CHECK(At(pixels, 32, 32).r == 0);
    // Between the two quads' edges: the earlier one shows.
    CHECK(At(pixels, 20, 32).r == 255);
    CHECK(At(pixels, 20, 32).g == 0);

    ReleaseCached(cached);
    [background release];
    [firstTexture release];
    [secondTexture release];
}

TEST_CASE("A quad samples only its subImage rect")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    // Left half red, right half green; the rect selects the right half.
    id<MTLTexture> quadTexture =
        MakeSplitTexture(fixture.device, 32, 16, Rgba{255, 0, 0, 255}, Rgba{0, 255, 0, 255});

    FrameSource frameSource = MakeFrameSource();
    FrameQuadLayer quad = MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f);
    quad.image.sourceX = 16;
    quad.image.sourceY = 0;
    quad.image.sourceWidth = 16;
    quad.image.sourceHeight = 16;
    frameSource.quads.push_back(quad);

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // Whole quad green: it sampled only the right half. Sampling the whole
    // image would leave the quad's own left half red.
    CHECK(At(pixels, 32, 32).g == 255);
    CHECK(At(pixels, 20, 32).g == 255);
    CHECK(At(pixels, 20, 32).r == 0);
    CHECK(At(pixels, 44, 32).g == 255);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("A quad behind the viewer is not composited")
{
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, 1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    for (uint32_t y = 0; y < kEyeSize; y += 8)
    {
        for (uint32_t x = 0; x < kEyeSize; x += 8)
        {
            CHECK(At(pixels, x, y).b == kBackgroundBlue);
            CHECK(At(pixels, x, y).r == 0);
        }
    }

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("The composite texture is reused across frames")
{
    // The encoder caches one composite texture per eye per slot; reallocating
    // it every frame would be a per-frame allocation on the hot path.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    id<MTLTexture> first = nil;
    id<MTLTexture> second = nil;
    Compose(fixture, renderer, background, frameSource, true, &cached, &first);
    Compose(fixture, renderer, background, frameSource, true, &cached, &second);
    CHECK(first == second);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("The quad texture is not flipped or transposed")
{
    // The one that catches a UV mistake a uniformly coloured quad cannot: the
    // texture's top-left texel must land at the quad's upper-left corner,
    // which in a Metal render target is the *low* pixel row.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> background =
        MakeSolidTexture(fixture.device, kEyeSize, kEyeSize, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeQuadrantTexture(fixture.device, 32,
                                                     Rgba{255, 0, 0, 255},    // top-left: red
                                                     Rgba{0, 255, 0, 255},    // top-right: green
                                                     Rgba{255, 255, 0, 255},  // bottom-left: yellow
                                                     Rgba{0, 255, 255, 255}); // bottom-right: cyan
    REQUIRE(quadTexture != nil);

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    const std::vector<Rgba> pixels =
        Compose(fixture, renderer, background, frameSource, true, &cached);

    // The quad covers pixels [16, 48); sample well inside each of its
    // quadrants so linear filtering at the seams cannot blur the verdict.
    const Rgba& upperLeft = At(pixels, 22, 22);
    const Rgba& upperRight = At(pixels, 42, 22);
    const Rgba& lowerLeft = At(pixels, 22, 42);
    const Rgba& lowerRight = At(pixels, 42, 42);

    CHECK(upperLeft.r == 255);
    CHECK(upperLeft.g == 0);

    CHECK(upperRight.r == 0);
    CHECK(upperRight.g == 255);
    CHECK(upperRight.b == 0);

    CHECK(lowerLeft.r == 255);
    CHECK(lowerLeft.g == 255);

    CHECK(lowerRight.r == 0);
    CHECK(lowerRight.g == 255);
    CHECK(lowerRight.b == 255);

    ReleaseCached(cached);
    [background release];
    [quadTexture release];
}

TEST_CASE("A quad over a 1x1 eye image composites at the encoded eye size")
{
    // HITMAN 3 (through OpenComposite) shows loading screens and cutscenes as an overlay over
    // 1x1 black eye textures. Composing at the eye image's own size reduced the whole overlay to
    // one texel, so the headset showed black. The encoder passes its eye size as the minimum.
    MetalFixture fixture = MakeFixture();
    if (!fixture.Available())
    {
        SUCCEED("No Metal device available");
        return;
    }

    QuadLayerRenderer renderer;
    id<MTLTexture> tinyEye = MakeSolidTexture(fixture.device, 1, 1, Rgba{0, 0, kBackgroundBlue, 255});
    id<MTLTexture> quadTexture = MakeSolidTexture(fixture.device, 16, 16, Rgba{255, 0, 0, 255});
    REQUIRE(tinyEye != nil);
    REQUIRE(quadTexture != nil);

    FrameSource frameSource = MakeFrameSource();
    frameSource.quads.push_back(MakeQuad(quadTexture, PoseAt(0.0f, 0.0f, -1.0f), 1.0f, 1.0f));

    void* cached = nullptr;
    id<MTLCommandBuffer> cmd = [fixture.queue commandBuffer];
    void* composed = renderer.ComposeEye((__bridge void*)cmd, (__bridge void*)tinyEye, frameSource,
                                         true, &cached, kEyeSize, kEyeSize);
    [cmd commit];
    [cmd waitUntilCompleted];
    REQUIRE(composed != nullptr);
    id<MTLTexture> composedTexture = (__bridge id<MTLTexture>)composed;
    CHECK(composedTexture.width == kEyeSize);
    CHECK(composedTexture.height == kEyeSize);

    const std::vector<Rgba> pixels = ReadBack(fixture.device, fixture.queue, composedTexture);
    // The quad at full resolution, the 1x1 eye stretched behind it.
    CHECK(At(pixels, 32, 32).r == 255);
    CHECK(At(pixels, 17, 32).r == 255);
    CHECK(At(pixels, 46, 46).r == 255);
    CHECK(At(pixels, 14, 32).b == kBackgroundBlue);
    CHECK(At(pixels, 0, 0).b == kBackgroundBlue);
    CHECK(At(pixels, 63, 63).b == kBackgroundBlue);
    CHECK(At(pixels, 63, 63).r == 0);

    ReleaseCached(cached);
    [tinyEye release];
    [quadTexture release];
}
