// SPDX-License-Identifier: MPL-2.0

#import <Metal/Metal.h>

#include "QuadLayerRenderer.h"

#include "QuadLayerProjection.h"
#include "VideoTextureFormat.h"

#include <spdlog/spdlog.h>

#include <atomic>

namespace oxrsys::quad
{

namespace
{

// Vertices are already in clip space -- QuadLayerProjection.h does the matrix
// work on the CPU so the shader stays trivial and the maths stays testable
// without a GPU. The colour-space fixups mirror video_copy_kernel in
// VideoEncoder.mm: swapchain textures and the composite target do not always
// agree on whether they carry sRGB-encoded or linear values.
constexpr const char* kQuadLayerMetalSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct QuadVertex
{
    float4 position;
    float2 uv;
    float2 padding;
};

struct QuadVaryings
{
    float4 position [[position]];
    float2 uv;
};

struct QuadFragmentUniforms
{
    // 0 = leave as sampled, 1 = linear -> sRGB, 2 = sRGB -> linear.
    uint colorMode;
    // 1 when the layer opted out of source-alpha blending, in which case the
    // OpenXR spec says the texture's alpha is ignored entirely.
    uint forceOpaque;
    uint padding0;
    uint padding1;
};

static float3 linear_to_srgb(float3 value)
{
    value = clamp(value, float3(0.0), float3(1.0));
    float3 linearSegment = value * 12.92;
    float3 powerSegment = 1.055 * pow(value, float3(1.0 / 2.4)) - 0.055;
    return select(powerSegment, linearSegment, value <= float3(0.0031308));
}

static float3 srgb_to_linear(float3 value)
{
    value = clamp(value, float3(0.0), float3(1.0));
    float3 linearSegment = value / 12.92;
    float3 powerSegment = pow((value + 0.055) / 1.055, float3(2.4));
    return select(powerSegment, linearSegment, value <= float3(0.04045));
}

vertex QuadVaryings quad_layer_vertex(uint vertexId [[vertex_id]],
                                      constant QuadVertex* vertices [[buffer(0)]])
{
    QuadVaryings out;
    out.position = vertices[vertexId].position;
    out.uv = vertices[vertexId].uv;
    return out;
}

fragment float4 quad_layer_fragment(QuadVaryings in [[stage_in]],
                                    texture2d<float, access::sample> quadTexture [[texture(0)]],
                                    sampler quadSampler [[sampler(0)]],
                                    constant QuadFragmentUniforms& params [[buffer(0)]])
{
    float4 color = quadTexture.sample(quadSampler, in.uv);
    if (params.colorMode == 1u)
    {
        color.rgb = linear_to_srgb(color.rgb);
    }
    else if (params.colorMode == 2u)
    {
        color.rgb = srgb_to_linear(color.rgb);
    }
    if (params.forceOpaque != 0u)
    {
        color.a = 1.0;
    }
    return color;
}
)METAL";

struct QuadVertex
{
    float position[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float uv[2] = {0.0f, 0.0f};
    float padding[2] = {0.0f, 0.0f};
};

struct QuadFragmentUniforms
{
    uint32_t colorMode = 0;
    uint32_t forceOpaque = 0;
    uint32_t padding0 = 0;
    uint32_t padding1 = 0;
};

// The source and the destination do not always agree on whether their stored
// values are sRGB-encoded. Metal decodes on read and encodes on write only for
// *_sRGB formats, so a mismatch needs an explicit conversion in the shader.
uint32_t SelectColorMode(MTLPixelFormat sourceFormat, MTLPixelFormat destinationFormat)
{
    const bool sourceSrgb =
        oxrsys::video::IsSrgbTextureFormat(static_cast<uint64_t>(sourceFormat));
    const bool destinationSrgb =
        oxrsys::video::IsSrgbTextureFormat(static_cast<uint64_t>(destinationFormat));
    if (sourceSrgb == destinationSrgb)
    {
        return 0;
    }
    // Source decoded to linear, destination stores raw values: re-encode.
    return sourceSrgb ? 1u : 2u;
}

std::atomic_bool g_loggedQuadFailure{false};

void LogQuadFailureOnce(const char* reason)
{
    if (!g_loggedQuadFailure.exchange(true))
    {
        spdlog::warn("QuadLayerRenderer: {}", reason);
    }
}

} // namespace

QuadLayerRenderer::~QuadLayerRenderer()
{
    Shutdown();
}

bool QuadLayerRenderer::HasWorkForEye(const FrameSource& frameSource, bool leftEye)
{
    if (!frameSource.HasQuadLayers())
    {
        return false;
    }
    for (const FrameQuadLayer& quad : frameSource.quads)
    {
        if (quad.IsValid() && quad.IsVisibleForEye(leftEye))
        {
            return true;
        }
    }
    return false;
}

bool QuadLayerRenderer::EnsureInitialized(void* device)
{
    if (initialized_)
    {
        return true;
    }
    id<MTLDevice> metalDevice = (__bridge id<MTLDevice>)device;
    if (metalDevice == nil)
    {
        return false;
    }

    NSError* error = nil;
    NSString* source = [NSString stringWithUTF8String:kQuadLayerMetalSource];
    id<MTLLibrary> library = [metalDevice newLibraryWithSource:source options:nil error:&error];
    if (library == nil)
    {
        spdlog::error("QuadLayerRenderer: failed to compile quad shader: {}",
                      error != nil ? error.localizedDescription.UTF8String : "unknown error");
        return false;
    }

    id<MTLFunction> vertexFunction = [library newFunctionWithName:@"quad_layer_vertex"];
    id<MTLFunction> fragmentFunction = [library newFunctionWithName:@"quad_layer_fragment"];
    if (vertexFunction == nil || fragmentFunction == nil)
    {
        spdlog::error("QuadLayerRenderer: failed to load quad shader entry points");
        [vertexFunction release];
        [fragmentFunction release];
        [library release];
        return false;
    }

    MTLSamplerDescriptor* samplerDescriptor = [[MTLSamplerDescriptor alloc] init];
    samplerDescriptor.minFilter = MTLSamplerMinMagFilterLinear;
    samplerDescriptor.magFilter = MTLSamplerMinMagFilterLinear;
    samplerDescriptor.sAddressMode = MTLSamplerAddressModeClampToEdge;
    samplerDescriptor.tAddressMode = MTLSamplerAddressModeClampToEdge;
    id<MTLSamplerState> sampler = [metalDevice newSamplerStateWithDescriptor:samplerDescriptor];
    [samplerDescriptor release];
    if (sampler == nil)
    {
        spdlog::error("QuadLayerRenderer: failed to create quad sampler");
        [vertexFunction release];
        [fragmentFunction release];
        [library release];
        return false;
    }

    device_ = (void*)[metalDevice retain];
    library_ = (void*)library;
    vertexFunction_ = (void*)vertexFunction;
    fragmentFunction_ = (void*)fragmentFunction;
    sampler_ = (void*)sampler;
    initialized_ = true;
    return true;
}

void QuadLayerRenderer::Shutdown()
{
    for (PipelineEntry& entry : pipelines_)
    {
        if (entry.pipeline != nullptr)
        {
            [(id<MTLRenderPipelineState>)entry.pipeline release];
            entry.pipeline = nullptr;
        }
    }
    pipelines_.clear();

    if (sampler_ != nullptr)
    {
        [(id<MTLSamplerState>)sampler_ release];
        sampler_ = nullptr;
    }
    if (fragmentFunction_ != nullptr)
    {
        [(id<MTLFunction>)fragmentFunction_ release];
        fragmentFunction_ = nullptr;
    }
    if (vertexFunction_ != nullptr)
    {
        [(id<MTLFunction>)vertexFunction_ release];
        vertexFunction_ = nullptr;
    }
    if (library_ != nullptr)
    {
        [(id<MTLLibrary>)library_ release];
        library_ = nullptr;
    }
    if (device_ != nullptr)
    {
        [(id<MTLDevice>)device_ release];
        device_ = nullptr;
    }
    initialized_ = false;
}

void* QuadLayerRenderer::GetPipeline(uint64_t pixelFormat, FrameQuadBlend blend)
{
    for (const PipelineEntry& entry : pipelines_)
    {
        if (entry.pixelFormat == pixelFormat && entry.blend == blend)
        {
            return entry.pipeline;
        }
    }

    id<MTLDevice> metalDevice = (__bridge id<MTLDevice>)device_;
    if (metalDevice == nil)
    {
        return nullptr;
    }

    MTLRenderPipelineDescriptor* descriptor = [[MTLRenderPipelineDescriptor alloc] init];
    descriptor.vertexFunction = (id<MTLFunction>)vertexFunction_;
    descriptor.fragmentFunction = (id<MTLFunction>)fragmentFunction_;
    MTLRenderPipelineColorAttachmentDescriptor* attachment = descriptor.colorAttachments[0];
    attachment.pixelFormat = (MTLPixelFormat)pixelFormat;

    switch (blend)
    {
        case FrameQuadBlend::Opaque:
            // XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT unset: the
            // texture's alpha is ignored and the quad fully replaces what is
            // underneath.
            attachment.blendingEnabled = NO;
            break;
        case FrameQuadBlend::PremultipliedAlpha:
            attachment.blendingEnabled = YES;
            attachment.rgbBlendOperation = MTLBlendOperationAdd;
            attachment.alphaBlendOperation = MTLBlendOperationAdd;
            attachment.sourceRGBBlendFactor = MTLBlendFactorOne;
            attachment.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            attachment.sourceAlphaBlendFactor = MTLBlendFactorOne;
            attachment.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            break;
        case FrameQuadBlend::UnpremultipliedAlpha:
            attachment.blendingEnabled = YES;
            attachment.rgbBlendOperation = MTLBlendOperationAdd;
            attachment.alphaBlendOperation = MTLBlendOperationAdd;
            attachment.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
            attachment.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            attachment.sourceAlphaBlendFactor = MTLBlendFactorOne;
            attachment.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            break;
    }

    NSError* error = nil;
    id<MTLRenderPipelineState> pipeline =
        [metalDevice newRenderPipelineStateWithDescriptor:descriptor error:&error];
    [descriptor release];
    if (pipeline == nil)
    {
        spdlog::warn("QuadLayerRenderer: failed to create quad pipeline for format {}: {}",
                     pixelFormat,
                     error != nil ? error.localizedDescription.UTF8String : "unknown error");
        return nullptr;
    }

    pipelines_.push_back(PipelineEntry{pixelFormat, blend, (void*)pipeline});
    return (void*)pipeline;
}

void* QuadLayerRenderer::ComposeEye(void* commandBuffer, void* eyeTexture,
                                    const FrameSource& frameSource, bool leftEye,
                                    void** cachedTexture)
{
    if (!HasWorkForEye(frameSource, leftEye))
    {
        return eyeTexture;
    }

    id<MTLCommandBuffer> cmdBuf = (__bridge id<MTLCommandBuffer>)commandBuffer;
    id<MTLTexture> source = (__bridge id<MTLTexture>)eyeTexture;
    if (cmdBuf == nil || source == nil || cachedTexture == nullptr)
    {
        return nullptr;
    }
    if (!EnsureInitialized((void*)cmdBuf.device))
    {
        LogQuadFailureOnce("quad renderer unavailable, dropping quad layers");
        return eyeTexture;
    }

    const FrameEyeView& view = frameSource.views[leftEye ? 0 : 1];

    // Reuse the per-eye composite texture; it only changes when the eye image's
    // size or format changes, which happens at session start or a streaming
    // reconfigure, not per frame.
    id<MTLTexture> destination = (__bridge id<MTLTexture>)*cachedTexture;
    if (destination == nil || destination.pixelFormat != source.pixelFormat ||
        destination.width != source.width || destination.height != source.height)
    {
        MTLTextureDescriptor* descriptor =
            [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:source.pixelFormat
                                                               width:source.width
                                                              height:source.height
                                                           mipmapped:NO];
        descriptor.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite |
                           MTLTextureUsageRenderTarget;
        descriptor.storageMode = MTLStorageModePrivate;
        id<MTLTexture> created = [(__bridge id<MTLDevice>)device_ newTextureWithDescriptor:descriptor];
        if (created == nil)
        {
            LogQuadFailureOnce("failed to allocate quad composite texture");
            return eyeTexture;
        }
        if (destination != nil)
        {
            [destination release];
        }
        destination = created;
        *cachedTexture = (void*)destination;
    }

    // Start from the projection layer's pixels, then draw the quads on top --
    // OpenXR composites layers in submission order.
    id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
    if (blit == nil)
    {
        LogQuadFailureOnce("failed to create quad background blit encoder");
        return nullptr;
    }
    [blit copyFromTexture:source
              sourceSlice:0
              sourceLevel:0
             sourceOrigin:MTLOriginMake(0, 0, 0)
               sourceSize:MTLSizeMake(source.width, source.height, 1)
                toTexture:destination
         destinationSlice:0
         destinationLevel:0
        destinationOrigin:MTLOriginMake(0, 0, 0)];
    [blit endEncoding];

    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = destination;
    pass.colorAttachments[0].loadAction = MTLLoadActionLoad;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;

    id<MTLRenderCommandEncoder> encoder = [cmdBuf renderCommandEncoderWithDescriptor:pass];
    if (encoder == nil)
    {
        LogQuadFailureOnce("failed to create quad render encoder");
        return nullptr;
    }
    [encoder setFragmentSamplerState:(id<MTLSamplerState>)sampler_ atIndex:0];

    bool drewAnything = false;
    for (const FrameQuadLayer& quad : frameSource.quads)
    {
        if (!quad.IsValid() || !quad.IsVisibleForEye(leftEye))
        {
            continue;
        }
        id<MTLTexture> quadTexture = (__bridge id<MTLTexture>)quad.image.GetImage();
        if (quadTexture == nil)
        {
            continue;
        }

        const QuadCorners corners =
            ProjectQuad(view, quad.pose, quad.widthMeters, quad.heightMeters);
        if (!corners.valid)
        {
            continue;
        }

        void* pipeline = GetPipeline(static_cast<uint64_t>(destination.pixelFormat), quad.blend);
        if (pipeline == nullptr)
        {
            continue;
        }

        const QuadUvRect uvRect = MakeUvRect(quad.image, static_cast<uint32_t>(quadTexture.width),
                                             static_cast<uint32_t>(quadTexture.height));

        // Triangle-strip order matching QuadCorners: UL, LL, UR, LR. OpenXR
        // puts the subImage rect's top-left texel at the quad's upper-left
        // corner, hence v = 0 on the +Y side.
        static constexpr float kBaseUv[4][2] = {
            {0.0f, 0.0f},
            {0.0f, 1.0f},
            {1.0f, 0.0f},
            {1.0f, 1.0f},
        };

        QuadVertex vertices[4] = {};
        for (size_t i = 0; i < 4; ++i)
        {
            vertices[i].position[0] = corners.positions[i].x;
            vertices[i].position[1] = corners.positions[i].y;
            vertices[i].position[2] = corners.positions[i].z;
            vertices[i].position[3] = corners.positions[i].w;
            vertices[i].uv[0] = uvRect.offsetU + kBaseUv[i][0] * uvRect.scaleU;
            vertices[i].uv[1] = uvRect.offsetV + kBaseUv[i][1] * uvRect.scaleV;
        }

        QuadFragmentUniforms uniforms = {};
        uniforms.colorMode = SelectColorMode(quadTexture.pixelFormat, destination.pixelFormat);
        uniforms.forceOpaque = quad.blend == FrameQuadBlend::Opaque ? 1u : 0u;

        [encoder setRenderPipelineState:(id<MTLRenderPipelineState>)pipeline];
        [encoder setVertexBytes:vertices length:sizeof(vertices) atIndex:0];
        [encoder setFragmentTexture:quadTexture atIndex:0];
        [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:0];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
        drewAnything = true;
    }

    [encoder endEncoding];
    // Every quad was culled (behind the eye, degenerate pose, unusable format).
    // The blit already produced an exact copy, so returning `destination` is
    // still correct; returning the source avoids the redundant copy next frame.
    return drewAnything ? (void*)destination : eyeTexture;
}

} // namespace oxrsys::quad
