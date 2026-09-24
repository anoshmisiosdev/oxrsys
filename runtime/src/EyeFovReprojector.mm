// SPDX-License-Identifier: MPL-2.0

#import <Metal/Metal.h>

#include "EyeFovReprojector.h"

#include <spdlog/spdlog.h>

namespace oxrsys::video
{

namespace
{

// Sampling and writing the same pixel format keeps colour exact: an *_sRGB source is
// decoded on read and re-encoded on write.
constexpr const char* kEyeFovMetalSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct FovVaryings
{
    float4 position [[position]];
    float2 uv;
};

struct FovRemapUniforms
{
    float uScale;
    float uOffset;
    float vScale;
    float vOffset;
};

vertex FovVaryings eye_fov_vertex(uint vertexId [[vertex_id]])
{
    // One triangle covering the target; uv (0,0) is the top-left texel.
    float2 uv = float2((vertexId << 1) & 2, vertexId & 2);
    FovVaryings out;
    out.position = float4(uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    out.uv = uv;
    return out;
}

fragment float4 eye_fov_fragment(FovVaryings in [[stage_in]],
                                 texture2d<float, access::sample> eyeTexture [[texture(0)]],
                                 sampler eyeSampler [[sampler(0)]],
                                 constant FovRemapUniforms& remap [[buffer(0)]])
{
    float2 source = float2(in.uv.x * remap.uScale + remap.uOffset,
                           in.uv.y * remap.vScale + remap.vOffset);
    if (any(source < float2(0.0)) || any(source > float2(1.0)))
    {
        return float4(0.0, 0.0, 0.0, 1.0);
    }
    return eyeTexture.sample(eyeSampler, source);
}
)METAL";

struct FovRemapUniforms
{
    float uScale;
    float uOffset;
    float vScale;
    float vOffset;
};

} // namespace

EyeFovReprojector::~EyeFovReprojector()
{
    Shutdown();
}

bool EyeFovReprojector::EnsureInitialized(void* device)
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
    id<MTLLibrary> library =
        [metalDevice newLibraryWithSource:[NSString stringWithUTF8String:kEyeFovMetalSource]
                                  options:nil
                                    error:&error];
    if (library == nil)
    {
        spdlog::error("EyeFovReprojector: failed to compile shader: {}",
                      error != nil ? error.localizedDescription.UTF8String : "unknown error");
        return false;
    }
    id<MTLFunction> vertexFunction = [library newFunctionWithName:@"eye_fov_vertex"];
    id<MTLFunction> fragmentFunction = [library newFunctionWithName:@"eye_fov_fragment"];
    MTLSamplerDescriptor* samplerDescriptor = [[MTLSamplerDescriptor alloc] init];
    samplerDescriptor.minFilter = MTLSamplerMinMagFilterLinear;
    samplerDescriptor.magFilter = MTLSamplerMinMagFilterLinear;
    samplerDescriptor.sAddressMode = MTLSamplerAddressModeClampToEdge;
    samplerDescriptor.tAddressMode = MTLSamplerAddressModeClampToEdge;
    id<MTLSamplerState> sampler = [metalDevice newSamplerStateWithDescriptor:samplerDescriptor];
    [samplerDescriptor release];
    if (vertexFunction == nil || fragmentFunction == nil || sampler == nil)
    {
        spdlog::error("EyeFovReprojector: failed to create shader functions or sampler");
        [vertexFunction release];
        [fragmentFunction release];
        [sampler release];
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

void EyeFovReprojector::Shutdown()
{
    for (PipelineEntry& entry : pipelines_)
    {
        [(id<MTLRenderPipelineState>)entry.pipeline release];
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

void* EyeFovReprojector::GetPipeline(uint64_t pixelFormat)
{
    for (const PipelineEntry& entry : pipelines_)
    {
        if (entry.pixelFormat == pixelFormat)
        {
            return entry.pipeline;
        }
    }
    MTLRenderPipelineDescriptor* descriptor = [[MTLRenderPipelineDescriptor alloc] init];
    descriptor.vertexFunction = (id<MTLFunction>)vertexFunction_;
    descriptor.fragmentFunction = (id<MTLFunction>)fragmentFunction_;
    descriptor.colorAttachments[0].pixelFormat = (MTLPixelFormat)pixelFormat;
    NSError* error = nil;
    id<MTLRenderPipelineState> pipeline =
        [(id<MTLDevice>)device_ newRenderPipelineStateWithDescriptor:descriptor error:&error];
    [descriptor release];
    if (pipeline == nil)
    {
        spdlog::warn("EyeFovReprojector: failed to create pipeline for format {}: {}", pixelFormat,
                     error != nil ? error.localizedDescription.UTF8String : "unknown error");
        return nullptr;
    }
    pipelines_.push_back(PipelineEntry{pixelFormat, (void*)pipeline});
    return (void*)pipeline;
}

void* EyeFovReprojector::Reproject(void* commandBuffer, void* eyeTexture, const FovRemap& remap,
                                   void** cachedTexture)
{
    id<MTLCommandBuffer> cmdBuf = (__bridge id<MTLCommandBuffer>)commandBuffer;
    id<MTLTexture> source = (__bridge id<MTLTexture>)eyeTexture;
    if (cmdBuf == nil || source == nil || cachedTexture == nullptr ||
        !EnsureInitialized((void*)cmdBuf.device))
    {
        return nullptr;
    }
    id<MTLRenderPipelineState> pipeline =
        (id<MTLRenderPipelineState>)GetPipeline(static_cast<uint64_t>(source.pixelFormat));
    if (pipeline == nil)
    {
        return nullptr;
    }

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
        id<MTLTexture> created = [(id<MTLDevice>)device_ newTextureWithDescriptor:descriptor];
        if (created == nil)
        {
            return nullptr;
        }
        [destination release];
        destination = created;
        *cachedTexture = (void*)destination;
    }

    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = destination;
    pass.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> encoder = [cmdBuf renderCommandEncoderWithDescriptor:pass];
    if (encoder == nil)
    {
        return nullptr;
    }
    FovRemapUniforms uniforms = {remap.uScale, remap.uOffset, remap.vScale, remap.vOffset};
    [encoder setRenderPipelineState:pipeline];
    [encoder setFragmentTexture:source atIndex:0];
    [encoder setFragmentSamplerState:(id<MTLSamplerState>)sampler_ atIndex:0];
    [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:0];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [encoder endEncoding];
    return (void*)destination;
}

} // namespace oxrsys::video
