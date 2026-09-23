// SPDX-License-Identifier: MPL-2.0
//
// Draws XR_TYPE_COMPOSITION_LAYER_QUAD layers over a composed eye image.
//
// Quads are composited into a per-eye copy *before* the encoder's existing
// downscale/convert/foveate paths run, rather than on top of the final encode
// target. Everything downstream consumes the returned texture unchanged, so the
// foveated path gets quads for free and the quads are foveated along with the
// rest of the eye image instead of being warped after the fact.

#pragma once

#include "GraphicsTypes.h"

#include <cstdint>
#include <vector>

namespace oxrsys::quad
{

// Cached GPU resources for quad compositing. Owns its pipelines; the per-eye
// destination textures are owned by the caller so they can live in the
// encoder's existing per-slot cache.
class QuadLayerRenderer
{
public:
    QuadLayerRenderer() = default;
    ~QuadLayerRenderer();

    QuadLayerRenderer(const QuadLayerRenderer&) = delete;
    QuadLayerRenderer& operator=(const QuadLayerRenderer&) = delete;

    // `device` is an id<MTLDevice>. Safe to call repeatedly.
    bool EnsureInitialized(void* device);
    void Shutdown();

    bool IsInitialized() const { return initialized_; }

    // Composites every quad visible in `leftEye` over a copy of `eyeTexture`,
    // in submission order, and returns the composited id<MTLTexture>.
    //
    // Returns `eyeTexture` unchanged when there is nothing to draw. Returns
    // nullptr only when compositing was attempted and failed, which the caller
    // should treat as a dropped frame.
    //
    // `commandBuffer` is an id<MTLCommandBuffer>; `cachedTexture` points at a
    // caller-owned slot holding the reusable destination texture, reallocated
    // here when the eye size or format changes.
    void* ComposeEye(void* commandBuffer, void* eyeTexture, const FrameSource& frameSource,
                     bool leftEye, void** cachedTexture);

    // True when at least one quad in `frameSource` would be drawn for this eye.
    static bool HasWorkForEye(const FrameSource& frameSource, bool leftEye);

private:
    // One render pipeline per (destination pixel format, blend mode).
    struct PipelineEntry
    {
        uint64_t pixelFormat = 0;
        FrameQuadBlend blend = FrameQuadBlend::Opaque;
        void* pipeline = nullptr; // id<MTLRenderPipelineState>
    };

    void* GetPipeline(uint64_t pixelFormat, FrameQuadBlend blend);

    bool initialized_ = false;
    void* device_ = nullptr;         // id<MTLDevice>
    void* library_ = nullptr;        // id<MTLLibrary>
    void* vertexFunction_ = nullptr; // id<MTLFunction>
    void* fragmentFunction_ = nullptr; // id<MTLFunction>
    void* sampler_ = nullptr;        // id<MTLSamplerState>
    std::vector<PipelineEntry> pipelines_;
};

} // namespace oxrsys::quad
