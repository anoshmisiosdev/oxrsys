// SPDX-License-Identifier: MPL-2.0
//
// GPU half of EyeFovRemap.h: redraws an eye image onto the display fov.

#pragma once

#include "EyeFovRemap.h"

#include <cstdint>
#include <vector>

namespace oxrsys::video
{

class EyeFovReprojector
{
public:
    EyeFovReprojector() = default;
    ~EyeFovReprojector();

    EyeFovReprojector(const EyeFovReprojector&) = delete;
    EyeFovReprojector& operator=(const EyeFovReprojector&) = delete;

    // `device` is an id<MTLDevice>. Safe to call repeatedly.
    bool EnsureInitialized(void* device);
    void Shutdown();

    // Draws `eyeTexture` (an id<MTLTexture>) resampled by `remap` into a same-sized,
    // same-format texture held in the caller-owned `*cachedTexture`, black where the
    // application rendered nothing. Returns that texture, or nullptr on failure (the
    // caller drops the frame). `commandBuffer` is an id<MTLCommandBuffer>.
    void* Reproject(void* commandBuffer, void* eyeTexture, const FovRemap& remap, void** cachedTexture);

private:
    void* GetPipeline(uint64_t pixelFormat);

    struct PipelineEntry
    {
        uint64_t pixelFormat = 0;
        void* pipeline = nullptr; // id<MTLRenderPipelineState>
    };

    bool initialized_ = false;
    void* device_ = nullptr;           // id<MTLDevice>
    void* library_ = nullptr;          // id<MTLLibrary>
    void* vertexFunction_ = nullptr;   // id<MTLFunction>
    void* fragmentFunction_ = nullptr; // id<MTLFunction>
    void* sampler_ = nullptr;          // id<MTLSamplerState>
    std::vector<PipelineEntry> pipelines_;
};

} // namespace oxrsys::video
