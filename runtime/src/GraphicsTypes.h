// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

enum class GraphicsApi
{
    Metal,
    Vulkan,
};

struct VulkanGraphicsContext
{
    void* instance = nullptr;
    void* physicalDevice = nullptr;
    void* device = nullptr;
    void* queue = nullptr;
    uint32_t queueFamilyIndex = 0;
    uint32_t queueIndex = 0;
};

struct GraphicsContext
{
    GraphicsApi api = GraphicsApi::Metal;
    void* metalDevice = nullptr;
    void* metalCommandQueue = nullptr;
    VulkanGraphicsContext vulkan = {};

    static GraphicsContext Metal(void* device, void* commandQueue = nullptr)
    {
        GraphicsContext context = {};
        context.api = GraphicsApi::Metal;
        context.metalDevice = device;
        context.metalCommandQueue = commandQueue;
        return context;
    }

    static GraphicsContext Vulkan(const VulkanGraphicsContext& vulkanContext,
                                  void* debugMetalDevice = nullptr)
    {
        GraphicsContext context = {};
        context.api = GraphicsApi::Vulkan;
        context.metalDevice = debugMetalDevice;
        context.vulkan = vulkanContext;
        return context;
    }

};

enum class FrameSyncKind
{
    None,
    MetalSharedEvent,
    HostFence,
};

struct FrameSyncToken
{
    FrameSyncKind kind = FrameSyncKind::None;
    std::shared_ptr<void> waitObject = {};
    uint64_t waitValue = 0;
    std::function<bool(uint64_t)> waitForReady = {};

    bool IsValid() const
    {
        switch (kind)
        {
            case FrameSyncKind::MetalSharedEvent:
                return waitObject != nullptr && waitValue != 0;
            case FrameSyncKind::HostFence:
                return static_cast<bool>(waitForReady);
            case FrameSyncKind::None:
            default:
                return false;
        }
    }

    bool WaitForHostReady(uint64_t timeoutNs) const
    {
        return kind != FrameSyncKind::HostFence ||
               (waitForReady && waitForReady(timeoutNs));
    }
};

struct FrameImageSource
{
    GraphicsApi api = GraphicsApi::Metal;
    std::shared_ptr<void> image = {};
    FrameSyncToken sync = {};
    std::shared_ptr<void> lifetime = {};
    uint32_t sourceX = 0;
    uint32_t sourceY = 0;
    uint32_t sourceWidth = 0;
    uint32_t sourceHeight = 0;
    uint64_t sourceFormat = 0;
    uint32_t imageWidth = 0;
    uint32_t imageHeight = 0;

    void* GetImage() const
    {
        return image.get();
    }

    bool IsValid() const
    {
        return image != nullptr;
    }

    bool HasSourceRect() const
    {
        return sourceWidth != 0 && sourceHeight != 0;
    }

    void Reset()
    {
        image.reset();
        sync = {};
        lifetime.reset();
        sourceX = 0;
        sourceY = 0;
        sourceWidth = 0;
        sourceHeight = 0;
        sourceFormat = 0;
        imageWidth = 0;
        imageHeight = 0;
    }
};

// Rigid transform, OpenXR convention (right-handed, -Z forward, +Y up) but
// expressed with plain scalars so this header stays free of OpenXR types --
// Session.cpp converts at the OpenXR boundary.
struct FramePose
{
    // Quaternion, x/y/z/w order to match XrQuaternionf's memory layout.
    float orientation[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float position[3] = {0.0f, 0.0f, 0.0f};
};

// Half-angles in radians, same signs as XrFovf: left/down negative,
// right/up positive.
struct FrameFov
{
    float angleLeft = 0.0f;
    float angleRight = 0.0f;
    float angleUp = 0.0f;
    float angleDown = 0.0f;
};

// The pose and projection an eye image was rendered with, expressed in the
// reference space the projection layer was submitted in. Quad layers are
// projected against this.
struct FrameEyeView
{
    FramePose pose = {};
    FrameFov fov = {};
    bool valid = false;
};

enum class FrameEyeVisibility
{
    Both,
    Left,
    Right,
};

// How a quad's sampled texel combines with the eye image underneath.
enum class FrameQuadBlend
{
    // No source-alpha blending requested: the quad is opaque.
    Opaque,
    // XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT, colour already
    // multiplied by alpha.
    PremultipliedAlpha,
    // ... plus XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT.
    UnpremultipliedAlpha,
};

// One XrCompositionLayerQuad, reduced to what the compositor needs.
struct FrameQuadLayer
{
    FrameImageSource image = {};
    // Quad centre, in the same reference space as FrameSource::views.
    FramePose pose = {};
    // Quad extent in metres, in the quad's own XY plane.
    float widthMeters = 0.0f;
    float heightMeters = 0.0f;
    FrameEyeVisibility eyeVisibility = FrameEyeVisibility::Both;
    FrameQuadBlend blend = FrameQuadBlend::Opaque;

    bool IsVisibleForEye(bool leftEye) const
    {
        switch (eyeVisibility)
        {
            case FrameEyeVisibility::Left:
                return leftEye;
            case FrameEyeVisibility::Right:
                return !leftEye;
            case FrameEyeVisibility::Both:
            default:
                return true;
        }
    }

    bool IsValid() const
    {
        return image.IsValid() && widthMeters > 0.0f && heightMeters > 0.0f;
    }
};

struct FrameSource
{
    FrameImageSource left = {};
    FrameImageSource right = {};
    bool alphaBlend = false;
    // Index 0 = left eye, 1 = right eye.
    FrameEyeView views[2] = {};
    // Quad layers submitted after the projection layer, in submission order:
    // later entries composite on top of earlier ones.
    std::vector<FrameQuadLayer> quads = {};

    bool IsStereoValid() const
    {
        return left.IsValid() && right.IsValid();
    }

    bool HasQuadLayers() const
    {
        return !quads.empty() && views[0].valid && views[1].valid;
    }

    void Reset()
    {
        left.Reset();
        right.Reset();
        alphaBlend = false;
        views[0] = {};
        views[1] = {};
        quads.clear();
    }
};
