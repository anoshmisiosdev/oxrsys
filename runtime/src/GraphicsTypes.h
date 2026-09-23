// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
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

// A sub-region of a swapchain image, in pixels. This mirrors XrRect2Di without
// pulling the OpenXR headers into this header, which the graphics backends and
// the streaming code include without an OpenXR dependency.
//
// An empty rect (a non-positive extent) means "the whole image", so a default
// constructed FrameImageSource keeps the pre-imageRect behaviour.
struct FrameImageRect
{
    int32_t offsetX = 0;
    int32_t offsetY = 0;
    int32_t width = 0;
    int32_t height = 0;

    bool IsEmpty() const
    {
        return width <= 0 || height <= 0;
    }

    bool CoversFullImage(uint32_t imageWidth, uint32_t imageHeight) const
    {
        if (IsEmpty())
        {
            return true;
        }
        return offsetX == 0 && offsetY == 0 &&
               static_cast<int64_t>(width) >= static_cast<int64_t>(imageWidth) &&
               static_cast<int64_t>(height) >= static_cast<int64_t>(imageHeight);
    }
};

// Resolve a submitted rect against the real image size: an empty rect becomes
// the full image, and anything reaching outside the image is clipped so the
// consumers can use the result as a blit/sample region without re-checking it.
inline FrameImageRect ResolveFrameImageRect(const FrameImageRect& rect,
                                            uint32_t imageWidth,
                                            uint32_t imageHeight)
{
    FrameImageRect full = {};
    full.width = static_cast<int32_t>(imageWidth);
    full.height = static_cast<int32_t>(imageHeight);

    if (rect.IsEmpty() || imageWidth == 0 || imageHeight == 0)
    {
        return full;
    }

    const int32_t maxX = static_cast<int32_t>(imageWidth);
    const int32_t maxY = static_cast<int32_t>(imageHeight);

    FrameImageRect resolved = {};
    resolved.offsetX = std::clamp(rect.offsetX, 0, maxX);
    resolved.offsetY = std::clamp(rect.offsetY, 0, maxY);
    resolved.width = std::min(rect.width, maxX - resolved.offsetX);
    resolved.height = std::min(rect.height, maxY - resolved.offsetY);
    if (resolved.IsEmpty())
    {
        return full;
    }
    return resolved;
}

// Normalized texture-coordinate remap for a resolved rect: sampling at
// offset + uv * scale, with uv in [0,1], walks the rect instead of the image.
struct FrameImageUvTransform
{
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
};

inline FrameImageUvTransform MakeFrameImageUvTransform(const FrameImageRect& resolvedRect,
                                                       uint32_t imageWidth,
                                                       uint32_t imageHeight)
{
    FrameImageUvTransform transform = {};
    if (imageWidth == 0 || imageHeight == 0 || resolvedRect.IsEmpty())
    {
        return transform;
    }
    const float widthF = static_cast<float>(imageWidth);
    const float heightF = static_cast<float>(imageHeight);
    transform.scaleX = static_cast<float>(resolvedRect.width) / widthF;
    transform.scaleY = static_cast<float>(resolvedRect.height) / heightF;
    transform.offsetX = static_cast<float>(resolvedRect.offsetX) / widthF;
    transform.offsetY = static_cast<float>(resolvedRect.offsetY) / heightF;
    return transform;
}

// Affine source->destination mapping for a crop-and-scale blit, laid out like
// MPSScaleTransform: dst = src * scale + translate.
struct FrameImageScaleTransform
{
    double scaleX = 1.0;
    double scaleY = 1.0;
    double translateX = 0.0;
    double translateY = 0.0;
};

inline FrameImageScaleTransform MakeFrameImageScaleTransform(const FrameImageRect& resolvedRect,
                                                             uint32_t destWidth,
                                                             uint32_t destHeight)
{
    FrameImageScaleTransform transform = {};
    if (resolvedRect.IsEmpty() || destWidth == 0 || destHeight == 0)
    {
        return transform;
    }
    transform.scaleX = static_cast<double>(destWidth) / static_cast<double>(resolvedRect.width);
    transform.scaleY = static_cast<double>(destHeight) / static_cast<double>(resolvedRect.height);
    transform.translateX = -static_cast<double>(resolvedRect.offsetX) * transform.scaleX;
    transform.translateY = -static_cast<double>(resolvedRect.offsetY) * transform.scaleY;
    return transform;
}

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

    // sourceX/Y/Width/Height carry the XrSwapchainSubImage::imageRect the app
    // submitted this image with (a zero extent means the whole image). Rect()
    // and SetRect() view the same fields as a FrameImageRect, so there is one
    // copy of the rect, not two.
    FrameImageRect Rect() const
    {
        FrameImageRect rect = {};
        if (!HasSourceRect())
        {
            return rect;
        }
        rect.offsetX = static_cast<int32_t>(sourceX);
        rect.offsetY = static_cast<int32_t>(sourceY);
        rect.width = static_cast<int32_t>(sourceWidth);
        rect.height = static_cast<int32_t>(sourceHeight);
        return rect;
    }

    void SetRect(const FrameImageRect& rect)
    {
        if (rect.IsEmpty())
        {
            sourceX = sourceY = sourceWidth = sourceHeight = 0;
            return;
        }
        sourceX = static_cast<uint32_t>(std::max(rect.offsetX, 0));
        sourceY = static_cast<uint32_t>(std::max(rect.offsetY, 0));
        sourceWidth = static_cast<uint32_t>(rect.width);
        sourceHeight = static_cast<uint32_t>(rect.height);
    }

    // The region of this image the app actually asked to be presented, clipped
    // to the real image size.
    FrameImageRect GetRect(uint32_t textureWidth, uint32_t textureHeight) const
    {
        return ResolveFrameImageRect(Rect(), textureWidth, textureHeight);
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
