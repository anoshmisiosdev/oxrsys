// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>

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

struct FrameSyncToken
{
    GraphicsApi api = GraphicsApi::Metal;
    std::shared_ptr<void> waitObject = {};
    uint64_t waitValue = 0;

    bool IsValid() const
    {
        return waitObject != nullptr && waitValue != 0;
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

    // The XrSwapchainSubImage::imageRect the app submitted this image with.
    // Empty means the whole image.
    FrameImageRect rect = {};

    void* GetImage() const
    {
        return image.get();
    }

    // The region of this image the app actually asked to be presented, clipped
    // to the real image size.
    FrameImageRect GetRect(uint32_t imageWidth, uint32_t imageHeight) const
    {
        return ResolveFrameImageRect(rect, imageWidth, imageHeight);
    }

    bool IsValid() const
    {
        return image != nullptr;
    }

    void Reset()
    {
        image.reset();
        sync = {};
        lifetime.reset();
        rect = {};
    }
};

struct FrameSource
{
    FrameImageSource left = {};
    FrameImageSource right = {};

    bool IsStereoValid() const
    {
        return left.IsValid() && right.IsValid();
    }

    void Reset()
    {
        left.Reset();
        right.Reset();
    }
};
