// SPDX-License-Identifier: MPL-2.0

#include "VideoFramePreparation.h"

#include "Swapchain.h"

#include <algorithm>
#include <cstring>
#include <optional>

#if defined(_WIN32)
#include <synchapi.h>
#endif

namespace
{

enum class PixelLayout
{
    Rgba,
    Bgra,
};

struct SourceRect
{
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

struct SourceImage
{
    std::vector<uint8_t> pixels;
    uint32_t width = 0;
    uint32_t height = 0;
    PixelLayout layout = PixelLayout::Rgba;
};

struct YuvSample
{
    uint8_t y = 16;
    uint8_t u = 128;
    uint8_t v = 128;
};

std::optional<PixelLayout> PixelLayoutForVulkanFormat(VkFormat format)
{
    switch (format)
    {
        case VK_FORMAT_R8G8B8A8_UNORM:
        case VK_FORMAT_R8G8B8A8_SRGB:
            return PixelLayout::Rgba;
        case VK_FORMAT_B8G8R8A8_UNORM:
        case VK_FORMAT_B8G8R8A8_SRGB:
            return PixelLayout::Bgra;
        default:
            return std::nullopt;
    }
}

#ifdef XR_USE_GRAPHICS_API_OPENGL
std::optional<PixelLayout> PixelLayoutForOpenGLReadFormat(uint64_t format)
{
    switch (static_cast<GLenum>(format))
    {
        case GL_RGBA:
            return PixelLayout::Rgba;
        case GL_BGRA:
            return PixelLayout::Bgra;
        default:
            return std::nullopt;
    }
}
#endif

#if defined(_WIN32) && (defined(OXRSYS_USE_D3D11) || defined(OXRSYS_USE_D3D12) || \
                        defined(XR_USE_GRAPHICS_API_D3D11) || defined(XR_USE_GRAPHICS_API_D3D12))
std::optional<PixelLayout> PixelLayoutForDxgiFormat(DXGI_FORMAT format)
{
    switch (format)
    {
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            return PixelLayout::Rgba;
        case DXGI_FORMAT_B8G8R8A8_UNORM:
        case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
            return PixelLayout::Bgra;
        default:
            return std::nullopt;
    }
}
#endif

uint8_t ClampByte(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

YuvSample RgbToBt709Nv12(uint8_t r, uint8_t g, uint8_t b)
{
    // BT.709 limited-range integer approximation.
    YuvSample sample;
    sample.y = ClampByte(16 + ((47 * r + 157 * g + 16 * b + 128) >> 8));
    sample.u = ClampByte(128 + ((-26 * r - 87 * g + 112 * b + 128) >> 8));
    sample.v = ClampByte(128 + ((112 * r - 102 * g - 10 * b + 128) >> 8));
    return sample;
}

std::optional<SourceRect> ResolveSourceRect(const FrameImageSource& source,
                                            uint32_t imageWidth,
                                            uint32_t imageHeight)
{
    SourceRect rect = {};
    if (source.HasSourceRect())
    {
        rect.x = source.sourceX;
        rect.y = source.sourceY;
        rect.width = source.sourceWidth;
        rect.height = source.sourceHeight;
    }
    else
    {
        rect.width = imageWidth;
        rect.height = imageHeight;
    }

    const uint64_t maxX = static_cast<uint64_t>(rect.x) + rect.width;
    const uint64_t maxY = static_cast<uint64_t>(rect.y) + rect.height;
    if (rect.width == 0 || rect.height == 0 || maxX > imageWidth || maxY > imageHeight)
    {
        return std::nullopt;
    }
    return rect;
}

bool ReadVulkanFrameSource(const Swapchain::VulkanFrameSource& source,
                           SourceImage& output)
{
    const auto layout = PixelLayoutForVulkanFormat(source.format);
    if (!layout.has_value() || source.stagingBuffer == VK_NULL_HANDLE ||
        source.stagingMemory == VK_NULL_HANDLE || source.mapMemory == nullptr ||
        source.unmapMemory == nullptr || source.width == 0 || source.height == 0)
    {
        return false;
    }

    const size_t readbackSize = static_cast<size_t>(source.width) * source.height * 4u;
    if (source.stagingSize < readbackSize)
    {
        return false;
    }

    if (source.fenceSubmitted && source.fence != VK_NULL_HANDLE && source.waitForFences != nullptr &&
        source.waitForFences(
            reinterpret_cast<VkDevice>(source.context.device),
            1, &source.fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
    {
        return false;
    }

    void* mapped = nullptr;
    VkDevice device = reinterpret_cast<VkDevice>(source.context.device);
    if (source.mapMemory(device, source.stagingMemory, 0, readbackSize, 0, &mapped) != VK_SUCCESS)
    {
        return false;
    }
    if (source.invalidateMappedMemoryRanges != nullptr)
    {
        VkMappedMemoryRange range{};
        range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
        range.memory = source.stagingMemory;
        range.offset = 0;
        range.size = readbackSize;
        source.invalidateMappedMemoryRanges(device, 1, &range);
    }

    output.width = source.width;
    output.height = source.height;
    output.layout = *layout;
    output.pixels.resize(readbackSize);
    std::memcpy(output.pixels.data(), mapped, readbackSize);
    source.unmapMemory(device, source.stagingMemory);
    return true;
}

#ifdef XR_USE_GRAPHICS_API_OPENGL
bool ReadOpenGLFrameSource(const Swapchain::OpenGLFrameSource& source,
                           SourceImage& output)
{
    const auto layout = PixelLayoutForOpenGLReadFormat(source.format);
    if (!layout.has_value() || source.width == 0 || source.height == 0 || source.pixels.empty())
    {
        return false;
    }

    output.width = source.width;
    output.height = source.height;
    output.layout = *layout;
    output.pixels = source.pixels;
    return true;
}
#endif

#if defined(_WIN32) && (defined(OXRSYS_USE_D3D11) || defined(XR_USE_GRAPHICS_API_D3D11))
bool ReadD3D11FrameSource(const Swapchain::D3D11FrameSource& source,
                          SourceImage& output)
{
    const auto layout = PixelLayoutForDxgiFormat(source.format);
    if (!layout.has_value() || source.immediateContext == nullptr ||
        source.stagingTexture == nullptr || source.width == 0 || source.height == 0)
    {
        return false;
    }

    D3D11_MAPPED_SUBRESOURCE mapped = {};
    HRESULT result = source.immediateContext->Map(source.stagingTexture, 0, D3D11_MAP_READ, 0, &mapped);
    if (FAILED(result))
    {
        return false;
    }

    const size_t rowBytes = static_cast<size_t>(source.width) * 4u;
    output.width = source.width;
    output.height = source.height;
    output.layout = *layout;
    output.pixels.resize(rowBytes * source.height);
    const auto* srcRows = static_cast<const uint8_t*>(mapped.pData);
    for (uint32_t y = 0; y < source.height; ++y)
    {
        std::memcpy(output.pixels.data() + rowBytes * y,
                    srcRows + static_cast<size_t>(mapped.RowPitch) * y,
                    rowBytes);
    }

    source.immediateContext->Unmap(source.stagingTexture, 0);
    return true;
}
#endif

#if defined(_WIN32) && (defined(OXRSYS_USE_D3D12) || defined(XR_USE_GRAPHICS_API_D3D12))
bool ReadD3D12FrameSource(const Swapchain::D3D12FrameSource& source,
                          SourceImage& output)
{
    const auto layout = PixelLayoutForDxgiFormat(source.format);
    if (!layout.has_value() || source.readbackBuffer == nullptr ||
        source.width == 0 || source.height == 0 || source.totalBytes == 0)
    {
        return false;
    }

    if (source.fenceSubmitted && source.fence != nullptr &&
        source.fence->GetCompletedValue() < source.fenceValue)
    {
        if (source.fenceEvent != nullptr &&
            SUCCEEDED(source.fence->SetEventOnCompletion(source.fenceValue, source.fenceEvent)))
        {
            WaitForSingleObject(source.fenceEvent, INFINITE);
        }
        else
        {
            while (source.fence->GetCompletedValue() < source.fenceValue)
            {
                Sleep(1);
            }
        }
    }

    D3D12_RANGE readRange = {
        static_cast<SIZE_T>(source.footprint.Offset),
        static_cast<SIZE_T>(source.footprint.Offset + source.totalBytes),
    };
    void* mapped = nullptr;
    if (FAILED(source.readbackBuffer->Map(0, &readRange, &mapped)) || mapped == nullptr)
    {
        return false;
    }

    const size_t rowBytes = static_cast<size_t>(source.width) * 4u;
    output.width = source.width;
    output.height = source.height;
    output.layout = *layout;
    output.pixels.resize(rowBytes * source.height);
    const auto* srcRows = static_cast<const uint8_t*>(mapped) + source.footprint.Offset;
    for (uint32_t y = 0; y < source.height; ++y)
    {
        std::memcpy(output.pixels.data() + rowBytes * y,
                    srcRows + static_cast<size_t>(source.footprint.Footprint.RowPitch) * y,
                    rowBytes);
    }

    D3D12_RANGE writtenRange = {0, 0};
    source.readbackBuffer->Unmap(0, &writtenRange);
    return true;
}
#endif

bool ReadFrameImage(const FrameImageSource& frameImage,
                    const GraphicsContext& graphicsContext,
                    SourceImage& output)
{
    if (graphicsContext.api == GraphicsApi::Vulkan)
    {
        auto* source = static_cast<Swapchain::VulkanFrameSource*>(frameImage.GetImage());
        return source != nullptr && ReadVulkanFrameSource(*source, output);
    }

    if (graphicsContext.api == GraphicsApi::OpenGL)
    {
#ifdef XR_USE_GRAPHICS_API_OPENGL
        auto* source = static_cast<Swapchain::OpenGLFrameSource*>(frameImage.GetImage());
        return source != nullptr && ReadOpenGLFrameSource(*source, output);
#else
        return false;
#endif
    }

#if defined(_WIN32) && (defined(OXRSYS_USE_D3D11) || defined(XR_USE_GRAPHICS_API_D3D11))
    if (graphicsContext.api == GraphicsApi::D3D11)
    {
        auto* source = static_cast<Swapchain::D3D11FrameSource*>(frameImage.GetImage());
        return source != nullptr && ReadD3D11FrameSource(*source, output);
    }
#endif

#if defined(_WIN32) && (defined(OXRSYS_USE_D3D12) || defined(XR_USE_GRAPHICS_API_D3D12))
    if (graphicsContext.api == GraphicsApi::D3D12)
    {
        auto* source = static_cast<Swapchain::D3D12FrameSource*>(frameImage.GetImage());
        return source != nullptr && ReadD3D12FrameSource(*source, output);
    }
#endif

    return false;
}

YuvSample SampleSource(const SourceImage& image,
                       const SourceRect& rect,
                       uint32_t dstX,
                       uint32_t dstY,
                       uint32_t dstWidth,
                       uint32_t dstHeight)
{
    const uint32_t srcX = rect.x + static_cast<uint32_t>(
        (static_cast<uint64_t>(dstX) * rect.width) / std::max(dstWidth, 1u));
    const uint32_t srcY = rect.y + static_cast<uint32_t>(
        (static_cast<uint64_t>(dstY) * rect.height) / std::max(dstHeight, 1u));
    const uint32_t clampedX = std::min(srcX, image.width - 1);
    const uint32_t clampedY = std::min(srcY, image.height - 1);
    const size_t offset = (static_cast<size_t>(clampedY) * image.width + clampedX) * 4u;
    if (offset + 2 >= image.pixels.size())
    {
        return {};
    }

    const uint8_t c0 = image.pixels[offset + 0];
    const uint8_t c1 = image.pixels[offset + 1];
    const uint8_t c2 = image.pixels[offset + 2];
    if (image.layout == PixelLayout::Bgra)
    {
        return RgbToBt709Nv12(c2, c1, c0);
    }
    return RgbToBt709Nv12(c0, c1, c2);
}

bool AllocateNv12(uint32_t width, uint32_t height, PreparedVideoFrame& output)
{
    if (width == 0 || height == 0 || (width % 2u) != 0 || (height % 2u) != 0)
    {
        return false;
    }

    output.width = width;
    output.height = height;
    output.yStride = width;
    output.uvStride = width;
    const size_t yBytes = static_cast<size_t>(output.yStride) * output.height;
    const size_t uvBytes = static_cast<size_t>(output.uvStride) * (output.height / 2u);
    output.nv12.assign(yBytes + uvBytes, 0);
    return true;
}

bool BlitEyeToNv12(const SourceImage& image,
                   const SourceRect& rect,
                   uint32_t outputX,
                   uint32_t outputEyeWidth,
                   PreparedVideoFrame& output,
                   std::vector<uint32_t>& uAccum,
                   std::vector<uint32_t>& vAccum,
                   std::vector<uint32_t>& uvCount)
{
    if (image.width == 0 || image.height == 0 || outputEyeWidth == 0)
    {
        return false;
    }

    const uint32_t uvWidth = output.width / 2u;
    for (uint32_t y = 0; y < output.height; ++y)
    {
        uint8_t* yRow = output.YPlane() + static_cast<size_t>(y) * output.yStride;
        for (uint32_t x = 0; x < outputEyeWidth; ++x)
        {
            const YuvSample sample = SampleSource(image, rect, x, y, outputEyeWidth, output.height);
            const uint32_t dstX = outputX + x;
            yRow[dstX] = sample.y;

            const uint32_t uvIndex = (y / 2u) * uvWidth + (dstX / 2u);
            uAccum[uvIndex] += sample.u;
            vAccum[uvIndex] += sample.v;
            uvCount[uvIndex] += 1;
        }
    }
    return true;
}

bool FinalizeNv12Chroma(PreparedVideoFrame& output,
                        const std::vector<uint32_t>& uAccum,
                        const std::vector<uint32_t>& vAccum,
                        const std::vector<uint32_t>& uvCount)
{
    const uint32_t uvWidth = output.width / 2u;
    const uint32_t uvHeight = output.height / 2u;
    uint8_t* uvPlane = output.UVPlane();
    for (uint32_t y = 0; y < uvHeight; ++y)
    {
        uint8_t* uvRow = uvPlane + static_cast<size_t>(y) * output.uvStride;
        for (uint32_t x = 0; x < uvWidth; ++x)
        {
            const uint32_t index = y * uvWidth + x;
            const uint32_t count = std::max(uvCount[index], 1u);
            uvRow[x * 2u + 0u] = static_cast<uint8_t>(uAccum[index] / count);
            uvRow[x * 2u + 1u] = static_cast<uint8_t>(vAccum[index] / count);
        }
    }
    return true;
}

} // namespace

bool VideoFramePreparer::FillBlack(uint32_t outputWidth,
                                   uint32_t outputHeight,
                                   PreparedVideoFrame& output)
{
    if (!AllocateNv12(outputWidth, outputHeight, output))
    {
        return false;
    }
    std::memset(output.YPlane(), 16, static_cast<size_t>(output.yStride) * output.height);
    std::memset(output.UVPlane(), 128, static_cast<size_t>(output.uvStride) * (output.height / 2u));
    return true;
}

bool VideoFramePreparer::PrepareStereo(FrameSource frameSource,
                                       bool stereo,
                                       const GraphicsContext& graphicsContext,
                                       uint32_t outputWidth,
                                       uint32_t outputHeight,
                                       PreparedVideoFrame& output)
{
    if (!AllocateNv12(outputWidth, outputHeight, output))
    {
        return false;
    }

    SourceImage leftImage;
    SourceImage rightImage;
    if (!frameSource.left.IsValid() || (stereo && !frameSource.right.IsValid()) ||
        !ReadFrameImage(frameSource.left, graphicsContext, leftImage))
    {
        return false;
    }
    if (stereo)
    {
        if (!ReadFrameImage(frameSource.right, graphicsContext, rightImage))
        {
            return false;
        }
    }
    else
    {
        rightImage = leftImage;
    }

    const auto leftRect = ResolveSourceRect(frameSource.left, leftImage.width, leftImage.height);
    const auto rightRect = stereo
        ? ResolveSourceRect(frameSource.right, rightImage.width, rightImage.height)
        : leftRect;
    if (!leftRect.has_value() || !rightRect.has_value())
    {
        return false;
    }

    const uint32_t eyeWidth = stereo ? outputWidth / 2u : outputWidth;
    const uint32_t uvSampleCount = (outputWidth / 2u) * (outputHeight / 2u);
    std::vector<uint32_t> uAccum(uvSampleCount, 0);
    std::vector<uint32_t> vAccum(uvSampleCount, 0);
    std::vector<uint32_t> uvCount(uvSampleCount, 0);

    if (!BlitEyeToNv12(leftImage, *leftRect, 0, eyeWidth, output, uAccum, vAccum, uvCount))
    {
        return false;
    }
    if (stereo &&
        !BlitEyeToNv12(rightImage, *rightRect, eyeWidth, eyeWidth, output, uAccum, vAccum, uvCount))
    {
        return false;
    }

    return FinalizeNv12Chroma(output, uAccum, vAccum, uvCount);
}
