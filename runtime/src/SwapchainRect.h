// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <openxr/openxr.h>

#include "GraphicsTypes.h"

// Carry XrSwapchainSubImage::imageRect across into the graphics-side plain
// struct. The rect is stored verbatim here; consumers resolve it against the
// real image size with FrameImageSource::GetRect().
inline FrameImageRect FrameImageRectFromSubImage(const XrSwapchainSubImage& subImage)
{
    FrameImageRect rect = {};
    rect.offsetX = subImage.imageRect.offset.x;
    rect.offsetY = subImage.imageRect.offset.y;
    rect.width = subImage.imageRect.extent.width;
    rect.height = subImage.imageRect.extent.height;
    return rect;
}
