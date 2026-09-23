// SPDX-License-Identifier: MPL-2.0
//
// The OpenXR -> compositor translation for XrCompositionLayerQuad. Kept out of
// Session.cpp so the flag and enum mappings are unit-testable on their own:
// getting XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT's meaning wrong is
// invisible in a screenshot but obvious in a table of expectations.

#pragma once

#include <openxr/openxr.h>

#include "GraphicsTypes.h"

namespace oxrsys::runtime
{

inline FrameEyeVisibility ToFrameEyeVisibility(XrEyeVisibility visibility)
{
    switch (visibility)
    {
        case XR_EYE_VISIBILITY_LEFT:
            return FrameEyeVisibility::Left;
        case XR_EYE_VISIBILITY_RIGHT:
            return FrameEyeVisibility::Right;
        case XR_EYE_VISIBILITY_BOTH:
        default:
            return FrameEyeVisibility::Both;
    }
}

// XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT only has meaning alongside
// XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT: without the source-alpha
// bit the spec says the texture's alpha channel is ignored and the layer is
// composited as opaque, whatever the alpha bit says.
inline FrameQuadBlend ToFrameQuadBlend(XrCompositionLayerFlags layerFlags)
{
    if ((layerFlags & XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT) == 0)
    {
        return FrameQuadBlend::Opaque;
    }
    if ((layerFlags & XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT) != 0)
    {
        return FrameQuadBlend::UnpremultipliedAlpha;
    }
    return FrameQuadBlend::PremultipliedAlpha;
}

} // namespace oxrsys::runtime
