// SPDX-License-Identifier: MPL-2.0
//
// Maps an eye image rendered with one field of view onto another.
//
// An XrCompositionLayerProjectionView carries the fov its image was rendered with.
// The headset displays each eye with its own fov; when the two differ (an application
// cached an older projection, e.g. the placeholder view reported before a client
// connected), showing the image unchanged shifts and scales it in each eye - up to
// uncrossable double vision. Both are planar projections from the same eye, so the
// correction is exactly affine per axis in tangent space.

#pragma once

#include "GraphicsTypes.h"

#include <cmath>

namespace oxrsys::video
{

// Output UV (u right, v down, over the display fov) -> UV in the submitted image:
//   srcU = u * uScale + uOffset,  srcV = v * vScale + vOffset
// Source UVs outside [0, 1] were never rendered by the application.
struct FovRemap
{
    float uScale = 1.0f;
    float uOffset = 0.0f;
    float vScale = 1.0f;
    float vOffset = 0.0f;
};

inline bool IsUsableFov(const FrameFov& fov)
{
    constexpr float kLimit = 1.55f; // just under 90 degrees, where tan() blows up
    return std::isfinite(fov.angleLeft) && std::isfinite(fov.angleRight) &&
           std::isfinite(fov.angleUp) && std::isfinite(fov.angleDown) &&
           fov.angleLeft > -kLimit && fov.angleRight < kLimit && fov.angleDown > -kLimit &&
           fov.angleUp < kLimit && fov.angleRight > fov.angleLeft && fov.angleUp > fov.angleDown;
}

inline bool FovNeedsRemap(const FrameFov& rendered, const FrameFov& display, float epsilon = 1e-4f)
{
    if (!IsUsableFov(rendered) || !IsUsableFov(display))
    {
        return false;
    }
    return std::fabs(rendered.angleLeft - display.angleLeft) > epsilon ||
           std::fabs(rendered.angleRight - display.angleRight) > epsilon ||
           std::fabs(rendered.angleUp - display.angleUp) > epsilon ||
           std::fabs(rendered.angleDown - display.angleDown) > epsilon;
}

inline FovRemap ComputeFovRemap(const FrameFov& rendered, const FrameFov& display)
{
    const float rl = std::tan(rendered.angleLeft), rr = std::tan(rendered.angleRight);
    const float ru = std::tan(rendered.angleUp), rd = std::tan(rendered.angleDown);
    const float dl = std::tan(display.angleLeft), dr = std::tan(display.angleRight);
    const float du = std::tan(display.angleUp), dd = std::tan(display.angleDown);

    FovRemap remap;
    // Horizontal: tangent grows to the right.
    remap.uScale = (dr - dl) / (rr - rl);
    remap.uOffset = (dl - rl) / (rr - rl);
    // Vertical: v = 0 is the top (angleUp).
    remap.vScale = (dd - du) / (rd - ru);
    remap.vOffset = (du - ru) / (rd - ru);
    return remap;
}

} // namespace oxrsys::video
