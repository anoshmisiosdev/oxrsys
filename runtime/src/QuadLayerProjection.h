// SPDX-License-Identifier: MPL-2.0
//
// Projection maths for XR_TYPE_COMPOSITION_LAYER_QUAD.
//
// Deliberately free of both OpenXR and Metal: a quad's four corners are
// transformed to clip space here, on the CPU, and the renderer only has to
// rasterise them. That keeps the half of quad compositing that is easy to get
// wrong (handedness, the asymmetric FOV, the UV origin) unit-testable without a
// GPU, so a failing pixel test can be blamed on the shader rather than on the
// matrices.

#pragma once

#include "GraphicsTypes.h"

#include <array>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace oxrsys::quad
{

// A homogeneous clip-space position. Kept as plain floats so the renderer can
// upload the array straight to a Metal vertex buffer.
struct ClipPosition
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;
};

// Quad corners, in the order the renderer draws them as a triangle strip:
// 0 = upper-left, 1 = lower-left, 2 = upper-right, 3 = lower-right, where
// "upper" is +Y in the quad's own frame. OpenXR maps the subImage rect's
// top-left texel to the quad's upper-left corner, so the renderer pairs these
// with UVs (0,0) (0,1) (1,0) (1,1) after remapping into the subImage rect.
struct QuadCorners
{
    std::array<ClipPosition, 4> positions = {};
    // False when any corner is non-finite -- a degenerate view or quad pose
    // that the renderer must skip rather than feed to the rasteriser.
    bool valid = false;
};

inline glm::quat ToGlmQuat(const FramePose& pose)
{
    // glm::quat's scalar-first constructor; FramePose stores x,y,z,w.
    return glm::quat(pose.orientation[3], pose.orientation[0], pose.orientation[1],
                     pose.orientation[2]);
}

inline glm::vec3 ToGlmVec3(const FramePose& pose)
{
    return glm::vec3(pose.position[0], pose.position[1], pose.position[2]);
}

inline bool IsFinitePose(const FramePose& pose)
{
    for (float value : pose.orientation)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    for (float value : pose.position)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    const float lengthSquared = pose.orientation[0] * pose.orientation[0] +
                                pose.orientation[1] * pose.orientation[1] +
                                pose.orientation[2] * pose.orientation[2] +
                                pose.orientation[3] * pose.orientation[3];
    return lengthSquared > 1.0e-6f;
}

// Rigid composition: `child` expressed in `parent`'s frame, lifted into
// whatever frame `parent` is expressed in.
inline FramePose ComposePose(const FramePose& parent, const FramePose& child)
{
    const glm::quat parentRotation = ToGlmQuat(parent);
    const glm::quat rotation = parentRotation * ToGlmQuat(child);
    const glm::vec3 position = ToGlmVec3(parent) + parentRotation * ToGlmVec3(child);

    FramePose result = {};
    result.orientation[0] = rotation.x;
    result.orientation[1] = rotation.y;
    result.orientation[2] = rotation.z;
    result.orientation[3] = rotation.w;
    result.position[0] = position.x;
    result.position[1] = position.y;
    result.position[2] = position.z;
    return result;
}

// Where XR_REFERENCE_SPACE_TYPE_VIEW sat at render time, in the projection
// layer's space: midway between the two submitted eye poses, which is exactly
// how the spec defines it.
//
// This matters for head-locked quads -- HUDs, menus, loading screens. Locating
// VIEW space the ordinary way would use the *latest predicted* head pose, one
// frame newer than the pose the eye images were rendered with, and the quad
// would swim against the scene under head motion. Deriving it from the
// submitted views instead pins it to the same instant as the pixels.
inline bool MakeRenderViewPose(const FrameEyeView views[2], FramePose& outPose)
{
    if (!views[0].valid || !views[1].valid || !IsFinitePose(views[0].pose) ||
        !IsFinitePose(views[1].pose))
    {
        return false;
    }

    // Both eyes share an orientation in any sane stereo submission; take the
    // left eye's and average the positions.
    outPose.orientation[0] = views[0].pose.orientation[0];
    outPose.orientation[1] = views[0].pose.orientation[1];
    outPose.orientation[2] = views[0].pose.orientation[2];
    outPose.orientation[3] = views[0].pose.orientation[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        outPose.position[axis] =
            (views[0].pose.position[axis] + views[1].pose.position[axis]) * 0.5f;
    }
    return true;
}

// An OpenXR asymmetric-frustum projection for a clip space whose depth range is
// [0, 1] -- Metal's convention, and D3D's. Right-handed, looking down -Z.
inline glm::mat4 MakeProjection(const FrameFov& fov, float nearZ = 0.01f, float farZ = 1000.0f)
{
    const float tanLeft = std::tan(fov.angleLeft);
    const float tanRight = std::tan(fov.angleRight);
    const float tanUp = std::tan(fov.angleUp);
    const float tanDown = std::tan(fov.angleDown);

    const float tanWidth = tanRight - tanLeft;
    const float tanHeight = tanUp - tanDown;

    glm::mat4 projection(0.0f);
    projection[0][0] = 2.0f / tanWidth;
    projection[1][1] = 2.0f / tanHeight;
    projection[2][0] = (tanRight + tanLeft) / tanWidth;
    projection[2][1] = (tanUp + tanDown) / tanHeight;
    projection[2][2] = -farZ / (farZ - nearZ);
    projection[2][3] = -1.0f;
    projection[3][2] = -(farZ * nearZ) / (farZ - nearZ);
    return projection;
}

// World (reference space) -> view space for an eye pose.
inline glm::mat4 MakeView(const FramePose& pose)
{
    const glm::mat4 rotation = glm::mat4_cast(ToGlmQuat(pose));
    glm::mat4 transform = rotation;
    transform[3] = glm::vec4(ToGlmVec3(pose), 1.0f);
    return glm::inverse(transform);
}

// The quad's four corners in the reference space, in QuadCorners order.
// The quad lies in the XY plane of its own pose, size.width along X and
// size.height along Y, facing +Z -- so a viewer on the quad's +Z side sees its
// front, which is what the OpenXR spec specifies.
inline std::array<glm::vec3, 4> MakeQuadCornersInSpace(const FramePose& pose, float widthMeters,
                                                       float heightMeters)
{
    const glm::quat rotation = ToGlmQuat(pose);
    const glm::vec3 centre = ToGlmVec3(pose);
    const glm::vec3 right = rotation * glm::vec3(widthMeters * 0.5f, 0.0f, 0.0f);
    const glm::vec3 up = rotation * glm::vec3(0.0f, heightMeters * 0.5f, 0.0f);

    return {
        centre - right + up,  // upper-left
        centre - right - up,  // lower-left
        centre + right + up,  // upper-right
        centre + right - up,  // lower-right
    };
}

// Project a quad for one eye. `view` and `quad.pose` must already be expressed
// in the same reference space; Session.cpp is responsible for relocating the
// quad's space into the projection layer's space before calling this.
inline QuadCorners ProjectQuad(const FrameEyeView& view, const FramePose& quadPose,
                               float widthMeters, float heightMeters)
{
    QuadCorners corners = {};
    if (!view.valid || !IsFinitePose(view.pose) || !IsFinitePose(quadPose) ||
        !std::isfinite(widthMeters) || !std::isfinite(heightMeters) ||
        widthMeters <= 0.0f || heightMeters <= 0.0f)
    {
        return corners;
    }
    if (!std::isfinite(view.fov.angleLeft) || !std::isfinite(view.fov.angleRight) ||
        !std::isfinite(view.fov.angleUp) || !std::isfinite(view.fov.angleDown) ||
        view.fov.angleRight <= view.fov.angleLeft || view.fov.angleUp <= view.fov.angleDown)
    {
        return corners;
    }

    const glm::mat4 viewProjection = MakeProjection(view.fov) * MakeView(view.pose);
    const std::array<glm::vec3, 4> world =
        MakeQuadCornersInSpace(quadPose, widthMeters, heightMeters);

    for (size_t i = 0; i < world.size(); ++i)
    {
        const glm::vec4 clip = viewProjection * glm::vec4(world[i], 1.0f);
        if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) ||
            !std::isfinite(clip.w))
        {
            return QuadCorners{};
        }
        corners.positions[i] = {clip.x, clip.y, clip.z, clip.w};
    }

    corners.valid = true;
    return corners;
}

// Axis-aligned pixel bounds of a projected quad inside an eye image, for tests
// and logging. Only meaningful when every corner is in front of the eye
// (all w > 0); returns false otherwise, because a quad straddling the near
// plane has no finite screen-space rect and must be left to the rasteriser's
// homogeneous clipper.
struct ScreenRect
{
    float minX = 0.0f;
    float minY = 0.0f;
    float maxX = 0.0f;
    float maxY = 0.0f;
};

inline bool ComputeScreenRect(const QuadCorners& corners, uint32_t imageWidth, uint32_t imageHeight,
                              ScreenRect& outRect)
{
    if (!corners.valid || imageWidth == 0 || imageHeight == 0)
    {
        return false;
    }

    bool first = true;
    for (const ClipPosition& position : corners.positions)
    {
        if (!(position.w > 1.0e-6f))
        {
            return false;
        }
        const float ndcX = position.x / position.w;
        const float ndcY = position.y / position.w;
        // Metal (and Vulkan, and D3D) put framebuffer y=0 at the top while NDC
        // +Y points up, so the vertical axis flips here.
        const float pixelX = (ndcX * 0.5f + 0.5f) * static_cast<float>(imageWidth);
        const float pixelY = (0.5f - ndcY * 0.5f) * static_cast<float>(imageHeight);
        if (first)
        {
            outRect.minX = outRect.maxX = pixelX;
            outRect.minY = outRect.maxY = pixelY;
            first = false;
            continue;
        }
        outRect.minX = std::fmin(outRect.minX, pixelX);
        outRect.maxX = std::fmax(outRect.maxX, pixelX);
        outRect.minY = std::fmin(outRect.minY, pixelY);
        outRect.maxY = std::fmax(outRect.maxY, pixelY);
    }
    return true;
}

// The quad's subImage rect as normalised UVs into its swapchain image, so the
// shader samples only the rect the application submitted. Falls back to the
// whole image when no rect was recorded, which is what a swapchain snapshot
// without sub-rect information means.
struct QuadUvRect
{
    float offsetU = 0.0f;
    float offsetV = 0.0f;
    float scaleU = 1.0f;
    float scaleV = 1.0f;
};

inline QuadUvRect MakeUvRect(const FrameImageSource& image, uint32_t textureWidth,
                             uint32_t textureHeight)
{
    QuadUvRect rect = {};
    if (!image.HasSourceRect() || textureWidth == 0 || textureHeight == 0)
    {
        return rect;
    }
    const float width = static_cast<float>(textureWidth);
    const float height = static_cast<float>(textureHeight);
    rect.offsetU = static_cast<float>(image.sourceX) / width;
    rect.offsetV = static_cast<float>(image.sourceY) / height;
    rect.scaleU = static_cast<float>(image.sourceWidth) / width;
    rect.scaleV = static_cast<float>(image.sourceHeight) / height;
    return rect;
}

} // namespace oxrsys::quad
