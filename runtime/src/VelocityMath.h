// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <openxr/openxr.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * Pure math behind XrSpaceVelocity reporting, kept free of Runtime/Session/socket state so it is
 * directly testable.
 *
 * Two layers use it:
 *   - TrackingReceiver::GetRawControllerVelocity — finite-differences two raw tracking samples.
 *   - Space::LocateSpace — changes that world velocity into the base space's frame and sets the
 *     valid bits.
 *
 * The finite difference is deliberately a plain two-point difference over the two most recent
 * samples: it reports the instantaneous peak. Do NOT "improve" this with smoothing, an average
 * over more samples, or an exponential filter. Any of those clip exactly the spike that
 * velocity-based game mechanics test for — punch detection, throw velocity — and clip it harder
 * the faster the motion, which is the failure this feature exists to fix.
 */
namespace oxrsys::velocity
{

// Below this the pair is a duplicate / same-instant sample and the interval is meaningless.
inline constexpr double kMinSampleIntervalSeconds = 1e-4;
// Above this the pair straddles dropped packets, so the difference is not an instantaneous
// velocity any more.
inline constexpr double kMaxSampleIntervalSeconds = 0.1;

/// True when a sample pair is far enough apart to differentiate, but close enough to still mean
/// "now". A false here must produce no velocity at all, never a fabricated one.
inline bool IsUsableSampleInterval(double dtSeconds)
{
    return dtSeconds > kMinSampleIntervalSeconds && dtSeconds <= kMaxSampleIntervalSeconds;
}

/// Undamped two-point linear velocity (m/s).
inline glm::vec3 FiniteDifferenceLinearVelocity(const glm::vec3& newerPosition,
                                                const glm::vec3& olderPosition,
                                                double dtSeconds)
{
    return (newerPosition - olderPosition) / static_cast<float>(dtSeconds);
}

/// Undamped two-point angular velocity (rad/s) from the delta rotation
/// q_delta = q_new * inverse(q_old), taken as an axis-angle over dt along the shortest arc.
inline glm::vec3 FiniteDifferenceAngularVelocity(const glm::quat& newerOrientation,
                                                 const glm::quat& olderOrientation,
                                                 double dtSeconds)
{
    glm::quat delta = glm::normalize(glm::normalize(newerOrientation) *
                                     glm::inverse(glm::normalize(olderOrientation)));
    // A quaternion and its negation are the same rotation; the negative-w one describes the long
    // way round. Flip it so the angle below is the shortest arc in [0, pi] rather than its
    // complement, which would report a large backwards spin for a small forward one.
    if (delta.w < 0.0f)
    {
        delta = -delta;
    }

    const glm::vec3 axis = glm::axis(delta);
    const float angle = glm::angle(delta); // [0, pi]
    if (angle <= 1e-5f || glm::length(axis) <= 1e-5f)
    {
        return glm::vec3(0.0f);
    }
    return glm::normalize(axis) * (angle / static_cast<float>(dtSeconds));
}

/**
 * Fill `velocity` for an xrLocateSpace call.
 *
 * @param haveWorldVelocity true when a real world-frame velocity is known for the located space.
 * @param posesActive       true when both the located pose and the base pose are tracked.
 * @param baseOrientation   world orientation of the base space.
 * @param worldLinear/worldAngular located space velocity, world frame (m/s, rad/s).
 * @param baseLinear/baseAngular   base space velocity, world frame (zero for a static base).
 *
 * @return true when a real velocity was reported (both valid bits set). When false the struct is
 *         left fully zeroed with velocityFlags == 0 — the spec's "no velocity data", which is
 *         deliberately what an app must see rather than a fabricated number.
 */
inline bool FillRelativeVelocity(XrSpaceVelocity& velocity,
                                 bool haveWorldVelocity,
                                 bool posesActive,
                                 const glm::quat& baseOrientation,
                                 const glm::vec3& worldLinear,
                                 const glm::vec3& worldAngular,
                                 const glm::vec3& baseLinear,
                                 const glm::vec3& baseAngular)
{
    if (!haveWorldVelocity || !posesActive)
    {
        velocity.velocityFlags = 0;
        velocity.linearVelocity = {0.0f, 0.0f, 0.0f};
        velocity.angularVelocity = {0.0f, 0.0f, 0.0f};
        return false;
    }

    // Change the world velocity into the base space's frame. For a static base (LOCAL/STAGE — the
    // usual case for input) this is an exact change of basis, so the reported speed magnitude
    // equals the true controller speed: no damping is introduced here either.
    const glm::quat baseRotInv = glm::inverse(glm::normalize(baseOrientation));
    const glm::vec3 relLinear = baseRotInv * (worldLinear - baseLinear);
    const glm::vec3 relAngular = baseRotInv * (worldAngular - baseAngular);

    velocity.linearVelocity = {relLinear.x, relLinear.y, relLinear.z};
    velocity.angularVelocity = {relAngular.x, relAngular.y, relAngular.z};
    velocity.velocityFlags =
        XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    return true;
}

} // namespace oxrsys::velocity
