// SPDX-License-Identifier: MPL-2.0
//
// Coverage for controller XrSpaceVelocity reporting.
//
// Before this feature the runtime always answered xrLocateSpace with velocityFlags = 0 and zeroed
// vectors. That is spec-legal ("no velocity data"), but games that read XrSpaceVelocity instead of
// differentiating poses themselves saw no motion at all — boxing titles such as UNDERDOGS never
// registered a fast punch, because hand *positions* were right while the reported *speed* was
// always zero.
//
// Two properties these tests exist to protect:
//   1. The velocity is UNDAMPED. It is a two-point difference over the two newest samples, so it
//      reports the instantaneous peak. Smoothing would clip exactly the spike a punch test looks
//      for, and clip it harder the faster the punch. "TrackingReceiver — undamped" below fails if
//      anyone introduces a filter.
//   2. When the velocity is not known, NO velocity is reported. Fabricating a number is worse than
//      reporting none, so every negative path is asserted explicitly, not just left unchecked.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "InputManager.h"
#include "TrackingReceiver.h"
#include "VelocityMath.h"

#include <cmath>
#include <cstring>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace
{

constexpr int64_t kMsNs = 1'000'000;

void SetVec3(float* dst, float x, float y, float z)
{
    dst[0] = x;
    dst[1] = y;
    dst[2] = z;
}

void SetQuat(float* dst, const glm::quat& q)
{
    dst[0] = q.x;
    dst[1] = q.y;
    dst[2] = q.z;
    dst[3] = q.w;
}

// A tracking packet with an active left controller at the given position/orientation.
oxr::protocol::TrackingPacket MakeLeftControllerPacket(int64_t timestampNs, glm::vec3 position,
                                                       glm::quat orientation = glm::quat(1, 0, 0, 0))
{
    oxr::protocol::TrackingPacket packet = {};
    packet.timestampNs = timestampNs;
    packet.headOrientation[3] = 1.0f;
    packet.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE;
    SetVec3(packet.leftControllerPos, position.x, position.y, position.z);
    SetQuat(packet.leftControllerRot, orientation);
    SetQuat(packet.rightControllerRot, glm::quat(1, 0, 0, 0));
    return packet;
}

void Inject(TrackingReceiver& receiver, const oxr::protocol::TrackingPacket& packet)
{
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

// A velocity struct pre-filled with garbage, so the negative paths must be seen to clear it rather
// than merely to leave it alone.
XrSpaceVelocity DirtyVelocity()
{
    XrSpaceVelocity velocity{};
    velocity.type = XR_TYPE_SPACE_VELOCITY;
    velocity.velocityFlags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT |
                             XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    velocity.linearVelocity = {9.0f, 9.0f, 9.0f};
    velocity.angularVelocity = {9.0f, 9.0f, 9.0f};
    return velocity;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Pure finite-difference math
// ---------------------------------------------------------------------------------------------

TEST_CASE("Velocity math — linear finite difference over a known pair", "[velocity]")
{
    // 30mm right, 20mm down, 50mm forward in exactly 10ms -> 3, -2, 5 m/s. Chosen so the
    // arithmetic is unambiguous by inspection.
    const glm::vec3 velocity = oxrsys::velocity::FiniteDifferenceLinearVelocity(
        glm::vec3(0.130f, 1.280f, -0.350f), glm::vec3(0.100f, 1.300f, -0.400f), 0.010);

    CHECK_THAT(velocity.x, WithinAbs(3.0f, 1e-4f));
    CHECK_THAT(velocity.y, WithinAbs(-2.0f, 1e-4f));
    CHECK_THAT(velocity.z, WithinAbs(5.0f, 1e-4f));
}

TEST_CASE("Velocity math — linear finite difference direction follows the newer sample",
          "[velocity]")
{
    // Sign check: reversing the pair must reverse the reported velocity, never repeat it.
    const glm::vec3 forward = oxrsys::velocity::FiniteDifferenceLinearVelocity(
        glm::vec3(0.0f, 0.0f, 0.1f), glm::vec3(0.0f, 0.0f, 0.0f), 0.010);
    const glm::vec3 backward = oxrsys::velocity::FiniteDifferenceLinearVelocity(
        glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.1f), 0.010);

    CHECK_THAT(forward.z, WithinAbs(10.0f, 1e-4f));
    CHECK_THAT(backward.z, WithinAbs(-10.0f, 1e-4f));
}

TEST_CASE("Velocity math — angular finite difference over a known delta quaternion", "[velocity]")
{
    const glm::quat older = glm::angleAxis(0.05f, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat newer = glm::angleAxis(0.25f, glm::vec3(0.0f, 1.0f, 0.0f));

    SECTION("0.2 rad of yaw in 10ms is 20 rad/s about +Y")
    {
        const glm::vec3 angular =
            oxrsys::velocity::FiniteDifferenceAngularVelocity(newer, older, 0.010);
        CHECK_THAT(angular.x, WithinAbs(0.0f, 1e-3f));
        CHECK_THAT(angular.y, WithinAbs(20.0f, 1e-3f));
        CHECK_THAT(angular.z, WithinAbs(0.0f, 1e-3f));
    }

    SECTION("Rotating the other way reports the opposite sign")
    {
        const glm::vec3 angular =
            oxrsys::velocity::FiniteDifferenceAngularVelocity(older, newer, 0.010);
        CHECK_THAT(angular.y, WithinAbs(-20.0f, 1e-3f));
    }

    SECTION("No rotation reports zero angular velocity")
    {
        const glm::vec3 angular =
            oxrsys::velocity::FiniteDifferenceAngularVelocity(older, older, 0.010);
        CHECK_THAT(glm::length(angular), WithinAbs(0.0f, 1e-4f));
    }

    SECTION("Angular velocity scales with 1/dt, not with sample count")
    {
        const glm::vec3 fast =
            oxrsys::velocity::FiniteDifferenceAngularVelocity(newer, older, 0.005);
        CHECK_THAT(fast.y, WithinAbs(40.0f, 1e-3f));
    }
}

TEST_CASE("Velocity math — angular finite difference takes the shortest arc", "[velocity]")
{
    // q and -q are the same rotation, but -q describes the long way round. A delta with w < 0 must
    // be negated first, otherwise a small forward turn is reported as a large backwards spin.
    const glm::quat older(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::quat newer = glm::angleAxis(0.2f, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat newerNegated = -newer; // delta.w = -cos(0.1) < 0

    const glm::vec3 direct =
        oxrsys::velocity::FiniteDifferenceAngularVelocity(newer, older, 0.010);
    const glm::vec3 negated =
        oxrsys::velocity::FiniteDifferenceAngularVelocity(newerNegated, older, 0.010);

    SECTION("The negated-hemisphere input gives the same answer as the direct one")
    {
        CHECK_THAT(negated.x, WithinAbs(direct.x, 1e-3f));
        CHECK_THAT(negated.y, WithinAbs(direct.y, 1e-3f));
        CHECK_THAT(negated.z, WithinAbs(direct.z, 1e-3f));
    }

    SECTION("And it is the short arc, not its 2*pi complement")
    {
        // Short arc: 0.2 rad / 0.01s = 20 rad/s. Long way round would be (2*pi - 0.2)/0.01,
        // roughly -608 rad/s — a fabricated spin an app would read as a violent flick.
        CHECK_THAT(negated.y, WithinAbs(20.0f, 1e-3f));
        CHECK(glm::length(negated) < 100.0f);
    }
}

TEST_CASE("Velocity math — sample interval guards", "[velocity]")
{
    SECTION("A usable interval is accepted")
    {
        CHECK(oxrsys::velocity::IsUsableSampleInterval(0.010));  // 90Hz-ish streaming
        CHECK(oxrsys::velocity::IsUsableSampleInterval(0.100));  // exactly the stale bound
    }

    SECTION("Duplicate / same-instant pairs are rejected")
    {
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(0.0));
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(1e-4));  // exactly the lower bound
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(5e-5));
    }

    SECTION("Backwards pairs are rejected")
    {
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(-0.010));
    }

    SECTION("Stale pairs spanning dropped packets are rejected")
    {
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(0.1001));
        CHECK_FALSE(oxrsys::velocity::IsUsableSampleInterval(0.5));
    }
}

// ---------------------------------------------------------------------------------------------
// TrackingReceiver::GetRawControllerVelocity — history walking and guards
// ---------------------------------------------------------------------------------------------

TEST_CASE("TrackingReceiver — raw controller velocity from a known sample pair", "[velocity]")
{
    TrackingReceiver receiver;

    // 40mm of +X travel and 0.2 rad of yaw in exactly 10ms -> 4 m/s and 20 rad/s.
    Inject(receiver, MakeLeftControllerPacket(1'000 * kMsNs, glm::vec3(0.100f, 1.300f, -0.400f),
                                              glm::angleAxis(0.00f, glm::vec3(0, 1, 0))));
    Inject(receiver, MakeLeftControllerPacket(1'010 * kMsNs, glm::vec3(0.140f, 1.300f, -0.400f),
                                              glm::angleAxis(0.20f, glm::vec3(0, 1, 0))));

    glm::vec3 linear(0.0f);
    glm::vec3 angular(0.0f);
    REQUIRE(receiver.GetRawControllerVelocity(true, linear, angular));

    CHECK_THAT(linear.x, WithinAbs(4.0f, 1e-3f));
    CHECK_THAT(linear.y, WithinAbs(0.0f, 1e-3f));
    CHECK_THAT(linear.z, WithinAbs(0.0f, 1e-3f));
    CHECK_THAT(angular.y, WithinAbs(20.0f, 1e-2f));
}

TEST_CASE("TrackingReceiver — velocity is undamped (the UNDERDOGS punch case)", "[velocity]")
{
    TrackingReceiver receiver;

    // A hand drifting slowly, then thrown forward: the last 10ms cover 0.5m/s -> 5.0 m/s... a
    // punch. A two-point difference reports the full 5 m/s. Any smoothing or averaging over the
    // older, slow samples would report something far lower, and a boxing game would score the
    // punch as a nudge. This test is the tripwire for that.
    Inject(receiver, MakeLeftControllerPacket(2'000 * kMsNs, glm::vec3(0.0f, 1.3f, -0.400f)));
    Inject(receiver, MakeLeftControllerPacket(2'010 * kMsNs, glm::vec3(0.0f, 1.3f, -0.405f)));
    Inject(receiver, MakeLeftControllerPacket(2'020 * kMsNs, glm::vec3(0.0f, 1.3f, -0.410f)));
    Inject(receiver, MakeLeftControllerPacket(2'030 * kMsNs, glm::vec3(0.0f, 1.3f, -0.460f)));

    glm::vec3 linear(0.0f);
    glm::vec3 angular(0.0f);
    REQUIRE(receiver.GetRawControllerVelocity(true, linear, angular));

    const float speed = glm::length(linear);

    SECTION("The instantaneous peak is reported in full")
    {
        CHECK_THAT(linear.z, WithinAbs(-5.0f, 1e-3f));
        CHECK_THAT(speed, WithinAbs(5.0f, 1e-3f));
    }

    SECTION("It is not an average over the older, slower samples")
    {
        // Mean speed across all four samples is (0.005+0.005+0.050)/0.030 = 2.0 m/s. Anything at
        // or below that means a filter crept in and the punch spike has been clipped.
        CHECK(speed > 4.5f);
    }
}

TEST_CASE("TrackingReceiver — no velocity without a usable sample pair", "[velocity]")
{
    glm::vec3 linear(0.0f);
    glm::vec3 angular(0.0f);

    SECTION("No samples at all")
    {
        TrackingReceiver receiver;
        CHECK_FALSE(receiver.GetRawControllerVelocity(true, linear, angular));
    }

    SECTION("Only one sample — nothing to differentiate against")
    {
        TrackingReceiver receiver;
        Inject(receiver, MakeLeftControllerPacket(3'000 * kMsNs, glm::vec3(0.1f, 1.3f, -0.4f)));
        CHECK_FALSE(receiver.GetRawControllerVelocity(true, linear, angular));
    }

    SECTION("Near-duplicate timestamps (50us apart) are below the differentiation floor")
    {
        TrackingReceiver receiver;
        Inject(receiver, MakeLeftControllerPacket(3'000 * kMsNs, glm::vec3(0.100f, 1.3f, -0.4f)));
        Inject(receiver, MakeLeftControllerPacket(3'000 * kMsNs + 50'000,
                                                  glm::vec3(0.140f, 1.3f, -0.4f)));
        CHECK_FALSE(receiver.GetRawControllerVelocity(true, linear, angular));
    }

    SECTION("A stale pair spanning 200ms of dropped packets is rejected")
    {
        TrackingReceiver receiver;
        Inject(receiver, MakeLeftControllerPacket(4'000 * kMsNs, glm::vec3(0.100f, 1.3f, -0.4f)));
        Inject(receiver, MakeLeftControllerPacket(4'200 * kMsNs, glm::vec3(0.140f, 1.3f, -0.4f)));
        CHECK_FALSE(receiver.GetRawControllerVelocity(true, linear, angular));
    }

    SECTION("The other hand has no samples of its own")
    {
        TrackingReceiver receiver;
        Inject(receiver, MakeLeftControllerPacket(5'000 * kMsNs, glm::vec3(0.100f, 1.3f, -0.4f)));
        Inject(receiver, MakeLeftControllerPacket(5'010 * kMsNs, glm::vec3(0.140f, 1.3f, -0.4f)));
        CHECK(receiver.GetRawControllerVelocity(true, linear, angular));
        CHECK_FALSE(receiver.GetRawControllerVelocity(false, linear, angular));
    }
}

TEST_CASE("TrackingReceiver — a broken active chain reports no velocity", "[velocity]")
{
    TrackingReceiver receiver;

    // Active, then tracking lost, then active again some distance away. Differencing across the
    // gap would invent a huge velocity out of a teleport the user never performed, so the pair
    // must be refused instead.
    auto first = MakeLeftControllerPacket(6'000 * kMsNs, glm::vec3(0.10f, 1.3f, -0.40f));

    auto lost = MakeLeftControllerPacket(6'010 * kMsNs, glm::vec3(0.10f, 1.3f, -0.40f));
    lost.trackingFlags = 0; // controller went inactive

    auto reacquired = MakeLeftControllerPacket(6'020 * kMsNs, glm::vec3(0.90f, 1.3f, -0.40f));

    Inject(receiver, first);
    Inject(receiver, lost);
    Inject(receiver, reacquired);

    glm::vec3 linear(1.0f);
    glm::vec3 angular(1.0f);
    CHECK_FALSE(receiver.GetRawControllerVelocity(true, linear, angular));
}

TEST_CASE("TrackingReceiver — velocity is per hand", "[velocity]")
{
    TrackingReceiver receiver;

    auto first = MakeLeftControllerPacket(7'000 * kMsNs, glm::vec3(0.10f, 1.3f, -0.40f));
    first.trackingFlags |= oxr::protocol::TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE;
    SetVec3(first.rightControllerPos, 0.30f, 1.3f, -0.40f);

    auto second = MakeLeftControllerPacket(7'010 * kMsNs, glm::vec3(0.12f, 1.3f, -0.40f));
    second.trackingFlags |= oxr::protocol::TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE;
    SetVec3(second.rightControllerPos, 0.30f, 1.3f, -0.45f);

    Inject(receiver, first);
    Inject(receiver, second);

    glm::vec3 leftLinear(0.0f);
    glm::vec3 rightLinear(0.0f);
    glm::vec3 angular(0.0f);
    REQUIRE(receiver.GetRawControllerVelocity(true, leftLinear, angular));
    REQUIRE(receiver.GetRawControllerVelocity(false, rightLinear, angular));

    CHECK_THAT(leftLinear.x, WithinAbs(2.0f, 1e-3f));
    CHECK_THAT(leftLinear.z, WithinAbs(0.0f, 1e-3f));
    CHECK_THAT(rightLinear.x, WithinAbs(0.0f, 1e-3f));
    CHECK_THAT(rightLinear.z, WithinAbs(-5.0f, 1e-3f));
}

// ---------------------------------------------------------------------------------------------
// InputManager forwarding
// ---------------------------------------------------------------------------------------------

TEST_CASE("InputManager — controller velocity requires a tracking receiver", "[velocity]")
{
    InputManager im;

    glm::vec3 linear(7.0f);
    glm::vec3 angular(7.0f);
    CHECK_FALSE(im.GetControllerVelocity(InputManager::Hand::Left, linear, angular));
}

TEST_CASE("InputManager — controller velocity forwards to the receiver per hand", "[velocity]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);

    Inject(receiver, MakeLeftControllerPacket(8'000 * kMsNs, glm::vec3(0.100f, 1.3f, -0.40f)));
    Inject(receiver, MakeLeftControllerPacket(8'010 * kMsNs, glm::vec3(0.130f, 1.3f, -0.40f)));

    glm::vec3 linear(0.0f);
    glm::vec3 angular(0.0f);

    SECTION("Left hand maps to the left controller samples")
    {
        REQUIRE(im.GetControllerVelocity(InputManager::Hand::Left, linear, angular));
        CHECK_THAT(linear.x, WithinAbs(3.0f, 1e-3f));
    }

    SECTION("Right hand has no samples and reports nothing")
    {
        CHECK_FALSE(im.GetControllerVelocity(InputManager::Hand::Right, linear, angular));
    }
}

// ---------------------------------------------------------------------------------------------
// Space::LocateSpace — base-space transform and velocity flags
// ---------------------------------------------------------------------------------------------

TEST_CASE("Space velocity — reported in the base space's frame", "[velocity]")
{
    const glm::vec3 worldLinear(0.0f, 0.0f, -5.0f); // a 5 m/s punch straight ahead
    const glm::vec3 worldAngular(0.0f, 20.0f, 0.0f);

    SECTION("A static base reports the true speed unchanged")
    {
        XrSpaceVelocity velocity = DirtyVelocity();
        REQUIRE(oxrsys::velocity::FillRelativeVelocity(
            velocity, true, true, glm::quat(1, 0, 0, 0), worldLinear, worldAngular,
            glm::vec3(0.0f), glm::vec3(0.0f)));

        CHECK_THAT(velocity.linearVelocity.z, WithinAbs(-5.0f, 1e-4f));
        const float speed = std::sqrt(velocity.linearVelocity.x * velocity.linearVelocity.x +
                                      velocity.linearVelocity.y * velocity.linearVelocity.y +
                                      velocity.linearVelocity.z * velocity.linearVelocity.z);
        CHECK_THAT(speed, WithinAbs(5.0f, 1e-4f));
        CHECK_THAT(velocity.angularVelocity.y, WithinAbs(20.0f, 1e-4f));
    }

    SECTION("A base yawed 90 degrees re-expresses the vector without changing its magnitude")
    {
        // Base yawed +90 deg about +Y. Expressed in that base's frame, a world -Z velocity
        // points along +X.
        const glm::quat baseRot = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 1, 0));

        XrSpaceVelocity velocity = DirtyVelocity();
        REQUIRE(oxrsys::velocity::FillRelativeVelocity(velocity, true, true, baseRot, worldLinear,
                                                       worldAngular, glm::vec3(0.0f),
                                                       glm::vec3(0.0f)));

        const glm::vec3 expected = glm::inverse(baseRot) * worldLinear;
        CHECK_THAT(velocity.linearVelocity.x, WithinAbs(expected.x, 1e-4f));
        CHECK_THAT(velocity.linearVelocity.y, WithinAbs(expected.y, 1e-4f));
        CHECK_THAT(velocity.linearVelocity.z, WithinAbs(expected.z, 1e-4f));
        CHECK_THAT(velocity.linearVelocity.x, WithinAbs(5.0f, 1e-4f));
        CHECK_THAT(velocity.linearVelocity.z, WithinAbs(0.0f, 1e-4f));

        // A change of basis is a rotation: the speed the game reads is still the true speed.
        const float speed = std::sqrt(velocity.linearVelocity.x * velocity.linearVelocity.x +
                                      velocity.linearVelocity.y * velocity.linearVelocity.y +
                                      velocity.linearVelocity.z * velocity.linearVelocity.z);
        CHECK_THAT(speed, WithinAbs(glm::length(worldLinear), 1e-4f));

        // The angular velocity rides along the same change of basis.
        const glm::vec3 expectedAngular = glm::inverse(baseRot) * worldAngular;
        CHECK_THAT(velocity.angularVelocity.y, WithinAbs(expectedAngular.y, 1e-4f));
    }

    SECTION("A moving base is subtracted, so the velocity is relative to it")
    {
        XrSpaceVelocity velocity = DirtyVelocity();
        REQUIRE(oxrsys::velocity::FillRelativeVelocity(
            velocity, true, true, glm::quat(1, 0, 0, 0), worldLinear, worldAngular,
            glm::vec3(0.0f, 0.0f, -2.0f), glm::vec3(0.0f, 5.0f, 0.0f)));

        CHECK_THAT(velocity.linearVelocity.z, WithinAbs(-3.0f, 1e-4f));
        CHECK_THAT(velocity.angularVelocity.y, WithinAbs(15.0f, 1e-4f));
    }
}

TEST_CASE("Space velocity — valid bits are set only for a real velocity", "[velocity]")
{
    const XrSpaceVelocityFlags bothBits =
        XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;

    SECTION("Both valid bits are set when the velocity is real")
    {
        XrSpaceVelocity velocity = DirtyVelocity();
        velocity.velocityFlags = 0;
        REQUIRE(oxrsys::velocity::FillRelativeVelocity(
            velocity, true, true, glm::quat(1, 0, 0, 0), glm::vec3(1.0f, 2.0f, 3.0f),
            glm::vec3(0.5f), glm::vec3(0.0f), glm::vec3(0.0f)));

        CHECK((velocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0);
        CHECK((velocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) != 0);
        CHECK(velocity.velocityFlags == bothBits);
    }

    // The negative cases matter more than the positive one: reporting a fabricated velocity is
    // worse for a game than reporting none, so the flags must be cleared AND the vectors zeroed.
    SECTION("No known velocity reports nothing, with the struct zeroed")
    {
        XrSpaceVelocity velocity = DirtyVelocity();
        CHECK_FALSE(oxrsys::velocity::FillRelativeVelocity(
            velocity, /*haveWorldVelocity=*/false, true, glm::quat(1, 0, 0, 0),
            glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(4.0f), glm::vec3(0.0f), glm::vec3(0.0f)));

        CHECK(velocity.velocityFlags == 0);
        CHECK(velocity.linearVelocity.x == 0.0f);
        CHECK(velocity.linearVelocity.y == 0.0f);
        CHECK(velocity.linearVelocity.z == 0.0f);
        CHECK(velocity.angularVelocity.x == 0.0f);
        CHECK(velocity.angularVelocity.y == 0.0f);
        CHECK(velocity.angularVelocity.z == 0.0f);
    }

    SECTION("An untracked pose reports nothing even when a velocity is known")
    {
        XrSpaceVelocity velocity = DirtyVelocity();
        CHECK_FALSE(oxrsys::velocity::FillRelativeVelocity(
            velocity, true, /*posesActive=*/false, glm::quat(1, 0, 0, 0),
            glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(4.0f), glm::vec3(0.0f), glm::vec3(0.0f)));

        CHECK(velocity.velocityFlags == 0);
        CHECK(velocity.linearVelocity.z == 0.0f);
        CHECK(velocity.angularVelocity.z == 0.0f);
    }
}

TEST_CASE("Space velocity — end to end from tracking samples to XrSpaceVelocity", "[velocity]")
{
    // The whole path a boxing game exercises: raw samples in, XrSpaceVelocity out.
    TrackingReceiver receiver;
    InputManager im;
    im.SetTrackingReceiver(&receiver);

    Inject(receiver, MakeLeftControllerPacket(9'000 * kMsNs, glm::vec3(0.0f, 1.3f, -0.400f)));
    Inject(receiver, MakeLeftControllerPacket(9'010 * kMsNs, glm::vec3(0.0f, 1.3f, -0.460f)));

    glm::vec3 linear(0.0f);
    glm::vec3 angular(0.0f);
    REQUIRE(im.GetControllerVelocity(InputManager::Hand::Left, linear, angular));

    XrSpaceVelocity velocity = DirtyVelocity();
    REQUIRE(oxrsys::velocity::FillRelativeVelocity(velocity, true, true, glm::quat(1, 0, 0, 0),
                                                   linear, angular, glm::vec3(0.0f),
                                                   glm::vec3(0.0f)));

    // 60mm in 10ms is a 6 m/s hand — a punch, and the app must see all of it.
    CHECK_THAT(velocity.linearVelocity.z, WithinAbs(-6.0f, 1e-3f));
    CHECK(velocity.velocityFlags == (XR_SPACE_VELOCITY_LINEAR_VALID_BIT |
                                     XR_SPACE_VELOCITY_ANGULAR_VALID_BIT));
}
