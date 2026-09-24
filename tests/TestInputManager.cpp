// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "InputManager.h"
#include "TrackingReceiver.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <glm/gtc/quaternion.hpp>
#include <vector>

using Catch::Matchers::WithinAbs;

namespace
{

void SetLeftHandJoint(oxr::protocol::TrackingPacket& packet, uint32_t joint,
                      float x, float y, float z, float radius = 0.01f)
{
    packet.leftHandJoints[joint][0] = x;
    packet.leftHandJoints[joint][1] = y;
    packet.leftHandJoints[joint][2] = z;
    packet.leftHandJoints[joint][3] = radius;
}

void PopulateLeftPinchingHand(oxr::protocol::TrackingPacket& packet,
                              float palmX, float palmY, float palmZ)
{
    SetLeftHandJoint(packet, XR_HAND_JOINT_PALM_EXT, palmX, palmY, palmZ, 0.025f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_WRIST_EXT, palmX, palmY - 0.08f, palmZ + 0.02f, 0.020f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_INDEX_METACARPAL_EXT,
                     palmX - 0.03f, palmY + 0.01f, palmZ - 0.02f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_LITTLE_METACARPAL_EXT,
                     palmX + 0.03f, palmY + 0.01f, palmZ - 0.02f);

    SetLeftHandJoint(packet, XR_HAND_JOINT_THUMB_TIP_EXT,
                     palmX - 0.008f, palmY + 0.01f, palmZ - 0.04f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_INDEX_TIP_EXT,
                     palmX + 0.007f, palmY + 0.01f, palmZ - 0.04f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_MIDDLE_TIP_EXT,
                     palmX, palmY + 0.01f, palmZ - 0.034f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_RING_TIP_EXT,
                     palmX + 0.010f, palmY + 0.01f, palmZ - 0.034f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_LITTLE_TIP_EXT,
                     palmX + 0.020f, palmY + 0.01f, palmZ - 0.034f);
    SetLeftHandJoint(packet, XR_HAND_JOINT_INDEX_PROXIMAL_EXT,
                     palmX - 0.015f, palmY + 0.01f, palmZ - 0.025f);
}

} // namespace

TEST_CASE("InputManager — initial state", "[input]")
{
    InputManager im;

    SECTION("Head starts at default position")
    {
        XrPosef pose = im.GetHeadPose();
        CHECK_THAT(pose.position.x, WithinAbs(0.0, 0.001));
        CHECK_THAT(pose.position.y, WithinAbs(1.6, 0.001));
        CHECK_THAT(pose.position.z, WithinAbs(0.0, 0.001));
        // Identity orientation (no rotation)
        CHECK_THAT(pose.orientation.w, WithinAbs(1.0, 0.001));
    }

    SECTION("Controllers at default positions")
    {
        XrPosef left = im.GetControllerPose(InputManager::Hand::Left);
        CHECK_THAT(left.position.x, WithinAbs(-0.2, 0.001));
        CHECK_THAT(left.position.y, WithinAbs(1.3, 0.001));
        CHECK_THAT(left.position.z, WithinAbs(-0.4, 0.001));

        XrPosef right = im.GetControllerPose(InputManager::Hand::Right);
        CHECK_THAT(right.position.x, WithinAbs(0.2, 0.001));
        CHECK_THAT(right.position.y, WithinAbs(1.3, 0.001));
        CHECK_THAT(right.position.z, WithinAbs(-0.4, 0.001));
    }

    SECTION("Mode defaults to Controller")
    {
        CHECK(im.GetInputMode() == InputManager::InputMode::Controller);
    }

    SECTION("Buttons default to released")
    {
        CHECK(im.GetGrabValue(InputManager::Hand::Left) == 0.0f);
        CHECK(im.GetGrabValue(InputManager::Hand::Right) == 0.0f);
        CHECK(im.GetMenuClick() == false);
    }

    SECTION("Not streaming by default")
    {
        CHECK(im.IsStreaming() == false);
    }
}

TEST_CASE("InputManager — hand joint generation", "[input]")
{
    InputManager im;

    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT] = {};
    im.GetHandJointLocations(InputManager::Hand::Left, joints, XR_HAND_JOINT_COUNT_EXT);

    SECTION("All 26 joints have valid flags")
    {
        for (uint32_t i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++)
        {
            CHECK((joints[i].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0);
            CHECK((joints[i].locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0);
        }
    }

    SECTION("All joints have positive radius")
    {
        for (uint32_t i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++)
        {
            CHECK(joints[i].radius > 0.0f);
        }
    }

    SECTION("Palm is at controller position")
    {
        XrPosef ctrl = im.GetControllerPose(InputManager::Hand::Left);
        CHECK_THAT(joints[0].pose.position.x, WithinAbs(ctrl.position.x, 0.001));
        CHECK_THAT(joints[0].pose.position.y, WithinAbs(ctrl.position.y, 0.001));
        CHECK_THAT(joints[0].pose.position.z, WithinAbs(ctrl.position.z, 0.001));
    }
}

TEST_CASE("InputManager — eye views", "[input]")
{
    InputManager im;

    XrView views[2] = {};
    im.GetEyeViews(views, 2);

    SECTION("Two views with valid types")
    {
        CHECK(views[0].type == XR_TYPE_VIEW);
        CHECK(views[1].type == XR_TYPE_VIEW);
    }

    SECTION("Left eye is to the left of right eye")
    {
        CHECK(views[0].pose.position.x < views[1].pose.position.x);
    }

    SECTION("FOV is set")
    {
        CHECK(views[0].fov.angleLeft < 0.0f);
        CHECK(views[0].fov.angleRight > 0.0f);
        CHECK(views[0].fov.angleUp > 0.0f);
        CHECK(views[0].fov.angleDown < 0.0f);
    }
}

TEST_CASE("InputManager — streaming eye data", "[input]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);

    oxr::protocol::TrackingPacket packet = {};
    packet.ipd = 0.070f;
    packet.eyeFov[0] = -1.10f;
    packet.eyeFov[1] = 0.90f;
    packet.eyeFov[2] = 1.00f;
    packet.eyeFov[3] = -0.95f;
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));

    im.Update(0.0f);

    XrView views[2] = {};
    im.GetEyeViews(views, 2);

    SECTION("Streaming IPD overrides the default eye separation")
    {
        CHECK_THAT(views[1].pose.position.x - views[0].pose.position.x, WithinAbs(0.070f, 0.001f));
    }

    SECTION("Streaming FOV is used for the left eye and mirrored for the right eye")
    {
        CHECK_THAT(views[0].fov.angleLeft, WithinAbs(-1.10f, 0.001f));
        CHECK_THAT(views[0].fov.angleRight, WithinAbs(0.90f, 0.001f));
        CHECK_THAT(views[0].fov.angleUp, WithinAbs(1.00f, 0.001f));
        CHECK_THAT(views[0].fov.angleDown, WithinAbs(-0.95f, 0.001f));

        CHECK_THAT(views[1].fov.angleLeft, WithinAbs(-0.90f, 0.001f));
        CHECK_THAT(views[1].fov.angleRight, WithinAbs(1.10f, 0.001f));
        CHECK_THAT(views[1].fov.angleUp, WithinAbs(1.00f, 0.001f));
        CHECK_THAT(views[1].fov.angleDown, WithinAbs(-0.95f, 0.001f));
    }
}

TEST_CASE("InputManager — streaming head pose", "[input]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);

    oxr::protocol::TrackingPacket packet = {};
    packet.headPosition[0] = 1.0f;
    packet.headPosition[1] = 2.0f;
    packet.headPosition[2] = 3.0f;
    glm::quat yaw45 = glm::angleAxis(0.785f, glm::vec3(0.0f, 1.0f, 0.0f));
    packet.headOrientation[0] = yaw45.x;
    packet.headOrientation[1] = yaw45.y;
    packet.headOrientation[2] = yaw45.z;
    packet.headOrientation[3] = yaw45.w;
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));

    im.Update(0.0f);

    XrPosef pose = im.GetHeadPose();
    CHECK_THAT(pose.position.x, WithinAbs(1.0, 0.001));
    CHECK_THAT(pose.position.y, WithinAbs(2.0, 0.001));
    CHECK_THAT(pose.position.z, WithinAbs(3.0, 0.001));
    CHECK(std::abs(pose.orientation.y) > 0.01f);
}

TEST_CASE("InputManager — streaming controller activity gates pose updates", "[input]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);
    im.SetStreamingClientName("Meta Quest 2");

    oxr::protocol::TrackingPacket active = {};
    active.timestampNs = 1'000'000'000;
    active.headOrientation[3] = 1.0f;
    active.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE |
                           oxr::protocol::TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE;
    active.leftControllerPos[0] = -0.35f;
    active.leftControllerPos[1] = 1.20f;
    active.leftControllerPos[2] = -0.55f;
    active.leftControllerRot[3] = 1.0f;
    active.rightControllerPos[0] = 0.35f;
    active.rightControllerPos[1] = 1.25f;
    active.rightControllerPos[2] = -0.50f;
    active.rightControllerRot[3] = 1.0f;

    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&active), sizeof(active));
    im.Update(0.0f);

    CHECK(im.IsControllerTrackingActive(InputManager::Hand::Left));
    CHECK(im.IsInputDeviceActive(InputManager::Hand::Left));
    CHECK(im.GetCurrentInteractionProfile(InputManager::Hand::Left) ==
          "/interaction_profiles/meta/touch_controller_quest_2");
    XrPosef left = im.GetControllerPose(InputManager::Hand::Left);
    CHECK_THAT(left.position.x, WithinAbs(-0.35f, 0.001f));
    CHECK_THAT(left.position.y, WithinAbs(1.20f, 0.001f));
    CHECK_THAT(left.position.z, WithinAbs(-0.55f, 0.001f));

    oxr::protocol::TrackingPacket inactive = {};
    inactive.timestampNs = 1'011'111'111;
    inactive.headOrientation[3] = 1.0f;
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&inactive), sizeof(inactive));
    im.Update(0.0f);

    CHECK_FALSE(im.IsControllerTrackingActive(InputManager::Hand::Left));
    CHECK_FALSE(im.IsInputDeviceActive(InputManager::Hand::Left));
    CHECK(im.GetCurrentInteractionProfile(InputManager::Hand::Left).empty());
    left = im.GetControllerPose(InputManager::Hand::Left);
    CHECK_THAT(left.position.x, WithinAbs(-0.35f, 0.001f));
    CHECK_THAT(left.position.y, WithinAbs(1.20f, 0.001f));
    CHECK_THAT(left.position.z, WithinAbs(-0.55f, 0.001f));
}

TEST_CASE("InputManager — streaming client names map to controller profiles and aliases", "[input]")
{
    struct Case
    {
        const char* clientName;
        const char* expectedProfile;
    };

    const Case cases[] = {
        {"Oculus Quest", "/interaction_profiles/oculus/touch_controller"},
        {"Meta Quest 1", "/interaction_profiles/meta/touch_controller_quest_1_rift_s"},
        {"Meta Quest 2", "/interaction_profiles/meta/touch_controller_quest_2"},
        {"Meta Quest 3", "/interaction_profiles/meta/touch_plus_controller"},
        {"Quest 3", "/interaction_profiles/meta/touch_plus_controller"},
        {"Unknown headset", "/interaction_profiles/oculus/touch_controller"},
        {"PICO Neo3", "/interaction_profiles/bytedance/pico_neo3_controller"},
        {"PICO 4", "/interaction_profiles/bytedance/pico4_controller"},
    };

    for (const Case& testCase : cases)
    {
        INFO(testCase.clientName);
        InputManager im;
        TrackingReceiver receiver;
        im.SetTrackingReceiver(&receiver);
        im.SetStreamingClientName(testCase.clientName);

        oxr::protocol::TrackingPacket packet = {};
        packet.timestampNs = 1'000'000'000;
        packet.headOrientation[3] = 1.0f;
        packet.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE;
        packet.leftControllerRot[3] = 1.0f;
        receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        im.Update(0.0f);

        CHECK(im.GetCurrentInteractionProfile(InputManager::Hand::Left) == testCase.expectedProfile);
        std::vector<std::string> profiles = im.GetActiveInteractionProfiles(InputManager::Hand::Left);
        CHECK(std::find(profiles.begin(), profiles.end(), testCase.expectedProfile) != profiles.end());
        CHECK(std::find(profiles.begin(), profiles.end(),
                        "/interaction_profiles/khr/simple_controller") != profiles.end());
        if (std::string(testCase.expectedProfile).find("/interaction_profiles/meta/") == 0)
        {
            CHECK(std::find(profiles.begin(), profiles.end(),
                            "/interaction_profiles/oculus/touch_controller") != profiles.end());
        }
    }
}

TEST_CASE("InputManager — streaming hands and controllers stay profile separated", "[input]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);
    im.SetStreamingClientName("Meta Quest 3");

    oxr::protocol::TrackingPacket packet = {};
    packet.timestampNs = 1'000'000'000;
    packet.headOrientation[3] = 1.0f;
    packet.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE |
                           oxr::protocol::TRACKING_FLAG_LEFT_HAND_ACTIVE;
    packet.leftControllerPos[0] = -0.40f;
    packet.leftControllerPos[1] = 1.10f;
    packet.leftControllerPos[2] = -0.60f;
    packet.leftControllerRot[3] = 1.0f;
    packet.leftTrigger = 0.20f;
    packet.leftGrip = 0.10f;
    packet.leftThumbstick[0] = 0.25f;
    packet.leftThumbstick[1] = -0.50f;
    PopulateLeftPinchingHand(packet, 0.10f, 1.30f, -0.20f);

    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    im.Update(0.0f);

    constexpr const char* TouchPlusProfile = "/interaction_profiles/meta/touch_plus_controller";
    constexpr const char* HandProfile = "/interaction_profiles/ext/hand_interaction_ext";

    CHECK(im.IsControllerTrackingActive(InputManager::Hand::Left));
    CHECK(im.IsHandTrackingActive(InputManager::Hand::Left));
    CHECK(im.GetCurrentInteractionProfile(InputManager::Hand::Left) == TouchPlusProfile);

    std::vector<std::string> activeProfiles = im.GetActiveInteractionProfiles(InputManager::Hand::Left);
    CHECK(std::find(activeProfiles.begin(), activeProfiles.end(), TouchPlusProfile) !=
          activeProfiles.end());
    CHECK(std::find(activeProfiles.begin(), activeProfiles.end(), HandProfile) !=
          activeProfiles.end());
    CHECK(std::find(activeProfiles.begin(), activeProfiles.end(),
                    "/interaction_profiles/khr/simple_controller") != activeProfiles.end());

    CHECK_THAT(im.GetFloatComponentForProfile(InputManager::Hand::Left,
                                              "trigger/value", TouchPlusProfile),
               WithinAbs(0.20f, 0.001f));
    CHECK_THAT(im.GetFloatComponentForProfile(InputManager::Hand::Left,
                                              "squeeze/value", TouchPlusProfile),
               WithinAbs(0.10f, 0.001f));
    XrVector2f stick = im.GetVector2fComponentForProfile(InputManager::Hand::Left,
                                                         "thumbstick", TouchPlusProfile);
    CHECK_THAT(stick.x, WithinAbs(0.25f, 0.001f));
    CHECK_THAT(stick.y, WithinAbs(-0.50f, 0.001f));

    XrPosef controllerPose = im.GetPoseComponentForProfile(InputManager::Hand::Left,
                                                           "grip/pose", TouchPlusProfile);
    CHECK_THAT(controllerPose.position.x, WithinAbs(-0.40f, 0.001f));
    CHECK_THAT(controllerPose.position.y, WithinAbs(1.10f, 0.001f));
    CHECK_THAT(controllerPose.position.z, WithinAbs(-0.60f, 0.001f));

    CHECK(im.GetFloatComponentForProfile(InputManager::Hand::Left,
                                         "pinch_ext/value", HandProfile) > 0.95f);
    CHECK(im.GetFloatComponentForProfile(InputManager::Hand::Left,
                                         "grasp_ext/value", HandProfile) > 0.75f);
    XrPosef handPose = im.GetPoseComponentForProfile(InputManager::Hand::Left,
                                                     "grip/pose", HandProfile);
    CHECK_THAT(handPose.position.x, WithinAbs(0.10f, 0.001f));
    CHECK_THAT(handPose.position.y, WithinAbs(1.30f, 0.001f));
    CHECK_THAT(handPose.position.z, WithinAbs(-0.20f, 0.001f));

    oxr::protocol::TrackingPacket handOnly = {};
    handOnly.timestampNs = 1'011'111'111;
    handOnly.headOrientation[3] = 1.0f;
    handOnly.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_HAND_ACTIVE;
    PopulateLeftPinchingHand(handOnly, 0.20f, 1.35f, -0.25f);
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&handOnly), sizeof(handOnly));
    im.Update(0.0f);

    CHECK_FALSE(im.IsControllerTrackingActive(InputManager::Hand::Left));
    CHECK(im.IsHandTrackingActive(InputManager::Hand::Left));
    CHECK(im.GetCurrentInteractionProfile(InputManager::Hand::Left) == HandProfile);

    XrPosef lastControllerPose = im.GetControllerPose(InputManager::Hand::Left);
    CHECK_THAT(lastControllerPose.position.x, WithinAbs(-0.40f, 0.001f));
    CHECK_THAT(lastControllerPose.position.y, WithinAbs(1.10f, 0.001f));
    CHECK_THAT(lastControllerPose.position.z, WithinAbs(-0.60f, 0.001f));

    handPose = im.GetPoseComponentForProfile(InputManager::Hand::Left,
                                             "grip/pose", HandProfile);
    CHECK_THAT(handPose.position.x, WithinAbs(0.20f, 0.001f));
    CHECK_THAT(handPose.position.y, WithinAbs(1.35f, 0.001f));
    CHECK_THAT(handPose.position.z, WithinAbs(-0.25f, 0.001f));
}

TEST_CASE("InputManager — select follows trigger and squeeze follows grab", "[input]")
{
    InputManager im;
    TrackingReceiver receiver;
    im.SetTrackingReceiver(&receiver);
    im.SetStreamingClientName("Oculus Quest");

    oxr::protocol::TrackingPacket packet = {};
    packet.timestampNs = 1'000'000'000;
    packet.headOrientation[3] = 1.0f;
    packet.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE;
    packet.leftControllerRot[3] = 1.0f;
    packet.leftTrigger = 0.75f;
    packet.leftGrip = 0.20f;
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    im.Update(0.0f);

    CHECK_THAT(im.GetFloatComponent(InputManager::Hand::Left, "select/value"),
               WithinAbs(0.75f, 0.001f));
    CHECK_THAT(im.GetFloatComponent(InputManager::Hand::Left, "trigger/value"),
               WithinAbs(0.75f, 0.001f));
    CHECK_THAT(im.GetFloatComponent(InputManager::Hand::Left, "squeeze/value"),
               WithinAbs(0.20f, 0.001f));
    CHECK(im.GetButtonClick(InputManager::Hand::Left, "select/click"));
    CHECK_FALSE(im.GetButtonClick(InputManager::Hand::Left, "squeeze/click"));
}

TEST_CASE("TrackingReceiver — predicted pose extrapolates recent motion", "[input]")
{
    TrackingReceiver receiver;

    oxr::protocol::TrackingPacket first = {};
    first.timestampNs = 1'000'000'000;
    first.headPosition[0] = 0.000f;
    first.headOrientation[3] = 1.0f;

    oxr::protocol::TrackingPacket second = {};
    second.timestampNs = 1'011'111'111;
    second.headPosition[0] = 0.020f;
    glm::quat yaw10 = glm::angleAxis(0.1745f, glm::vec3(0.0f, 1.0f, 0.0f));
    second.headOrientation[0] = yaw10.x;
    second.headOrientation[1] = yaw10.y;
    second.headOrientation[2] = yaw10.z;
    second.headOrientation[3] = yaw10.w;

    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&first), sizeof(first));
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&second), sizeof(second));
    receiver.SetPredictionHorizonMs(11.0f);

    oxr::protocol::TrackingPacket predicted = {};
    REQUIRE(receiver.GetPredictedPose(predicted));

    SECTION("Head position advances beyond the latest packet")
    {
        CHECK(predicted.headPosition[0] > second.headPosition[0]);
    }

    SECTION("Head orientation advances beyond the latest packet")
    {
        CHECK(std::abs(predicted.headOrientation[1]) > std::abs(second.headOrientation[1]));
    }
}

TEST_CASE("TrackingReceiver — controller prediction requires active history", "[input]")
{
    TrackingReceiver receiver;

    oxr::protocol::TrackingPacket inactive = {};
    inactive.timestampNs = 1'000'000'000;
    inactive.headOrientation[3] = 1.0f;

    oxr::protocol::TrackingPacket active = {};
    active.timestampNs = 1'011'111'111;
    active.headOrientation[3] = 1.0f;
    active.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE;
    active.leftControllerPos[0] = -0.30f;
    active.leftControllerPos[1] = 1.10f;
    active.leftControllerPos[2] = -0.50f;
    active.leftControllerRot[3] = 1.0f;

    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&inactive), sizeof(inactive));
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&active), sizeof(active));
    receiver.SetPredictionHorizonMs(20.0f);

    oxr::protocol::TrackingPacket predicted = {};
    REQUIRE(receiver.GetPredictedPose(predicted));

    CHECK((predicted.trackingFlags & oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE) != 0);
    CHECK_THAT(predicted.leftControllerPos[0], WithinAbs(active.leftControllerPos[0], 0.001f));
    CHECK_THAT(predicted.leftControllerPos[1], WithinAbs(active.leftControllerPos[1], 0.001f));
    CHECK_THAT(predicted.leftControllerPos[2], WithinAbs(active.leftControllerPos[2], 0.001f));
}

TEST_CASE("TrackingReceiver — angular velocity uses the full prediction horizon", "[input]")
{
    TrackingReceiver receiver;

    oxr::protocol::TrackingPacket first = {};
    first.timestampNs = 2'000'000'000;
    first.headOrientation[3] = 1.0f;

    oxr::protocol::TrackingPacket second = {};
    second.timestampNs = 2'011'111'111;
    second.headOrientation[3] = 1.0f;
    second.headAngularVelocity[1] = 1.0f;

    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&first), sizeof(first));
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&second), sizeof(second));
    receiver.SetPredictionHorizonMs(20.0f);

    oxr::protocol::TrackingPacket predicted = {};
    REQUIRE(receiver.GetPredictedPose(predicted));

    CHECK_THAT(predicted.headOrientation[1], WithinAbs(std::sin(0.010f), 0.001f));
    CHECK_THAT(predicted.headOrientation[3], WithinAbs(std::cos(0.010f), 0.001f));
}

// Regression: a Quest client built before the aim-pose fields were appended to
// TrackingPacket sends a 1008-byte prefix of the 1064-byte struct. The USB/TCP read loop
// in StreamingServer gated on `payload.size() >= sizeof(TrackingPacket)` and so dropped
// every one of those records: the connection looked healthy ("tracking client connected")
// but no packet was ever stored, IsReceiving() stayed false, InputManager::Update() never
// called UpdateFromStreaming(), and xrLocateSpace(VIEW) returned the default pose forever.
// All transports now share IsAcceptableTrackingPayloadSize().
TEST_CASE("TrackingReceiver — accepts short packets from pre-aim-pose clients", "[input][protocol]")
{
    // What an older client actually puts on the wire.
    constexpr size_t kOldClientPacketSize = oxr::protocol::TRACKING_PACKET_MIN_WIRE_SIZE;
    STATIC_REQUIRE(kOldClientPacketSize < sizeof(oxr::protocol::TrackingPacket));

    oxr::protocol::TrackingPacket source = {};
    source.timestampNs = 1'000'000'000;
    source.headPosition[0] = -0.296f;
    source.headPosition[1] = 0.737f;
    source.headPosition[2] = -0.079f;
    source.headOrientation[3] = 1.0f;
    source.trackingFlags = oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE;
    // Aim fields the old client never sends; they must not survive into the stored packet.
    source.leftControllerAimPos[0] = 99.0f;
    source.rightControllerAimRot[3] = 99.0f;

    std::vector<uint8_t> wire(kOldClientPacketSize);
    std::memcpy(wire.data(), &source, kOldClientPacketSize);

    SECTION("a truncated packet is stored, not dropped")
    {
        TrackingReceiver receiver;
        REQUIRE_FALSE(receiver.IsReceiving());

        receiver.InjectPacket(wire.data(), wire.size());

        REQUIRE(receiver.IsReceiving());
        CHECK(receiver.GetPacketCount() == 1);

        oxr::protocol::TrackingPacket stored = {};
        REQUIRE(receiver.GetLatestPose(stored));
        CHECK_THAT(stored.headPosition[0], WithinAbs(-0.296, 0.0001));
        CHECK_THAT(stored.headPosition[1], WithinAbs(0.737, 0.0001));
        CHECK_THAT(stored.headPosition[2], WithinAbs(-0.079, 0.0001));
        CHECK(stored.trackingFlags == oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE);

        // Fields past the truncation point are zero-filled, never read from the short buffer.
        CHECK_THAT(stored.leftControllerAimPos[0], WithinAbs(0.0, 0.0001));
        CHECK_THAT(stored.rightControllerAimRot[3], WithinAbs(0.0, 0.0001));
    }

    SECTION("a packet shorter than the minimum wire size is still rejected")
    {
        TrackingReceiver receiver;
        receiver.InjectPacket(wire.data(), kOldClientPacketSize - 1);
        CHECK_FALSE(receiver.IsReceiving());
        CHECK(receiver.GetPacketCount() == 0);
    }

    SECTION("InputManager picks up the head pose from a short packet")
    {
        TrackingReceiver receiver;
        InputManager im;
        im.SetTrackingReceiver(&receiver);

        // No data yet: the default pose, and NOT reported as tracked.
        im.Update(0.011f);
        XrPosef before = im.GetHeadPose();
        CHECK_THAT(before.position.y, WithinAbs(1.6, 0.001));
        CHECK_FALSE(im.IsHeadPoseTracked());

        receiver.InjectPacket(wire.data(), wire.size());
        im.Update(0.011f);

        XrPosef after = im.GetHeadPose();
        CHECK_THAT(after.position.x, WithinAbs(-0.296, 0.001));
        CHECK_THAT(after.position.y, WithinAbs(0.737, 0.001));
        CHECK_THAT(after.position.z, WithinAbs(-0.079, 0.001));
        CHECK(im.IsHeadPoseTracked());

        im.SetTrackingReceiver(nullptr);
    }
}

TEST_CASE("InputManager — head pose is only reported tracked with live data", "[input]")
{
    SECTION("simulator mode (no receiver) reports its synthesised pose as tracked")
    {
        InputManager im;
        CHECK(im.IsHeadPoseTracked());
    }

    SECTION("streaming with no packet yet is valid but untracked")
    {
        TrackingReceiver receiver;
        InputManager im;
        im.SetTrackingReceiver(&receiver);
        CHECK(im.IsStreaming());
        CHECK_FALSE(im.IsHeadPoseTracked());
        im.SetTrackingReceiver(nullptr);
    }

    SECTION("after a client disconnects the stale pose is no longer tracked")
    {
        oxr::protocol::TrackingPacket packet = {};
        packet.timestampNs = 1'000'000'000;
        packet.headPosition[1] = 1.2f;
        packet.headOrientation[3] = 1.0f;

        TrackingReceiver receiver;
        InputManager im;
        im.SetTrackingReceiver(&receiver);
        receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        im.Update(0.011f);
        REQUIRE(im.IsHeadPoseTracked());

        // Client goes away: the last pose is still served (valid) but is not live.
        im.SetTrackingReceiver(nullptr);
        CHECK_FALSE(im.IsStreaming());
        CHECK_FALSE(im.IsHeadPoseTracked());
        CHECK_THAT(im.GetHeadPose().position.y, WithinAbs(1.2, 0.001));
    }
}

namespace
{

oxr::protocol::TrackingPacket Quest2Packet(float headX, float headY, float headZ)
{
    oxr::protocol::TrackingPacket packet = {};
    packet.timestampNs = 1'000'000'000;
    packet.headPosition[0] = headX;
    packet.headPosition[1] = headY;
    packet.headPosition[2] = headZ;
    packet.headOrientation[3] = 1.0f;
    packet.ipd = 0.0583f;
    packet.eyeFov[0] = -0.9076f;
    packet.eyeFov[1] = 0.7330f;
    packet.eyeFov[2] = 0.8378f;
    packet.eyeFov[3] = -0.8727f;
    return packet;
}

// Head position relative to the LOCAL origin (no rotation involved in these tests).
float HeadYInLocal(const InputManager& im)
{
    return im.GetHeadPose().position.y - im.GetReferenceSpacePose(XR_REFERENCE_SPACE_TYPE_LOCAL).position.y;
}

} // namespace

TEST_CASE("InputManager — LOCAL is head-anchored before and after the first tracking packet", "[input][spaces]")
{
    TrackingReceiver receiver;
    InputManager im;

    // Before any client: the head is the placeholder and LOCAL is provisionally anchored
    // on it, so an app that reads its first pose sees the head at the LOCAL origin rather
    // than 1.6m up (and never sees it jump down by the eye height later).
    CHECK_THAT(HeadYInLocal(im), WithinAbs(0.0, 1e-5));
    XrPosef unused{};
    CHECK_FALSE(im.TakeLocalReferenceChange(unused));

    im.SetTrackingReceiver(&receiver);
    const oxr::protocol::TrackingPacket packet = Quest2Packet(-0.07f, 1.515f, 0.21f);
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    im.Update(0.011f);

    CHECK_THAT(HeadYInLocal(im), WithinAbs(0.0, 1e-5));
    CHECK_THAT(im.GetReferenceSpacePose(XR_REFERENCE_SPACE_TYPE_STAGE).position.y, WithinAbs(0.0, 1e-5));

    // The re-anchor is reported once, as the new origin in the previous LOCAL space.
    XrPosef moved{};
    REQUIRE(im.TakeLocalReferenceChange(moved));
    CHECK_THAT(moved.position.x, WithinAbs(-0.07, 1e-4));
    CHECK_THAT(moved.position.y, WithinAbs(1.515 - 1.6, 1e-4));
    CHECK_THAT(moved.position.z, WithinAbs(0.21, 1e-4));
    CHECK_FALSE(im.TakeLocalReferenceChange(moved));

    im.SetTrackingReceiver(nullptr);
}

TEST_CASE("InputManager — a streaming server waiting for its client reports an untracked head", "[input]")
{
    TrackingReceiver receiver;
    InputManager im;
    im.SetAwaitingStreamingClient(true);
    CHECK_FALSE(im.IsHeadPoseTracked());

    im.SetTrackingReceiver(&receiver);
    CHECK_FALSE(im.IsHeadPoseTracked());
    const oxr::protocol::TrackingPacket packet = Quest2Packet(0.0f, 1.5f, 0.0f);
    receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    im.Update(0.011f);
    CHECK(im.IsHeadPoseTracked());
    im.SetTrackingReceiver(nullptr);
}

TEST_CASE("InputManager — a restarted session reports the headset view from its first frame", "[input][restart]")
{
    // Session 1: the client streams its real view; the session persists it.
    oxrsys::runtime::SavedHeadsetView saved;
    {
        TrackingReceiver receiver;
        InputManager first;
        first.SetAwaitingStreamingClient(true);
        first.SetTrackingReceiver(&receiver);
        const oxr::protocol::TrackingPacket packet = Quest2Packet(-0.07f, 1.515f, 0.21f);
        receiver.InjectPacket(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        first.Update(0.011f);
        REQUIRE(first.TakeHeadsetViewUpdate(saved));
        CHECK_FALSE(first.TakeHeadsetViewUpdate(saved)); // only on change
        first.SetTrackingReceiver(nullptr);
    }

    // Session 2 (OpenComposite restarts its session to load input bindings): before the
    // client's tracking reaches it, views already use the saved headset view, the head is
    // untracked, and it sits at the LOCAL origin.
    InputManager second;
    second.SetSavedHeadsetView(saved);
    second.SetAwaitingStreamingClient(true);

    XrView views[2] = {};
    second.GetEyeViews(views, 2);
    CHECK_THAT(views[0].fov.angleLeft, WithinAbs(-0.9076, 1e-5));
    CHECK_THAT(views[0].fov.angleRight, WithinAbs(0.7330, 1e-5));
    CHECK_THAT(views[1].fov.angleLeft, WithinAbs(-0.7330, 1e-5));
    CHECK_THAT(views[1].fov.angleRight, WithinAbs(0.9076, 1e-5));
    CHECK_THAT(views[1].pose.position.x - views[0].pose.position.x, WithinAbs(0.0583, 1e-5));
    CHECK_FALSE(second.IsHeadPoseTracked());
    CHECK_THAT(HeadYInLocal(second), WithinAbs(0.0, 1e-5));

    // Without a saved view the placeholder is still used.
    InputManager fresh;
    XrView placeholder[2] = {};
    fresh.GetEyeViews(placeholder, 2);
    CHECK_THAT(placeholder[1].pose.position.x - placeholder[0].pose.position.x, WithinAbs(0.063, 1e-5));
    CHECK_THAT(placeholder[0].fov.angleLeft, WithinAbs(-placeholder[0].fov.angleRight, 1e-6));
}
