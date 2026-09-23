// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "RuntimeSockets.h"

#include <oxrsys/protocol/Protocol.h>

/**
 * Receives 6DOF tracking data from the headset client via UDP.
 *
 * Runs a background thread that listens for TrackingPackets.
 * The latest pose data is stored and can be read by InputManager
 * to feed into the OpenXR runtime.
 */
class TrackingReceiver
{
public:
    TrackingReceiver() = default;
    ~TrackingReceiver();

    // Non-copyable
    TrackingReceiver(const TrackingReceiver&) = delete;
    TrackingReceiver& operator=(const TrackingReceiver&) = delete;

    bool Start();
    void Stop();

    // Get the latest tracking data (thread-safe)
    bool GetLatestPose(oxr::protocol::TrackingPacket& outPacket) const;
    bool GetPredictedPose(oxr::protocol::TrackingPacket& outPacket) const;

    // Raw controller velocity (linear m/s, angular rad/s) in the client tracking frame, from a
    // finite difference of the two most recent RAW pose samples in which the controller is
    // active — deliberately NOT the predicted/clamped pose, and deliberately undamped, so
    // velocity-based game mechanics (Unity deviceVelocity, XRI throw/punch detection) see the
    // true instantaneous controller speed. Any smoothing here would clip exactly the peak such
    // mechanics test for, and clip it harder the faster the motion. The head keeps its own
    // prediction/jitter handling untouched. Returns false when there is no usable recent history
    // pair for that controller, in which case the caller must report no velocity rather than a
    // fabricated one.
    bool GetRawControllerVelocity(bool leftHand, glm::vec3& linearVelocity,
                                  glm::vec3& angularVelocity) const;

    // Inject a tracking packet from TCP (USB mode) — same effect as receiving via UDP
    void InjectPacket(const uint8_t* data, size_t size);

    void SetPredictionHorizonMs(float predictionHorizonMs);
    float GetPredictionHorizonMs() const { return predictionHorizonMs_.load(); }

    // Check if we're receiving tracking data
    bool IsReceiving() const { return hasData_.load(); }
    bool IsRunning() const { return running_.load(); }

    // Stats
    uint64_t GetPacketCount() const { return packetCount_.load(); }
    uint64_t GetReorderedDropCount() const { return reorderedDropCount_.load(); }

private:
    struct HistorySample
    {
        oxr::protocol::TrackingPacket packet = {};
        int64_t receiveTimeNs = 0;
    };

    void ReceiveThread();
    void StorePacket(const oxr::protocol::TrackingPacket& packet, int64_t receiveTimeNs);

    oxrsys::runtime_socket::SocketHandle socket_ = oxrsys::runtime_socket::InvalidSocket;
    std::thread receiveThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> hasData_{false};
    std::atomic<uint64_t> packetCount_{0};
    std::atomic<uint64_t> reorderedDropCount_{0};

    mutable std::mutex poseMutex_;
    oxr::protocol::TrackingPacket latestPacket_ = {};
    std::deque<HistorySample> history_;
    std::atomic<float> predictionHorizonMs_{0.0f};
    mutable std::atomic<int64_t> lastPredictionDiagnosticNs_{0};
    // Bit 0 = left, bit 1 = right. Set the first time each controller yields a real velocity so
    // the "velocity is live" diagnostic is logged once per hand and never per frame.
    mutable std::atomic<uint32_t> velocityLoggedHands_{0};

    static constexpr size_t MaxHistorySamples = 8;
};
