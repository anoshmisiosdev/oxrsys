// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/**
 * Runtime-side client for the native-arm64 HEVC encoder helper.
 *
 * The runtime dylib is x86_64 (Rosetta) and cannot reach VideoToolbox's
 * hardware HEVC encoder. This client spawns a native-arm64 helper process that
 * can, hands it the compose IOSurfaces once (zero-copy, via mach send rights),
 * and thereafter submits "encode slot N" requests over a Unix socket, receiving
 * Annex-B NAL units back. If the helper fails to start, fails to obtain the
 * hardware encoder, or dies mid-session, this reports not-alive so the caller
 * falls back to the existing in-process software path — never a black screen.
 *
 * Header stays framework-free (IOSurfaces are passed as opaque void*); the .mm
 * resolves IOSurface / mach types.
 */
class HevcEncoderHelperClient
{
public:
    struct Config
    {
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t fps = 0;
        uint32_t bitrateMbps = 0;
        uint32_t keyframeIntervalSec = 2;
        uint32_t preset = 0; // 0 balanced, 1 speed, 2 quality
        std::string helperPath;
    };

    // cookie is an opaque token the caller uses to correlate results with the
    // original frame; the helper echoes it back verbatim.
    using OnNal = std::function<void(uint64_t cookie, const uint8_t* data, size_t size,
                                     bool isKeyframe, int64_t ptsNs)>;
    using OnFrameDone = std::function<void(uint64_t cookie, bool dropped, double encodeMs,
                                           bool keyframe)>;

    HevcEncoderHelperClient() = default;
    ~HevcEncoderHelperClient();

    HevcEncoderHelperClient(const HevcEncoderHelperClient&) = delete;
    HevcEncoderHelperClient& operator=(const HevcEncoderHelperClient&) = delete;

    // Spawns the helper, transfers the `count` IOSurfaces (index == slot), and
    // waits for its init acknowledgement. `iosurfaces[i]` is an IOSurfaceRef.
    // Returns true only if the helper came up on the hardware encoder.
    bool Start(const Config& config, void* const* iosurfaces, size_t count, OnNal onNal,
               OnFrameDone onFrameDone);

    // True once Start() succeeded and the helper is still responsive.
    bool IsAlive() const { return alive_.load(); }
    bool IsUsingHardware() const { return usingHardware_.load(); }

    // Submit slot for encoding. cookie is echoed back on the NAL/done callbacks.
    // Safe to call from any thread; a no-op if the helper is not alive.
    void SubmitFrame(uint64_t cookie, uint32_t slot, int64_t ptsNs, bool forceKeyframe);

    void SetBitrate(uint32_t bitrateMbps);

    // Orderly shutdown: asks the helper to exit, joins the reader, reaps child.
    void Stop();

private:
    bool SendFramed(uint16_t type, const std::vector<uint8_t>& payload);
    void ReaderLoop();
    void MarkDead(const char* reason);

    int sockFd_ = -1;        // parent end of the control socket
    int stderrReadFd_ = -1;  // parent end of the helper's captured stderr
    int childPid_ = -1;

    std::mutex writeMutex_;
    std::atomic<bool> alive_{false};
    std::atomic<bool> usingHardware_{false};
    std::atomic<bool> stopping_{false};

    std::thread readerThread_;
    std::thread stderrThread_;

    OnNal onNal_;
    OnFrameDone onFrameDone_;

    uint32_t slotCount_ = 0;
};
