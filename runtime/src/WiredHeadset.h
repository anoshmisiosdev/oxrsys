// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "GraphicsTypes.h"

#include <cstdint>
#include <memory>
#include <string>

struct ConfigValues;
class TrackingReceiver;

/**
 * A headset attached directly to this machine (currently Windows Mixed
 * Reality through the Monado driver) that replaces the streaming client:
 * tracking comes from the headset's own sensors and submitted frames are
 * presented on its panel through the lens distortion warp.
 *
 * Process-wide singleton, because the panel window and the USB driver
 * outlive any one session and must be open before xrEnumerateViewConfigurationViews
 * so the recommended eye size matches the panel. Sessions attach their
 * graphics context to it for the frame path.
 *
 * All methods are safe to call from any thread. Frame submission is
 * non-blocking: SendFrame() hands the frame to a presenter thread through a
 * latest-frame-only queue, exactly like the streaming server.
 */
class WiredHeadset
{
public:
    static WiredHeadset& Shared();

    // True when the runtime was built with a wired-headset driver.
    static bool IsSupported();

    // Open the headset if the config enables it and one is connected. Idempotent;
    // returns IsOpen(). Switches the panel on, which makes macOS enumerate it.
    bool EnsureOpen(const ConfigValues& config);
    bool IsOpen() const;
    void Close();

    // Panel-native per-eye render size and refresh rate. Valid once open.
    uint32_t GetEyeWidth() const;
    uint32_t GetEyeHeight() const;
    uint32_t GetRefreshRateHz() const;
    std::string GetName() const;

    // Session frame path. AttachGraphics starts the presenter on the session's
    // Metal device; DetachGraphics stops it and drops any pending frame.
    bool AttachGraphics(const GraphicsContext& graphicsContext);
    void DetachGraphics();
    bool IsPresenting() const;
    void SendFrame(FrameSource frameSource);

    // Head (and headset-paired controller) tracking, fed continuously while open.
    TrackingReceiver* GetTrackingReceiver();

    uint64_t GetPresentedFrameCount() const;

private:
    WiredHeadset();
    ~WiredHeadset();
    WiredHeadset(const WiredHeadset&) = delete;
    WiredHeadset& operator=(const WiredHeadset&) = delete;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};
