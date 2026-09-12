// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Present distortion-corrected stereo images on a WMR headset's panel.
 *
 * The panel is an ordinary macOS display once the EDID override is in place
 * (see docs/platforms/wmr.md). This class finds it, covers it with a
 * CAMetalLayer, uploads the driver's per-channel distortion mesh once, and
 * warps a pair of eye textures into each drawable.
 *
 * Objective-C++ only; C++ callers keep it behind a pointer.
 */

#pragma once

#import <CoreGraphics/CoreGraphics.h>
#import <Metal/Metal.h>

#include <cstdint>
#include <vector>

struct xrt_hmd_parts;

namespace oxrsys {

struct WmrPanelView
{
	uint32_t x_pixels = 0, y_pixels = 0, w_pixels = 0, h_pixels = 0; // Viewport on the panel.
	uint32_t render_w = 0, render_h = 0;                             // Eye image size the driver expects.
	float fov[4] = {};                                               // left, right, up, down (radians).
};

struct WmrPanelGeometry
{
	uint32_t panel_w = 0;
	uint32_t panel_h = 0;
	uint32_t view_count = 0;
	WmrPanelView views[2] = {};
};

struct WmrPanelOptions
{
	CGDirectDisplayID display_id = 0; //!< 0: auto-detect.
	double display_timeout_s = 20.0;  //!< How long to wait for the panel to hot-plug.
	bool capture = false;             //!< Also CGDisplayCapture() the display.
	//! Displays that were online before the driver switched the panel on
	//! (WmrPanel::OnlineDisplays()); lets a hot-plugged display be recognised
	//! even without a mode of exactly the panel's size.
	std::vector<CGDirectDisplayID> displays_before;
};

/*!
 * One eye image to warp: the texture plus an optional GPU-side wait so the
 * warp does not read it before the producer's last write landed.
 */
struct WmrPanelEyeInput
{
	id<MTLTexture> texture = nil;
	id<MTLSharedEvent> waitEvent = nil;
	uint64_t waitValue = 0;
};

class WmrPanel
{
public:
	/*!
	 * @param parts The driver's HMD description; geometry and mesh are copied.
	 */
	explicit WmrPanel(const struct xrt_hmd_parts *parts);
	~WmrPanel();

	WmrPanel(const WmrPanel &) = delete;
	WmrPanel &operator=(const WmrPanel &) = delete;

	//! Displays online right now; see WmrPanelOptions::displays_before.
	static std::vector<CGDirectDisplayID> OnlineDisplays();

	/*!
	 * Create the warp pipeline and mesh buffers on @p device, the device the
	 * eye textures live on. Required before Present(); idempotent.
	 */
	bool EnsureRenderer(id<MTLDevice> device);

	/*!
	 * Find the panel's display (blocking, up to the timeout; safe to call from
	 * any thread), switch it to the panel's native mode, and schedule the
	 * window on the main thread. Returns false if no display was found.
	 * IsReady() turns true once the window exists.
	 */
	bool Open(const WmrPanelOptions &options);

	//! Window and layer exist; Present() will draw.
	bool IsReady() const;

	//! Tear down the window, release the display. Safe from any thread.
	void Close();

	const WmrPanelGeometry &Geometry() const
	{
		return geometry_;
	}

	//! Refresh rate of the mode the display was switched to (0 if unknown).
	double RefreshRateHz() const
	{
		return refreshHz_;
	}

	CGDirectDisplayID DisplayId() const
	{
		return displayId_;
	}

	/*!
	 * Warp the two eye images into the next drawable and present it. Blocks
	 * for the drawable (normally until the previous frame scans out) and
	 * returns once the command buffer is committed. @p completed runs when the
	 * GPU has finished reading the inputs; release them there.
	 *
	 * Returns false without touching the inputs when the panel is not ready.
	 */
	bool Present(id<MTLCommandQueue> queue,
	             const WmrPanelEyeInput &left,
	             const WmrPanelEyeInput &right,
	             void (^completed)(void));

	//! Clear the panel to a colour (used while no frames arrive).
	bool PresentClear(id<MTLCommandQueue> queue, double r, double g, double b);

	//! Frames presented since Open().
	uint64_t PresentedFrames() const
	{
		return presentedFrames_;
	}

private:
	struct Impl;
	Impl *impl_;

	WmrPanelGeometry geometry_ = {};
	CGDirectDisplayID displayId_ = 0;
	double refreshHz_ = 0.0;
	uint64_t presentedFrames_ = 0;
};

} // namespace oxrsys
