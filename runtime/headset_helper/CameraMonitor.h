// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief A desktop window showing what the headset's tracking cameras see.
 *
 * Both camera images side by side, with the features Basalt is tracking
 * drawn over them (read from the VIT monitor shim, drivers/monado/
 * vit_monitor.h), the PS Move sphere's tracked position projected into
 * camera 0, and a status line. Frames arrive on the camera reader thread
 * through PushFrame(); everything else happens on the main thread.
 *
 * Objective-C++ only; the helper keeps it behind a pointer.
 */

#pragma once

#import <CoreGraphics/CoreGraphics.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

struct xrt_device;

namespace oxrsys {

/*!
 * What the helper knows and the monitor shows; the provider callback fills
 * it on the main thread before every redraw.
 */
struct CameraMonitorInfo
{
	std::string trackingKind;
	//! Why there is no SLAM ("no VIT library", "IMU only"); empty when there is.
	std::string slamNote;
	bool haveHeadPosition = false;
	float headPosition[3] = {0.0f, 0.0f, 0.0f};
	//! A sphere-tracked controller's position in camera 0's frame (Monado
	//! camera space: +X right, +Y up, -Z forward), when one is reported.
	bool controllerPositionValid = false;
	float controllerPosition[3] = {0.0f, 0.0f, 0.0f};
};

class CameraMonitor
{
public:
	//! Implementation state; public only so the NSView can hold it.
	struct Impl;

	CameraMonitor();
	~CameraMonitor();

	CameraMonitor(const CameraMonitor &) = delete;
	CameraMonitor &operator=(const CameraMonitor &) = delete;

	/*!
	 * Pinhole intrinsics of camera @p cam in pixels, for projecting the
	 * controller position. SetIntrinsicsFromHeadset() reads them from a WMR
	 * headset's calibration; returns false when the device is not one.
	 */
	void SetIntrinsics(uint32_t cam, uint32_t width, uint32_t height, float fx, float fy, float cx, float cy);
	bool SetIntrinsicsFromHeadset(struct xrt_device *hmd);

	/*!
	 * Store a camera frame (8-bit grayscale, @p stride bytes per row). Any
	 * thread; only copies the pixels.
	 */
	void PushFrame(uint32_t cam, const uint8_t *data, uint32_t width, uint32_t height, size_t stride, int64_t timestampNs);

	/*!
	 * Create the window on a screen other than @p avoidDisplay (the headset
	 * panel), or the main screen when there is no other, and start
	 * redrawing at ~30 Hz. @p shimPath names liboxrsys-vit-monitor.dylib to
	 * read features from; empty for none. Main thread only.
	 */
	bool Open(CGDirectDisplayID avoidDisplay,
	          const std::string &shimPath,
	          std::function<void(CameraMonitorInfo &)> provider);

	//! Tear down the window and timer. Main thread only.
	void Close();

private:
	Impl *impl_;
};

} // namespace oxrsys
