// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  6DoF head tracking for a Windows Mixed Reality headset through a
 *         VIT plugin (Basalt), on top of Monado's SLAM tracker.
 *
 * The WMR driver can start its own SLAM tracker, but when that fails (no
 * plugin library installed) it refuses to create the headset at all. The
 * headset is therefore opened with the driver's SLAM disabled (`WMR_SLAM=false`)
 * and the tracker is created here afterwards, from the same calibration the
 * driver computed, so a missing Basalt only costs positional tracking. Owning
 * the tracker here also lets other camera consumers (the PS Move sphere
 * tracker) share the headset cameras through @ref oxrsys_wmr_slam_sinks.
 */

#pragma once

#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_fs;

enum oxrsys_wmr_slam_result
{
	OXRSYS_WMR_SLAM_OK = 0,
	//! This build has no SLAM support (needs OXRSYS_WMR_OPENCV).
	OXRSYS_WMR_SLAM_NOT_BUILT,
	//! The device is not a WMR headset or has fewer than two tracking cameras.
	OXRSYS_WMR_SLAM_NO_CAMERAS,
	//! Monado could not create the tracker: the VIT library did not load, or
	//! it rejected the calibration. The log has the reason.
	OXRSYS_WMR_SLAM_CREATE_FAILED,
	OXRSYS_WMR_SLAM_START_FAILED,
	//! The headset cameras could not be pointed at the tracker.
	OXRSYS_WMR_SLAM_CAMERA_FAILED,
};

struct oxrsys_wmr_slam;

/*!
 * Whether this build can do SLAM at all.
 */
bool
oxrsys_wmr_slam_available(void);

/*!
 * Sets the environment the headset must be opened with so the driver leaves
 * SLAM to us. Call before @ref oxrsys_wmr_headset_open.
 */
void
oxrsys_wmr_slam_prepare_environment(void);

/*!
 * Create and start a SLAM tracker on @p hmd (an open WMR headset device)
 * using the VIT plugin at @p vit_library_path, and route the headset's
 * cameras and IMU into it. On success the headset reports 6DoF poses from
 * `xrt_device_get_tracked_pose` and `supported.position_tracking` is set.
 */
enum oxrsys_wmr_slam_result
oxrsys_wmr_slam_create(struct xrt_device *hmd,
                       const char *vit_library_path,
                       enum u_logging_level log_level,
                       struct oxrsys_wmr_slam **out_slam);

/*!
 * The sinks the headset cameras feed. Another consumer that wants the frames
 * too must split with these rather than replace them.
 */
const struct xrt_slam_sinks *
oxrsys_wmr_slam_sinks(const struct oxrsys_wmr_slam *slam);

/*!
 * Switch the headset back to IMU-only tracking. The tracker itself lives in
 * the driver's frame context and is torn down with the headset.
 */
void
oxrsys_wmr_slam_destroy(struct oxrsys_wmr_slam **slam_ptr);

const char *
oxrsys_wmr_slam_result_str(enum oxrsys_wmr_slam_result result);

/*!
 * Point the headset's camera source at @p sinks (NULL or empty: nobody).
 *
 * The WMR source can only take new sinks by restarting the cameras, and its
 * stop only *requests* cancellation of the in-flight USB transfers; starting
 * again before libusb has finished cancelling fails with LIBUSB_ERROR_BUSY,
 * which the driver treats as fatal. This waits for the cancellations to
 * settle in between. Every camera consumer must go through here.
 */
bool
oxrsys_wmr_camera_route(struct xrt_fs *source, const struct xrt_slam_sinks *sinks);

#ifdef __cplusplus
}
#endif
