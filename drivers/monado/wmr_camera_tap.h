// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Observe the frames a WMR headset's tracking cameras deliver.
 *
 * A tap sits between the headset's camera source and whatever already
 * consumes the frames (the SLAM tracker, the PS Move sphere tracker) and
 * hands every frame to a callback as well: for a monitor window, for
 * snapshots, for diagnostics. It never alters the frames.
 */

#pragma once

#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

struct oxrsys_wmr_camera_tap;

/*!
 * Called on the camera reader thread for every frame of every camera; keep
 * it short (copy the pixels out). @p frame is only valid during the call.
 * Frames are 8-bit grayscale (XRT_FORMAT_L8), one per camera, with the
 * headset's timestamp in `frame->timestamp` (monotonic clock, ns).
 */
typedef void (*oxrsys_wmr_camera_tap_cb)(void *userdata, uint32_t cam_index, const struct xrt_frame *frame);

/*!
 * Create the tap on @p hmd (an open WMR headset device) and start receiving
 * frames. @p share_with is what the cameras currently feed (for example
 * oxrsys_wmr_slam_sinks()); those keep receiving every frame. NULL when
 * nothing else consumes them.
 */
struct oxrsys_wmr_camera_tap *
oxrsys_wmr_camera_tap_create(struct xrt_device *hmd,
                             const struct xrt_slam_sinks *share_with,
                             oxrsys_wmr_camera_tap_cb callback,
                             void *userdata,
                             enum u_logging_level log_level);

/*!
 * What the cameras feed now that the tap is in place. Pass this as
 * `share_with` to any consumer attached after the tap.
 */
const struct xrt_slam_sinks *
oxrsys_wmr_camera_tap_sinks(const struct oxrsys_wmr_camera_tap *tap);

/*!
 * Number of cameras the tap delivers (0 when the headset has none).
 */
uint32_t
oxrsys_wmr_camera_tap_count(const struct oxrsys_wmr_camera_tap *tap);

/*!
 * Stop and free the tap; the cameras go back to feeding what they fed
 * before. Consumers attached after the tap must be destroyed first.
 */
void
oxrsys_wmr_camera_tap_destroy(struct oxrsys_wmr_camera_tap **tap_ptr);

#ifdef __cplusplus
}
#endif
