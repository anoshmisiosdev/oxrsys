// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Track a PlayStation Move sphere with the WMR headset's cameras.
 *
 * Builds a stereo calibration from the headset's own camera calibration,
 * creates Monado's PS Move sphere tracker on it, combines the headset's two
 * head-tracking camera streams into one side-by-side frame for the tracker,
 * and exposes an xrt_tracking_factory that the PS Move driver asks for a
 * tracked sphere when a controller is created.
 *
 * The cameras are monochrome, so the sphere is lit white and segmented with
 * the HSV filter's "white" bucket; only one sphere can be told apart at a
 * time. Positions are in the first camera's frame, which rides on the head.
 *
 * Only functional when the driver library was built with OpenCV
 * (OXRSYS_WMR_OPENCV); otherwise the create call returns NULL.
 */

#pragma once

#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

struct oxrsys_wmr_psmv_tracking;

/*!
 * Create the tracker on @p hmd (an xrt_device created by the WMR driver) and
 * start feeding it camera frames.
 *
 * @return The tracking factory to hand to the PS Move driver, or NULL when
 * OpenCV is not built in or the headset has no usable tracking cameras.
 */
struct xrt_tracking_factory *
oxrsys_wmr_psmv_tracking_create(struct xrt_device *hmd,
                                enum u_logging_level log_level,
                                struct oxrsys_wmr_psmv_tracking **out_tracking);

//! Stop the trackers and release everything. Destroy PS Move devices first.
void
oxrsys_wmr_psmv_tracking_destroy(struct oxrsys_wmr_psmv_tracking **tracking_ptr);

//! True when the library can track spheres (built with OpenCV).
bool
oxrsys_wmr_psmv_tracking_available(void);

#ifdef __cplusplus
}
#endif
