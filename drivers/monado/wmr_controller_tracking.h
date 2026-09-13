// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Optical (LED constellation) tracking of 1st-gen WMR motion controllers
 *        with the headset's own tracking cameras.
 *
 * WMR headsets alternate their camera exposures: one long exposure for SLAM,
 * then two very short ones in which only the controllers' infrared LEDs show
 * up. Monado's driver already splits the short "controller" frames onto camera
 * sinks 2 and 3; this module consumes them.
 *
 * - LED sync: the controllers only pulse their LEDs during those short
 *   exposures when the host keeps telling them when the next exposure happens
 *   (in the controller's own clock) over Bluetooth. Upstream Monado does not
 *   do that yet, so this module listens to each controller's IMU reports to
 *   track its clock and sends the timesync and keepalive packets itself.
 * - Blobs: Monado's generic blob detector (t_rift_blobwatch) finds the LEDs.
 * - Poses: Monado's generic constellation tracker matches the blobs to each
 *   controller's LED model (read from the controller's calibration) and
 *   solves the controller pose in the headset's frame.
 *
 * Without OpenCV (OXRSYS_WMR_OPENCV=OFF) create() returns NULL.
 */

#pragma once

#include "util/u_logging.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OXRSYS_WMR_CT_CAMERAS 2
#define OXRSYS_WMR_CT_MAX_BLOBS 64

struct oxrsys_wmr_controller_tracking;

//! One detected LED blob in a controller frame, in camera pixels.
struct oxrsys_wmr_ct_blob
{
	float x, y; //!< Centre.
	float w, h; //!< Bounding box size.
	//! 0 = left controller, 1 = right controller, -1 = not (yet) assigned.
	int8_t hand;
};

struct oxrsys_wmr_ct_camera_view
{
	//! Timestamp of the latest controller frame's blobs; 0 = none yet.
	int64_t timestamp_ns;
	//! Blobs the detector found in that frame.
	uint32_t blob_count;
	//! Entries filled in `blobs` (at most OXRSYS_WMR_CT_MAX_BLOBS).
	uint32_t stored;
	struct oxrsys_wmr_ct_blob blobs[OXRSYS_WMR_CT_MAX_BLOBS];
};

struct oxrsys_wmr_ct_hand
{
	//! The controller is registered with the tracker.
	bool present;
	//! LED sync: controller clock locked and timesync packets are going out.
	bool led_sync;
	uint16_t led_intensity;
	uint64_t timesyncs_sent;

	//! Latest optical pose, in the headset's frame (OpenXR axes, metres).
	bool pose_valid;
	int64_t pose_timestamp_ns;
	struct xrt_pose head_relative;
	uint32_t pose_camera;
	uint32_t matched_blobs;
	uint32_t visible_leds;
	float reprojection_error;
	//! Optical poses per second, averaged over the last second.
	float poses_per_second;
	//! Per camera: last time the controller's LEDs were identified in it.
	int64_t last_seen_ns[OXRSYS_WMR_CT_CAMERAS];
};

struct oxrsys_wmr_ct_snapshot
{
	uint32_t camera_count;
	struct oxrsys_wmr_ct_camera_view cams[OXRSYS_WMR_CT_CAMERAS];
	struct oxrsys_wmr_ct_hand hands[2];
	//! Controller (short exposure) frames received from the headset.
	uint64_t controller_frames;
	float controller_fps;
};

/*!
 * Start tracking @p left and/or @p right (WMR motion controllers, either may
 * be NULL) with the cameras of @p hmd (an open WMR headset). @p share_with is
 * what the cameras feed already (SLAM tracker, camera tap); those keep every
 * frame. Returns NULL when tracking is unavailable; the log says why.
 */
struct oxrsys_wmr_controller_tracking *
oxrsys_wmr_controller_tracking_create(struct xrt_device *hmd,
                                      struct xrt_device *left,
                                      struct xrt_device *right,
                                      const struct xrt_slam_sinks *share_with,
                                      enum u_logging_level log_level);

/*!
 * What the cameras feed now; pass it as `share_with` to consumers attached
 * later.
 */
const struct xrt_slam_sinks *
oxrsys_wmr_controller_tracking_sinks(const struct oxrsys_wmr_controller_tracking *t);

/*!
 * Latest optical pose of @p hand (0 left, 1 right) in the headset's frame,
 * if one is younger than @p max_age_ns at monotonic time @p now_ns.
 */
bool
oxrsys_wmr_controller_tracking_get_pose(struct oxrsys_wmr_controller_tracking *t,
                                        int hand,
                                        int64_t now_ns,
                                        int64_t max_age_ns,
                                        struct xrt_pose *out_head_relative,
                                        int64_t *out_timestamp_ns);

/*!
 * Called on the camera thread with every controller (short exposure) frame,
 * 8-bit grayscale, one per camera; @p frame is only valid during the call.
 * For display. NULL removes it.
 */
typedef void (*oxrsys_wmr_ct_frame_cb)(void *userdata, uint32_t cam_index, const struct xrt_frame *frame);
void
oxrsys_wmr_controller_tracking_set_frame_callback(struct oxrsys_wmr_controller_tracking *t,
                                                  oxrsys_wmr_ct_frame_cb callback,
                                                  void *userdata);

//! Copy the current state for display. Any thread.
void
oxrsys_wmr_controller_tracking_snapshot(struct oxrsys_wmr_controller_tracking *t,
                                        struct oxrsys_wmr_ct_snapshot *out);

/*!
 * Stop tracking and free everything; the cameras go back to feeding what they
 * fed before. Consumers attached after this one must be destroyed first.
 */
void
oxrsys_wmr_controller_tracking_destroy(struct oxrsys_wmr_controller_tracking **t_ptr);

//! Whether this build has controller tracking (needs OpenCV).
bool
oxrsys_wmr_controller_tracking_available(void);

/*
 * Pieces exposed for drivers/tools/wmr_ct_selftest.c.
 */

struct wmr_led_config;
struct t_constellation_tracker_led;
struct t_constellation_tracker_led_model;

/*!
 * Build the constellation tracker's LED model from a controller calibration's
 * LEDs (@p count entries, at most WMR_MAX_LEDS). @p out_leds must hold @p count
 * entries and stays referenced by @p out_model.
 */
void
oxrsys_wmr_ct_build_led_model(const struct wmr_led_config *leds,
                              size_t count,
                              struct t_constellation_tracker_led *out_leds,
                              struct t_constellation_tracker_led_model *out_model);

/*!
 * The 12-byte LED timesync packet (report 0x03): command counter, 1..3 sync
 * counter, LED intensity (1..399), controller-clock time of the next
 * controller exposure in microseconds, the unknown 11-bit U2 and the flags.
 */
void
oxrsys_wmr_ct_fill_timesync_packet(
    uint8_t buf[12], uint8_t cmd_ctr, uint8_t ts_ctr, int led_intensity, uint64_t ts_us, int u2, uint8_t flags);

#ifdef __cplusplus
}
#endif
