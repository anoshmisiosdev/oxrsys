// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Monitoring shim between Monado's SLAM tracker and a VIT plugin.
 *
 * liboxrsys-vit-monitor.dylib implements the VIT plugin interface
 * (vit/vit_interface.h) by forwarding every call to the real plugin (Basalt)
 * named by the OXRSYS_VIT_REAL_LIBRARY environment variable, and on the way
 * records what the tracker is doing: the features it matched in each camera
 * image, its latest pose, and how much data it was fed. Monado loads the shim
 * like any plugin; the helper reads the record through
 * oxrsys_vit_monitor_snapshot() (looked up with dlsym on the same library)
 * and draws it over the camera images.
 *
 * Plain C, no Monado dependency: only this header is shared with the helper.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OXRSYS_VIT_MONITOR_MAX_CAMERAS 2
#define OXRSYS_VIT_MONITOR_MAX_FEATURES 512
#define OXRSYS_VIT_MONITOR_PATH_MAX 1024

//! Name of the environment variable holding the real plugin's path.
#define OXRSYS_VIT_MONITOR_REAL_LIBRARY_ENV "OXRSYS_VIT_REAL_LIBRARY"

//! One feature the tracker used, as vit_pose_feature: image position in
//! pixels of that camera's frame and the estimated depth in metres.
struct oxrsys_vit_monitor_feature
{
	int64_t id;
	float u, v, depth;
};

struct oxrsys_vit_monitor_camera
{
	//! Image samples pushed into the tracker for this camera.
	uint64_t images_pushed;
	//! Features reported with the latest pose (0 when the extension is off
	//! or the pose carried none).
	uint32_t feature_count;
	struct oxrsys_vit_monitor_feature features[OXRSYS_VIT_MONITOR_MAX_FEATURES];
};

struct oxrsys_vit_monitor_snapshot
{
	//! The real plugin loaded (false: every forwarded call fails).
	bool real_loaded;
	//! What OXRSYS_VIT_REAL_LIBRARY named, or the dlopen error when
	//! real_loaded is false.
	char real_library[OXRSYS_VIT_MONITOR_PATH_MAX];
	//! A tracker exists (vit_tracker_create succeeded and it was not destroyed).
	bool tracker_created;
	//! The pose-features extension could be enabled on the real tracker.
	bool features_enabled;

	//! Incremented whenever the pose or the feature lists change; a reader
	//! can skip work when it has not moved.
	uint64_t sequence;
	uint64_t imu_samples_pushed;
	uint64_t poses_popped;

	//! Latest pose popped by Monado (valid when poses_popped > 0).
	bool have_pose;
	int64_t pose_timestamp_ns;
	float px, py, pz;
	float ox, oy, oz, ow;

	uint32_t camera_count;
	struct oxrsys_vit_monitor_camera cams[OXRSYS_VIT_MONITOR_MAX_CAMERAS];
};

/*!
 * Copy the current record into @p out. Safe from any thread at any time,
 * also while the tracker thread is popping poses. Returns false only when
 * @p out is NULL.
 */
bool
oxrsys_vit_monitor_snapshot(struct oxrsys_vit_monitor_snapshot *out);

//! Signature of the above, for dlsym.
typedef bool (*oxrsys_vit_monitor_snapshot_fn)(struct oxrsys_vit_monitor_snapshot *out);

#ifdef __cplusplus
}
#endif
