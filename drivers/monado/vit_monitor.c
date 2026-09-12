// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Monitoring shim between Monado's SLAM tracker and a VIT plugin.
 *
 * See vit_monitor.h. The real plugin is opened on first use (Monado calls
 * vit_api_get_version before anything else) from OXRSYS_VIT_REAL_LIBRARY.
 * When that fails, vit_api_get_version still answers with the header
 * version so Monado's version check passes and it is tracker creation that
 * fails, with a clear error in Monado's log, instead of a symbol lookup.
 *
 * Threading: Monado pushes IMU samples from the headset's IMU thread, images
 * from the camera thread, and pops poses from its own tracker thread, while
 * the helper reads snapshots from its main thread. Counters are atomics; the
 * pose and feature record is written and copied under one mutex.
 */

#define VIT_INTERFACE_IMPLEMENTATION
#include "vit/vit_interface.h"

#include "vit_monitor.h"

#include <dlfcn.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 *
 * The real plugin.
 *
 */

struct real_plugin
{
	void *handle;
	bool loaded;
	//! Path, or the reason loading failed.
	char description[OXRSYS_VIT_MONITOR_PATH_MAX];

	PFN_vit_api_get_version api_get_version;
	PFN_vit_tracker_create tracker_create;
	PFN_vit_tracker_destroy tracker_destroy;
	PFN_vit_tracker_has_image_format tracker_has_image_format;
	PFN_vit_tracker_get_supported_extensions tracker_get_supported_extensions;
	PFN_vit_tracker_get_enabled_extensions tracker_get_enabled_extensions;
	PFN_vit_tracker_enable_extension tracker_enable_extension;
	PFN_vit_tracker_start tracker_start;
	PFN_vit_tracker_stop tracker_stop;
	PFN_vit_tracker_reset tracker_reset;
	PFN_vit_tracker_is_running tracker_is_running;
	PFN_vit_tracker_push_imu_sample tracker_push_imu_sample;
	PFN_vit_tracker_push_img_sample tracker_push_img_sample;
	PFN_vit_tracker_add_imu_calibration tracker_add_imu_calibration;
	PFN_vit_tracker_add_camera_calibration tracker_add_camera_calibration;
	PFN_vit_tracker_pop_pose tracker_pop_pose;
	PFN_vit_tracker_get_timing_titles tracker_get_timing_titles;
	PFN_vit_pose_destroy pose_destroy;
	PFN_vit_pose_get_data pose_get_data;
	PFN_vit_pose_get_timing pose_get_timing;
	PFN_vit_pose_get_features pose_get_features;
};

static struct real_plugin g_real;
static pthread_once_t g_real_once = PTHREAD_ONCE_INIT;

static bool
real_get_proc(const char *path, const char *name, void *proc_ptr)
{
	(void)dlerror();
	void *proc = dlsym(g_real.handle, name);
	if (proc == NULL) {
		const char *err = dlerror();
		snprintf(g_real.description, sizeof(g_real.description), "%s lacks %s (%s)", path, name,
		         err != NULL ? err : "not found");
		return false;
	}
	*(void **)proc_ptr = proc;
	return true;
}

static void
real_load_once(void)
{
	const char *path = getenv(OXRSYS_VIT_MONITOR_REAL_LIBRARY_ENV);
	if (path == NULL || path[0] == '\0') {
		snprintf(g_real.description, sizeof(g_real.description), "%s is not set",
		         OXRSYS_VIT_MONITOR_REAL_LIBRARY_ENV);
		return;
	}
	g_real.handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
	if (g_real.handle == NULL) {
		const char *err = dlerror();
		snprintf(g_real.description, sizeof(g_real.description), "dlopen(%s) failed: %s", path,
		         err != NULL ? err : "unknown error");
		return;
	}

#define GET_PROC(SYM)                                                                                                  \
	do {                                                                                                           \
		if (!real_get_proc(path, "vit_" #SYM, &g_real.SYM)) {                                                  \
			dlclose(g_real.handle);                                                                        \
			g_real.handle = NULL;                                                                          \
			return;                                                                                        \
		}                                                                                                      \
	} while (0)

	GET_PROC(api_get_version);
	GET_PROC(tracker_create);
	GET_PROC(tracker_destroy);
	GET_PROC(tracker_has_image_format);
	GET_PROC(tracker_get_supported_extensions);
	GET_PROC(tracker_get_enabled_extensions);
	GET_PROC(tracker_enable_extension);
	GET_PROC(tracker_start);
	GET_PROC(tracker_stop);
	GET_PROC(tracker_reset);
	GET_PROC(tracker_is_running);
	GET_PROC(tracker_push_imu_sample);
	GET_PROC(tracker_push_img_sample);
	GET_PROC(tracker_add_imu_calibration);
	GET_PROC(tracker_add_camera_calibration);
	GET_PROC(tracker_pop_pose);
	GET_PROC(tracker_get_timing_titles);
	GET_PROC(pose_destroy);
	GET_PROC(pose_get_data);
	GET_PROC(pose_get_timing);
	GET_PROC(pose_get_features);
#undef GET_PROC

	snprintf(g_real.description, sizeof(g_real.description), "%s", path);
	g_real.loaded = true;
}

static bool
real_loaded(void)
{
	pthread_once(&g_real_once, real_load_once);
	return g_real.loaded;
}

/*
 *
 * The record.
 *
 */

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
//! Everything but the counters below; guarded by g_lock.
static struct oxrsys_vit_monitor_snapshot g_record;
//! The tracker being monitored (the first one created); guarded by g_lock.
static vit_tracker_t *g_monitored;

static _Atomic uint64_t g_imu_pushed;
static _Atomic uint64_t g_images_pushed[OXRSYS_VIT_MONITOR_MAX_CAMERAS];
static _Atomic uint64_t g_poses_popped;

static bool
is_monitored(const vit_tracker_t *tracker)
{
	pthread_mutex_lock(&g_lock);
	const bool yes = tracker != NULL && tracker == g_monitored;
	pthread_mutex_unlock(&g_lock);
	return yes;
}

static void
record_pose(vit_tracker_t *tracker, const vit_pose_t *pose)
{
	pthread_mutex_lock(&g_lock);
	if (tracker != g_monitored) {
		pthread_mutex_unlock(&g_lock);
		return;
	}

	vit_pose_data_t data;
	memset(&data, 0, sizeof(data));
	if (g_real.pose_get_data(pose, &data) == VIT_SUCCESS) {
		g_record.have_pose = true;
		g_record.pose_timestamp_ns = data.timestamp;
		g_record.px = data.px;
		g_record.py = data.py;
		g_record.pz = data.pz;
		g_record.ox = data.ox;
		g_record.oy = data.oy;
		g_record.oz = data.oz;
		g_record.ow = data.ow;
	}

	for (uint32_t cam = 0; cam < g_record.camera_count; cam++) {
		struct oxrsys_vit_monitor_camera *c = &g_record.cams[cam];
		c->feature_count = 0;
		if (!g_record.features_enabled) {
			continue;
		}
		vit_pose_features_t features;
		memset(&features, 0, sizeof(features));
		// VIT_ERROR_NOT_ENABLED comes back for poses queued before the
		// extension was switched on; those simply carry no features.
		if (g_real.pose_get_features(pose, cam, &features) != VIT_SUCCESS || features.features == NULL) {
			continue;
		}
		uint32_t n = features.count;
		if (n > OXRSYS_VIT_MONITOR_MAX_FEATURES) {
			n = OXRSYS_VIT_MONITOR_MAX_FEATURES;
		}
		for (uint32_t i = 0; i < n; i++) {
			c->features[i].id = features.features[i].id;
			c->features[i].u = features.features[i].u;
			c->features[i].v = features.features[i].v;
			c->features[i].depth = features.features[i].depth;
		}
		c->feature_count = n;
	}
	g_record.sequence++;
	pthread_mutex_unlock(&g_lock);
}

bool
oxrsys_vit_monitor_snapshot(struct oxrsys_vit_monitor_snapshot *out)
{
	if (out == NULL) {
		return false;
	}
	// Report the load state without triggering the load: the plugin is
	// opened when Monado first asks for the version.
	pthread_mutex_lock(&g_lock);
	*out = g_record;
	pthread_mutex_unlock(&g_lock);
	out->real_loaded = g_real.loaded;
	snprintf(out->real_library, sizeof(out->real_library), "%s", g_real.description);
	out->imu_samples_pushed = atomic_load(&g_imu_pushed);
	out->poses_popped = atomic_load(&g_poses_popped);
	for (uint32_t cam = 0; cam < OXRSYS_VIT_MONITOR_MAX_CAMERAS; cam++) {
		out->cams[cam].images_pushed = atomic_load(&g_images_pushed[cam]);
	}
	return true;
}

/*
 *
 * The VIT interface.
 *
 */

vit_result_t
vit_api_get_version(uint32_t *out_major, uint32_t *out_minor, uint32_t *out_patch)
{
	if (real_loaded()) {
		return g_real.api_get_version(out_major, out_minor, out_patch);
	}
	if (out_major == NULL || out_minor == NULL || out_patch == NULL) {
		return VIT_ERROR_INVALID_VALUE;
	}
	*out_major = VIT_HEADER_VERSION_MAJOR;
	*out_minor = VIT_HEADER_VERSION_MINOR;
	*out_patch = VIT_HEADER_VERSION_PATCH;
	return VIT_SUCCESS;
}

vit_result_t
vit_tracker_create(const vit_config_t *config, vit_tracker_t **out_tracker)
{
	if (out_tracker == NULL) {
		return VIT_ERROR_INVALID_VALUE;
	}
	*out_tracker = NULL;
	if (!real_loaded()) {
		fprintf(stderr, "oxrsys-vit-monitor: no VIT plugin to forward to: %s\n", g_real.description);
		return VIT_ERROR_NOT_SUPPORTED;
	}
	vit_result_t res = g_real.tracker_create(config, out_tracker);
	if (res != VIT_SUCCESS || *out_tracker == NULL) {
		return res;
	}
	vit_tracker_t *tracker = *out_tracker;

	// Features per camera come from the pose-features extension, which
	// Monado does not enable itself. Enable it while the tracker is not
	// started; a plugin without it still works, just without boxes.
	bool features = false;
	vit_tracker_extension_set_t exts;
	memset(&exts, 0, sizeof(exts));
	if (g_real.tracker_get_supported_extensions(tracker, &exts) == VIT_SUCCESS && exts.has_pose_features) {
		features = g_real.tracker_enable_extension(tracker, VIT_TRACKER_EXTENSION_POSE_FEATURES, true) == VIT_SUCCESS;
	}

	pthread_mutex_lock(&g_lock);
	if (g_monitored == NULL) {
		g_monitored = tracker;
		memset(&g_record, 0, sizeof(g_record));
		g_record.tracker_created = true;
		g_record.features_enabled = features;
		g_record.camera_count = config != NULL ? config->cam_count : 0;
		if (g_record.camera_count > OXRSYS_VIT_MONITOR_MAX_CAMERAS) {
			g_record.camera_count = OXRSYS_VIT_MONITOR_MAX_CAMERAS;
		}
		atomic_store(&g_imu_pushed, 0);
		atomic_store(&g_poses_popped, 0);
		for (uint32_t cam = 0; cam < OXRSYS_VIT_MONITOR_MAX_CAMERAS; cam++) {
			atomic_store(&g_images_pushed[cam], 0);
		}
	}
	pthread_mutex_unlock(&g_lock);
	return VIT_SUCCESS;
}

void
vit_tracker_destroy(vit_tracker_t *tracker)
{
	if (!real_loaded() || tracker == NULL) {
		return;
	}
	pthread_mutex_lock(&g_lock);
	if (tracker == g_monitored) {
		g_monitored = NULL;
		memset(&g_record, 0, sizeof(g_record));
	}
	pthread_mutex_unlock(&g_lock);
	g_real.tracker_destroy(tracker);
}

vit_result_t
vit_tracker_has_image_format(const vit_tracker_t *tracker, vit_image_format_t image_format, bool *out_supported)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_has_image_format(tracker, image_format, out_supported);
}

vit_result_t
vit_tracker_get_supported_extensions(const vit_tracker_t *tracker, vit_tracker_extension_set_t *out_exts)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_get_supported_extensions(tracker, out_exts);
}

vit_result_t
vit_tracker_get_enabled_extensions(const vit_tracker_t *tracker, vit_tracker_extension_set_t *out_exts)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_get_enabled_extensions(tracker, out_exts);
}

vit_result_t
vit_tracker_enable_extension(vit_tracker_t *tracker, vit_tracker_extension_t ext, bool value)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	vit_result_t res = g_real.tracker_enable_extension(tracker, ext, value);
	if (res == VIT_SUCCESS && ext == VIT_TRACKER_EXTENSION_POSE_FEATURES) {
		// Monado's debug UI can toggle it too; follow whatever it did.
		pthread_mutex_lock(&g_lock);
		if (tracker == g_monitored) {
			g_record.features_enabled = value;
		}
		pthread_mutex_unlock(&g_lock);
	}
	return res;
}

vit_result_t
vit_tracker_start(vit_tracker_t *tracker)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_start(tracker);
}

vit_result_t
vit_tracker_stop(vit_tracker_t *tracker)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_stop(tracker);
}

vit_result_t
vit_tracker_reset(vit_tracker_t *tracker)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_reset(tracker);
}

vit_result_t
vit_tracker_is_running(const vit_tracker_t *tracker, bool *out_bool)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_is_running(tracker, out_bool);
}

vit_result_t
vit_tracker_push_imu_sample(vit_tracker_t *tracker, const vit_imu_sample_t *sample)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	vit_result_t res = g_real.tracker_push_imu_sample(tracker, sample);
	if (res == VIT_SUCCESS && is_monitored(tracker)) {
		atomic_fetch_add(&g_imu_pushed, 1);
	}
	return res;
}

vit_result_t
vit_tracker_push_img_sample(vit_tracker_t *tracker, const vit_img_sample_t *sample)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	vit_result_t res = g_real.tracker_push_img_sample(tracker, sample);
	if (res == VIT_SUCCESS && sample != NULL && sample->cam_index < OXRSYS_VIT_MONITOR_MAX_CAMERAS &&
	    is_monitored(tracker)) {
		atomic_fetch_add(&g_images_pushed[sample->cam_index], 1);
	}
	return res;
}

vit_result_t
vit_tracker_add_imu_calibration(vit_tracker_t *tracker, const vit_imu_calibration_t *calibration)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_add_imu_calibration(tracker, calibration);
}

vit_result_t
vit_tracker_add_camera_calibration(vit_tracker_t *tracker, const vit_camera_calibration_t *calibration)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_add_camera_calibration(tracker, calibration);
}

vit_result_t
vit_tracker_pop_pose(vit_tracker_t *tracker, vit_pose_t **out_pose)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	vit_result_t res = g_real.tracker_pop_pose(tracker, out_pose);
	// With out_pose NULL the plugin destroys the pose itself; nothing to see.
	if (res == VIT_SUCCESS && out_pose != NULL && *out_pose != NULL) {
		record_pose(tracker, *out_pose);
		if (is_monitored(tracker)) {
			atomic_fetch_add(&g_poses_popped, 1);
		}
	}
	return res;
}

vit_result_t
vit_tracker_get_timing_titles(const vit_tracker_t *tracker, vit_tracker_timing_titles *out_titles)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.tracker_get_timing_titles(tracker, out_titles);
}

void
vit_pose_destroy(vit_pose_t *pose)
{
	if (!real_loaded() || pose == NULL) {
		return;
	}
	g_real.pose_destroy(pose);
}

vit_result_t
vit_pose_get_data(const vit_pose_t *pose, vit_pose_data_t *out_data)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.pose_get_data(pose, out_data);
}

vit_result_t
vit_pose_get_timing(const vit_pose_t *pose, vit_pose_timing_t *out_timing)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.pose_get_timing(pose, out_timing);
}

vit_result_t
vit_pose_get_features(const vit_pose_t *pose, uint32_t camera_index, vit_pose_features_t *out_features)
{
	if (!real_loaded()) {
		return VIT_ERROR_NOT_SUPPORTED;
	}
	return g_real.pose_get_features(pose, camera_index, out_features);
}
