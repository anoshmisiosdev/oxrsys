// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  6DoF head tracking for a WMR headset through Monado's SLAM tracker.
 */

#include "wmr_slam.h"

#include "xrt/xrt_config_build.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "os/os_time.h"
#include "util/u_misc.h"
#include "wmr/wmr_hmd.h"
#include "xrt/xrt_frameserver.h"

#ifdef XRT_FEATURE_SLAM
#include "tracking/t_tracking.h"
#endif

bool
oxrsys_wmr_camera_route(struct xrt_fs *source, const struct xrt_slam_sinks *sinks)
{
	struct xrt_slam_sinks copy;
	memset(&copy, 0, sizeof(copy));
	if (sinks != NULL) {
		copy = *sinks;
	}
	xrt_fs_stream_stop(source);
	// wmr_camera_stop() only calls libusb_cancel_transfer(); the transfers
	// are reusable once their callbacks have run on the USB event thread.
	// A few event-loop turns is plenty; 150 ms leaves a wide margin.
	os_nanosleep(150 * U_TIME_1MS_IN_NS);
	return xrt_fs_slam_stream_start(source, &copy);
}

struct oxrsys_wmr_slam
{
	struct wmr_hmd *wh;
	struct xrt_slam_sinks sinks;
	enum u_logging_level log_level;
};

bool
oxrsys_wmr_slam_available(void)
{
#ifdef XRT_FEATURE_SLAM
	return true;
#else
	return false;
#endif
}

void
oxrsys_wmr_slam_prepare_environment(void)
{
	// With SLAM compiled in, the driver would otherwise fail headset
	// creation whenever its own tracker cannot start.
	setenv("WMR_SLAM", "false", 1);
}

enum oxrsys_wmr_slam_result
oxrsys_wmr_slam_create(struct xrt_device *hmd,
                       const char *vit_library_path,
                       enum u_logging_level log_level,
                       struct oxrsys_wmr_slam **out_slam)
{
	*out_slam = NULL;
#ifndef XRT_FEATURE_SLAM
	(void)hmd;
	(void)vit_library_path;
	(void)log_level;
	return OXRSYS_WMR_SLAM_NOT_BUILT;
#else
	if (hmd == NULL || hmd->name != XRT_DEVICE_GENERIC_HMD) {
		return OXRSYS_WMR_SLAM_NO_CAMERAS;
	}
	struct wmr_hmd *wh = wmr_hmd(hmd);
	if (wh->tracking.source == NULL || wh->config.slam_cam_count < 2) {
		U_LOG_IFL_W(log_level, "This headset exposes %d tracking cameras; Basalt needs at least two.",
		            wh->config.slam_cam_count);
		return OXRSYS_WMR_SLAM_NO_CAMERAS;
	}
	if (wh->tracking.slam != NULL) {
		U_LOG_IFL_W(log_level, "The driver already runs a SLAM tracker.");
		return OXRSYS_WMR_SLAM_CREATE_FAILED;
	}

	struct oxrsys_wmr_slam *s = U_TYPED_CALLOC(struct oxrsys_wmr_slam);
	s->wh = wh;
	s->log_level = log_level;

	// Same setup as the driver's wmr_hmd_slam_track(), with the plugin path
	// given explicitly instead of read from VIT_SYSTEM_LIBRARY_PATH.
	struct t_slam_tracker_config config;
	memset(&config, 0, sizeof(config));
	t_slam_fill_default_config(&config); // log level and knobs from SLAM_* env
	config.vit_system_library_path = vit_library_path;
	config.cam_count = wh->config.slam_cam_count;
	wh->tracking.slam_calib.cam_count = wh->config.slam_cam_count;
	config.slam_calib = &wh->tracking.slam_calib;
	if (getenv("SLAM_SUBMIT_FROM_START") == NULL) {
		config.submit_from_start = true;
	}

	struct xrt_slam_sinks *sinks = NULL;
	int status = t_slam_create(&wh->tracking.xfctx, &config, &wh->tracking.slam, &sinks);
	if (status != 0 || sinks == NULL || wh->tracking.slam == NULL) {
		U_LOG_IFL_E(log_level, "Monado could not create the SLAM tracker with '%s' (%d).", vit_library_path,
		            status);
		wh->tracking.slam = NULL;
		free(s);
		return OXRSYS_WMR_SLAM_CREATE_FAILED;
	}
	status = t_slam_start(wh->tracking.slam);
	if (status != 0) {
		U_LOG_IFL_E(log_level, "Monado could not start the SLAM tracker (%d).", status);
		wh->tracking.slam = NULL; // The node is destroyed with the frame context.
		free(s);
		return OXRSYS_WMR_SLAM_START_FAILED;
	}
	s->sinks = *sinks;

	// The driver started the cameras with empty sinks; re-point them at the
	// tracker.
	if (!oxrsys_wmr_camera_route(wh->tracking.source, &s->sinks)) {
		U_LOG_IFL_E(log_level, "Could not route the headset cameras into the SLAM tracker.");
		wh->tracking.slam = NULL;
		free(s);
		return OXRSYS_WMR_SLAM_CAMERA_FAILED;
	}

	wh->tracking.slam_enabled = true;
	wh->slam_over_3dof = true;
	wh->base.supported.position_tracking = true;
	snprintf(wh->gui.slam_status, sizeof(wh->gui.slam_status), "Enabled (OXRSys, %s)", vit_library_path);

	U_LOG_IFL_I(log_level, "6DoF head tracking: %d cameras into %s.", config.cam_count, vit_library_path);
	*out_slam = s;
	return OXRSYS_WMR_SLAM_OK;
#endif
}

const struct xrt_slam_sinks *
oxrsys_wmr_slam_sinks(const struct oxrsys_wmr_slam *slam)
{
	return slam != NULL ? &slam->sinks : NULL;
}

void
oxrsys_wmr_slam_destroy(struct oxrsys_wmr_slam **slam_ptr)
{
	struct oxrsys_wmr_slam *s = *slam_ptr;
	if (s == NULL) {
		return;
	}
#ifdef XRT_FEATURE_SLAM
	struct wmr_hmd *wh = s->wh;
	wh->slam_over_3dof = false;
	wh->tracking.slam_enabled = false;
	wh->base.supported.position_tracking = false;
	// The tracker node and its sinks are destroyed with wh->tracking.xfctx
	// when the headset closes; stop feeding it frames now.
	oxrsys_wmr_camera_route(wh->tracking.source, NULL);
#endif
	free(s);
	*slam_ptr = NULL;
}

const char *
oxrsys_wmr_slam_result_str(enum oxrsys_wmr_slam_result result)
{
	switch (result) {
	case OXRSYS_WMR_SLAM_OK: return "ok";
	case OXRSYS_WMR_SLAM_NOT_BUILT: return "not built (needs OXRSYS_WMR_OPENCV)";
	case OXRSYS_WMR_SLAM_NO_CAMERAS: return "headset has no usable tracking cameras";
	case OXRSYS_WMR_SLAM_CREATE_FAILED: return "tracker creation failed (see log)";
	case OXRSYS_WMR_SLAM_START_FAILED: return "tracker start failed";
	case OXRSYS_WMR_SLAM_CAMERA_FAILED: return "could not route cameras";
	}
	return "unknown";
}
