// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Observe the frames a WMR headset's tracking cameras deliver.
 */

#include "wmr_camera_tap.h"

#include <string.h>

#include "util/u_misc.h"
#include "util/u_sink.h"
#include "wmr/wmr_hmd.h"
#include "wmr_slam.h"
#include "xrt/xrt_frameserver.h"

#define TAP_MAX_CAMS 2

struct tap_sink
{
	struct xrt_frame_sink base;
	struct oxrsys_wmr_camera_tap *tap;
	uint32_t cam_index;
};

struct oxrsys_wmr_camera_tap
{
	struct xrt_frame_context xfctx;
	struct xrt_fs *source;
	struct tap_sink sinks[TAP_MAX_CAMS];
	uint32_t cam_count;
	//! What the source fed before the tap; restored on destroy.
	struct xrt_slam_sinks shared;
	//! What the source feeds with the tap in place.
	struct xrt_slam_sinks current;
	oxrsys_wmr_camera_tap_cb callback;
	void *userdata;
	enum u_logging_level log_level;
};

static void
tap_push_frame(struct xrt_frame_sink *xfs, struct xrt_frame *xf)
{
	struct tap_sink *s = (struct tap_sink *)xfs;
	if (s->tap->callback != NULL) {
		s->tap->callback(s->tap->userdata, s->cam_index, xf);
	}
}

struct oxrsys_wmr_camera_tap *
oxrsys_wmr_camera_tap_create(struct xrt_device *hmd,
                             const struct xrt_slam_sinks *share_with,
                             oxrsys_wmr_camera_tap_cb callback,
                             void *userdata,
                             enum u_logging_level log_level)
{
	if (hmd == NULL || hmd->name != XRT_DEVICE_GENERIC_HMD) {
		return NULL;
	}
	struct wmr_hmd *wh = wmr_hmd(hmd);
	if (wh->tracking.source == NULL || wh->config.tcam_count < 1) {
		U_LOG_IFL_W(log_level, "Headset has no tracking cameras to tap.");
		return NULL;
	}

	struct oxrsys_wmr_camera_tap *t = U_TYPED_CALLOC(struct oxrsys_wmr_camera_tap);
	t->callback = callback;
	t->userdata = userdata;
	t->log_level = log_level;
	t->cam_count = (uint32_t)wh->config.tcam_count;
	if (t->cam_count > TAP_MAX_CAMS) {
		t->cam_count = TAP_MAX_CAMS;
	}
	if (share_with != NULL) {
		t->shared = *share_with;
		t->current = *share_with;
	}

	for (uint32_t i = 0; i < t->cam_count; i++) {
		t->sinks[i].base.push_frame = tap_push_frame;
		t->sinks[i].tap = t;
		t->sinks[i].cam_index = i;
		struct xrt_frame_sink *ours = &t->sinks[i].base;
		struct xrt_frame_sink *out = ours;
		if (t->shared.cams[i] != NULL) {
			// The existing consumer stays first so its latency is unchanged.
			u_sink_split_create(&t->xfctx, t->shared.cams[i], ours, &out);
		}
		t->current.cams[i] = out;
	}
	if ((int)t->cam_count > t->current.cam_count) {
		t->current.cam_count = (int)t->cam_count;
	}

	if (!oxrsys_wmr_camera_route(wh->tracking.source, &t->current)) {
		U_LOG_IFL_E(log_level, "Could not attach the camera tap to the headset cameras.");
		xrt_frame_context_destroy_nodes(&t->xfctx);
		free(t);
		return NULL;
	}
	t->source = wh->tracking.source;
	U_LOG_IFL_D(log_level, "Camera tap on %u camera(s).", t->cam_count);
	return t;
}

const struct xrt_slam_sinks *
oxrsys_wmr_camera_tap_sinks(const struct oxrsys_wmr_camera_tap *tap)
{
	return tap != NULL ? &tap->current : NULL;
}

uint32_t
oxrsys_wmr_camera_tap_count(const struct oxrsys_wmr_camera_tap *tap)
{
	return tap != NULL ? tap->cam_count : 0;
}

void
oxrsys_wmr_camera_tap_destroy(struct oxrsys_wmr_camera_tap **tap_ptr)
{
	struct oxrsys_wmr_camera_tap *t = *tap_ptr;
	if (t == NULL) {
		return;
	}
	if (t->source != NULL) {
		oxrsys_wmr_camera_route(t->source, &t->shared);
		t->source = NULL;
	}
	t->callback = NULL;
	xrt_frame_context_destroy_nodes(&t->xfctx);
	free(t);
	*tap_ptr = NULL;
}
