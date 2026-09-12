// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Track a PlayStation Move sphere with the WMR headset's cameras.
 */

#include "wmr_psmv_tracking.h"

#include "xrt/xrt_config_have.h"

#include "util/u_misc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef XRT_HAVE_OPENCV

#include "tracking/t_tracking.h"
#include "util/u_sink.h"
#include "wmr/wmr_config.h"
#include "wmr/wmr_hmd.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"

#define MAX_TRACKED 2

struct oxrsys_wmr_psmv_tracking
{
	struct xrt_tracking_factory base;
	struct xrt_frame_context xfctx;
	struct xrt_tracking_origin origin;
	struct t_stereo_camera_calibration *calib;
	struct xrt_tracked_psmv *xtmv[MAX_TRACKED];
	size_t handed_out;
	enum u_logging_level log_level;
	//! The headset's camera source our sinks are attached to.
	struct xrt_fs *source;
};

static struct oxrsys_wmr_psmv_tracking *
from_factory(struct xrt_tracking_factory *xtf)
{
	return (struct oxrsys_wmr_psmv_tracking *)xtf;
}

/*
 * Same numbers wmr_hmd.c derives for its own trackers (wmr_hmd_get_cam_calib
 * / wmr_hmd_create_stereo_camera_calib are static there).
 */
static struct t_camera_calibration
camera_calibration(const struct wmr_camera_config *cam)
{
	struct t_camera_calibration res;
	memset(&res, 0, sizeof(res));
	const struct wmr_distortion_6KT *intr = &cam->distortion6KT;

	res.image_size_pixels.h = cam->roi.extent.h;
	res.image_size_pixels.w = cam->roi.extent.w;
	res.intrinsics[0][0] = intr->params.fx * (double)cam->roi.extent.w;
	res.intrinsics[1][1] = intr->params.fy * (double)cam->roi.extent.h;
	res.intrinsics[0][2] = intr->params.cx * (double)cam->roi.extent.w;
	res.intrinsics[1][2] = intr->params.cy * (double)cam->roi.extent.h;
	res.intrinsics[2][2] = 1.0;

	res.distortion_model = T_DISTORTION_WMR;
	res.wmr.k1 = intr->params.k[0];
	res.wmr.k2 = intr->params.k[1];
	res.wmr.p1 = intr->params.p1;
	res.wmr.p2 = intr->params.p2;
	res.wmr.k3 = intr->params.k[2];
	res.wmr.k4 = intr->params.k[3];
	res.wmr.k5 = intr->params.k[4];
	res.wmr.k6 = intr->params.k[5];
	res.wmr.codx = intr->params.dist_x;
	res.wmr.cody = intr->params.dist_y;
	res.wmr.rpmax = intr->params.metric_radius;
	return res;
}

static struct t_stereo_camera_calibration *
stereo_calibration(struct wmr_hmd *wh)
{
	struct t_stereo_camera_calibration *calib = NULL;
	t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_WMR);
	for (int i = 0; i < 2; i++) {
		calib->view[i] = camera_calibration(wh->config.tcams[i]);
	}
	const struct wmr_camera_config *ht1 = &wh->config.cams[1];
	calib->camera_translation[0] = ht1->translation.x;
	calib->camera_translation[1] = ht1->translation.y;
	calib->camera_translation[2] = ht1->translation.z;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			calib->camera_rotation[r][c] = ht1->rotation.v[r * 3 + c];
		}
	}
	return calib;
}

static int
create_tracked_psmv(struct xrt_tracking_factory *xtf, struct xrt_tracked_psmv **out_xtmv)
{
	struct oxrsys_wmr_psmv_tracking *t = from_factory(xtf);
	if (t->handed_out >= MAX_TRACKED || t->xtmv[t->handed_out] == NULL) {
		return -1;
	}
	struct xrt_tracked_psmv *xtmv = t->xtmv[t->handed_out++];
	t_psmv_start(xtmv);
	*out_xtmv = xtmv;
	return 0;
}

static int
create_tracked_psvr(struct xrt_tracking_factory *xtf, struct xrt_tracked_psvr **out)
{
	(void)xtf;
	(void)out;
	return -1;
}

static int
create_tracked_slam(struct xrt_tracking_factory *xtf, struct xrt_tracked_slam **out)
{
	(void)xtf;
	(void)out;
	return -1;
}

struct xrt_tracking_factory *
oxrsys_wmr_psmv_tracking_create(struct xrt_device *hmd,
                                enum u_logging_level log_level,
                                struct oxrsys_wmr_psmv_tracking **out_tracking)
{
	*out_tracking = NULL;
	struct wmr_hmd *wh = wmr_hmd(hmd);
	if (wh->config.tcam_count < 2 || wh->tracking.source == NULL) {
		U_LOG_IFL_W(log_level, "Headset has %d tracking cameras; sphere tracking needs two.",
		            wh->config.tcam_count);
		return NULL;
	}

	struct oxrsys_wmr_psmv_tracking *t = U_TYPED_CALLOC(struct oxrsys_wmr_psmv_tracking);
	t->log_level = log_level;
	t->base.xfctx = &t->xfctx;
	t->base.create_tracked_psmv = create_tracked_psmv;
	t->base.create_tracked_psvr = create_tracked_psvr;
	t->base.create_tracked_slam = create_tracked_slam;
	t->origin.type = XRT_TRACKING_TYPE_RGB;
	t->origin.initial_offset.orientation.w = 1.0f;
	snprintf(t->origin.name, sizeof(t->origin.name), "WMR camera PS Move tracking");

	t->calib = stereo_calibration(wh);

	// Monochrome cameras: light the sphere white and use the filter's
	// "white" bucket. Only one sphere is distinguishable; the second tracker
	// exists so a second Move still gets a device, it just never sees frames.
	struct xrt_colour_rgb_f32 white = {1.0f, 1.0f, 1.0f};
	struct xrt_frame_sink *psmv_sinks[MAX_TRACKED] = {NULL, NULL};
	for (size_t i = 0; i < MAX_TRACKED; i++) {
		if (t_psmv_create(&t->xfctx, &white, t->calib, &t->xtmv[i], &psmv_sinks[i]) != 0 || t->xtmv[i] == NULL) {
			U_LOG_IFL_E(log_level, "t_psmv_create failed for tracker %zu", i);
			oxrsys_wmr_psmv_tracking_destroy(&t);
			return NULL;
		}
		t->xtmv[i]->origin = &t->origin;
	}

	// HSV filter: sinks[0..2] are the colour buckets, sinks[3] the white one.
	struct t_hsv_filter_params params = T_HSV_DEFAULT_PARAMS();
	struct xrt_frame_sink *hsv_sinks[4] = {NULL, NULL, NULL, psmv_sinks[0]};
	struct xrt_frame_sink *sink = NULL;
	t_hsv_filter_create(&t->xfctx, &params, hsv_sinks, &sink);
	// The filter wants YUV; the headset delivers 8-bit grayscale.
	u_sink_create_to_yuv_or_yuyv(&t->xfctx, sink, &sink);
	// Decouple from the USB reader thread.
	u_sink_simple_queue_create(&t->xfctx, sink, &sink);
	// Two single-camera streams into one side-by-side stereo frame.
	struct xrt_frame_sink *left = NULL;
	struct xrt_frame_sink *right = NULL;
	u_sink_combiner_create(&t->xfctx, sink, &left, &right);

	// Point the headset's camera source at us. The WMR source only forwards
	// frames to these sinks; its own IMU/SLAM plumbing is unaffected. The
	// driver already started the cameras, and starting them twice fails, so
	// stop first.
	struct xrt_slam_sinks sinks;
	memset(&sinks, 0, sizeof(sinks));
	sinks.cam_count = 2;
	sinks.cams[0] = left;
	sinks.cams[1] = right;
	xrt_fs_stream_stop(wh->tracking.source);
	if (!xrt_fs_slam_stream_start(wh->tracking.source, &sinks)) {
		U_LOG_IFL_E(log_level, "Could not attach the sphere tracker to the headset cameras.");
		oxrsys_wmr_psmv_tracking_destroy(&t);
		return NULL;
	}
	t->source = wh->tracking.source;

	U_LOG_IFL_I(log_level, "PS Move sphere tracking on the headset cameras (%dx%d x2, white sphere).",
	            wh->config.tcams[0]->roi.extent.w, wh->config.tcams[0]->roi.extent.h);
	*out_tracking = t;
	return &t->base;
}

void
oxrsys_wmr_psmv_tracking_destroy(struct oxrsys_wmr_psmv_tracking **tracking_ptr)
{
	struct oxrsys_wmr_psmv_tracking *t = *tracking_ptr;
	if (t == NULL) {
		return;
	}
	// The headset's camera source keeps raw pointers to our sinks; point it
	// at nothing before they go away.
	if (t->source != NULL) {
		struct xrt_slam_sinks none;
		memset(&none, 0, sizeof(none));
		xrt_fs_stream_stop(t->source);
		xrt_fs_slam_stream_start(t->source, &none); // back to the driver's own (empty) sinks
		t->source = NULL;
	}
	// Tears down every sink and tracker created in the context.
	xrt_frame_context_destroy_nodes(&t->xfctx);
	if (t->calib != NULL) {
		t_stereo_camera_calibration_destroy(t->calib);
	}
	free(t);
	*tracking_ptr = NULL;
}

bool
oxrsys_wmr_psmv_tracking_available(void)
{
	return true;
}

#else /* !XRT_HAVE_OPENCV */

struct xrt_tracking_factory *
oxrsys_wmr_psmv_tracking_create(struct xrt_device *hmd,
                                enum u_logging_level log_level,
                                struct oxrsys_wmr_psmv_tracking **out_tracking)
{
	(void)hmd;
	U_LOG_IFL_I(log_level, "Sphere tracking not built (configure with -DOXRSYS_WMR_OPENCV=ON).");
	*out_tracking = NULL;
	return NULL;
}

void
oxrsys_wmr_psmv_tracking_destroy(struct oxrsys_wmr_psmv_tracking **tracking_ptr)
{
	*tracking_ptr = NULL;
}

bool
oxrsys_wmr_psmv_tracking_available(void)
{
	return false;
}

#endif
