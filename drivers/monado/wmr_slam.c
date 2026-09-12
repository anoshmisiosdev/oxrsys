// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  6DoF head tracking for a WMR headset through Monado's SLAM tracker.
 */

#include "wmr_slam.h"

#include "xrt/xrt_config_build.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "os/os_time.h"
#include "util/u_misc.h"
#include "wmr/wmr_hmd.h"
#include "xrt/xrt_frameserver.h"

#ifdef XRT_FEATURE_SLAM
#include "tracking/t_tracking.h"
#endif

/*
 *
 * Basalt configuration.
 *
 * Without a config file Basalt runs with its generic defaults and Monado
 * sends the calibration through the plugin interface. With one it also
 * reads the calibration from that file. The pipeline settings that Basalt
 * ships for Windows Mixed Reality headsets (its "msdmo" profile, tuned on
 * the Monado SLAM Datasets recorded with an Odyssey+) differ from the
 * defaults in ways that matter for these cameras, above all a safe radius
 * that keeps features out of the vignetted corners, so the helper generates
 * that config plus a calibration file converted from the driver's own
 * calibration, and points Monado at it.
 *
 */

#ifdef XRT_FEATURE_SLAM

/*
 * Basalt's data/msd/msdmo_config.json (BSD-3-Clause, Copyright 2019
 * Vladyslav Usenko and Nikolaus Demmel; Monado fork). Keys Basalt no longer
 * knows are ignored by its loader.
 */
static const char *const basalt_wmr_vio_config =
    "{\n"
    "    \"value0\": {\n"
    "        \"config.optical_flow_type\": \"frame_to_frame\",\n"
    "        \"config.optical_flow_detection_grid_size\": 50,\n"
    "        \"config.optical_flow_detection_num_points_cell\": 1,\n"
    "        \"config.optical_flow_detection_min_threshold\": 5,\n"
    "        \"config.optical_flow_detection_max_threshold\": 40,\n"
    "        \"config.optical_flow_detection_nonoverlap\": true,\n"
    "        \"config.optical_flow_max_recovered_dist2\": 0.04,\n"
    "        \"config.optical_flow_pattern\": 51,\n"
    "        \"config.optical_flow_max_iterations\": 5,\n"
    "        \"config.optical_flow_epipolar_error\": 0.005,\n"
    "        \"config.optical_flow_levels\": 3,\n"
    "        \"config.optical_flow_skip_frames\": 1,\n"
    "        \"config.optical_flow_matching_guess_type\": \"REPROJ_AVG_DEPTH\",\n"
    "        \"config.optical_flow_matching_default_depth\": 2.0,\n"
    "        \"config.optical_flow_image_safe_radius\": 388.0,\n"
    "        \"config.optical_flow_recall_enable\": false,\n"
    "        \"config.optical_flow_recall_all_cams\": false,\n"
    "        \"config.optical_flow_recall_num_points_cell\": true,\n"
    "        \"config.optical_flow_recall_over_tracking\": false,\n"
    "        \"config.optical_flow_recall_update_patch_viewpoint\": false,\n"
    "        \"config.optical_flow_recall_max_patch_dist\": 3,\n"
    "        \"config.optical_flow_recall_max_patch_norms\": [1.74, 0.96, 0.99, 0.44],\n"
    "        \"config.vio_linearization_type\": \"ABS_QR\",\n"
    "        \"config.vio_sqrt_marg\": true,\n"
    "        \"config.vio_max_states\": 3,\n"
    "        \"config.vio_max_kfs\": 7,\n"
    "        \"config.vio_min_frames_after_kf\": 5,\n"
    "        \"config.vio_new_kf_keypoints_thresh\": 0.7,\n"
    "        \"config.vio_debug\": false,\n"
    "        \"config.vio_extended_logging\": false,\n"
    "        \"config.vio_obs_std_dev\": 0.5,\n"
    "        \"config.vio_obs_huber_thresh\": 1.0,\n"
    "        \"config.vio_min_triangulation_dist\": 0.05,\n"
    "        \"config.vio_outlier_threshold\": 3.0,\n"
    "        \"config.vio_filter_iteration\": 4,\n"
    "        \"config.vio_max_iterations\": 7,\n"
    "        \"config.vio_enforce_realtime\": false,\n"
    "        \"config.vio_use_lm\": true,\n"
    "        \"config.vio_lm_lambda_initial\": 1e-4,\n"
    "        \"config.vio_lm_lambda_min\": 1e-6,\n"
    "        \"config.vio_lm_lambda_max\": 1e2,\n"
    "        \"config.vio_lm_landmark_damping_variant\": 1,\n"
    "        \"config.vio_lm_pose_damping_variant\": 1,\n"
    "        \"config.vio_scale_jacobian\": false,\n"
    "        \"config.vio_init_pose_weight\": 1e8,\n"
    "        \"config.vio_init_ba_weight\": 1e1,\n"
    "        \"config.vio_init_bg_weight\": 1e2,\n"
    "        \"config.vio_marg_lost_landmarks\": true,\n"
    "        \"config.vio_fix_long_term_keyframes\": false,\n"
    "        \"config.vio_kf_marg_feature_ratio\": 0.1,\n"
    "        \"config.vio_kf_marg_criteria\": \"KF_MARG_DEFAULT\",\n"
    "        \"config.mapper_obs_std_dev\": 0.25,\n"
    "        \"config.mapper_obs_huber_thresh\": 1.5,\n"
    "        \"config.mapper_detection_num_points\": 800,\n"
    "        \"config.mapper_num_frames_to_match\": 30,\n"
    "        \"config.mapper_frames_to_match_threshold\": 0.04,\n"
    "        \"config.mapper_min_matches\": 20,\n"
    "        \"config.mapper_ransac_threshold\": 5e-5,\n"
    "        \"config.mapper_min_track_length\": 5,\n"
    "        \"config.mapper_max_hamming_distance\": 70,\n"
    "        \"config.mapper_second_best_test_ratio\": 1.2,\n"
    "        \"config.mapper_bow_num_bits\": 16,\n"
    "        \"config.mapper_min_triangulation_dist\": 0.07,\n"
    "        \"config.mapper_no_factor_weights\": false,\n"
    "        \"config.mapper_use_factors\": true,\n"
    "        \"config.mapper_use_lm\": true,\n"
    "        \"config.mapper_lm_lambda_min\": 1e-32,\n"
    "        \"config.mapper_lm_lambda_max\": 1e3\n"
    "    }\n"
    "}\n";

//! Rotation matrix (row-major, r[row][col]) to quaternion, Shepperd's method.
static void
rotation_to_quat(const double r[3][3], double *qx, double *qy, double *qz, double *qw)
{
	const double trace = r[0][0] + r[1][1] + r[2][2];
	if (trace > 0.0) {
		const double s = sqrt(trace + 1.0) * 2.0;
		*qw = 0.25 * s;
		*qx = (r[2][1] - r[1][2]) / s;
		*qy = (r[0][2] - r[2][0]) / s;
		*qz = (r[1][0] - r[0][1]) / s;
	} else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
		const double s = sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
		*qw = (r[2][1] - r[1][2]) / s;
		*qx = 0.25 * s;
		*qy = (r[0][1] + r[1][0]) / s;
		*qz = (r[0][2] + r[2][0]) / s;
	} else if (r[1][1] > r[2][2]) {
		const double s = sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
		*qw = (r[0][2] - r[2][0]) / s;
		*qx = (r[0][1] + r[1][0]) / s;
		*qy = 0.25 * s;
		*qz = (r[1][2] + r[2][1]) / s;
	} else {
		const double s = sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
		*qw = (r[1][0] - r[0][1]) / s;
		*qx = (r[0][2] + r[2][0]) / s;
		*qy = (r[1][2] + r[2][1]) / s;
		*qz = 0.25 * s;
	}
}

static void
write_vec3(FILE *f, const char *name, const double v[3], const char *trailing)
{
	fprintf(f, "        \"%s\": [%.9g, %.9g, %.9g]%s\n", name, v[0], v[1], v[2], trailing);
}

/*
 * The calibration file, in the cereal JSON layout of basalt::Calibration
 * (see Basalt's data/msd/msdmo_calib.json), holding what Monado's
 * t_tracker_slam would otherwise send through vit_tracker_add_*_calibration.
 */
static bool
write_basalt_calibration(const struct t_slam_calibration *c, const char *path)
{
	FILE *f = fopen(path, "w");
	if (f == NULL) {
		return false;
	}
	fprintf(f, "{\n    \"value0\": {\n");
	fprintf(f, "        \"comment\": \"Generated by oxrsys-headset-helper from the headset's calibration\",\n");

	// T_imu_cam: the driver stores it column-major.
	fprintf(f, "        \"T_imu_cam\": [\n");
	for (int i = 0; i < c->cam_count; i++) {
		const float *m = c->cams[i].T_imu_cam.v;
		double r[3][3];
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				r[row][col] = m[col * 4 + row];
			}
		}
		double qx, qy, qz, qw;
		rotation_to_quat(r, &qx, &qy, &qz, &qw);
		fprintf(f,
		        "            {\"px\": %.12g, \"py\": %.12g, \"pz\": %.12g, "
		        "\"qx\": %.12g, \"qy\": %.12g, \"qz\": %.12g, \"qw\": %.12g}%s\n",
		        (double)m[12], (double)m[13], (double)m[14], qx, qy, qz, qw, i + 1 < c->cam_count ? "," : "");
	}
	fprintf(f, "        ],\n");

	// Intrinsics: the WMR model goes to Basalt as pinhole-radtan8, exactly as
	// Monado's add_camera_calibration() does (rpmax = the WMR metric radius).
	fprintf(f, "        \"intrinsics\": [\n");
	for (int i = 0; i < c->cam_count; i++) {
		const struct t_camera_calibration *b = &c->cams[i].base;
		if (b->distortion_model != T_DISTORTION_WMR) {
			fclose(f);
			return false;
		}
		const struct t_camera_calibration_wmr_params *w = &b->wmr;
		fprintf(f,
		        "            {\"camera_type\": \"pinhole-radtan8\", \"intrinsics\": {"
		        "\"fx\": %.12g, \"fy\": %.12g, \"cx\": %.12g, \"cy\": %.12g, "
		        "\"k1\": %.12g, \"k2\": %.12g, \"p1\": %.12g, \"p2\": %.12g, "
		        "\"k3\": %.12g, \"k4\": %.12g, \"k5\": %.12g, \"k6\": %.12g, \"rpmax\": %.12g}}%s\n",
		        b->intrinsics[0][0], b->intrinsics[1][1], b->intrinsics[0][2], b->intrinsics[1][2], w->k1, w->k2,
		        w->p1, w->p2, w->k3, w->k4, w->k5, w->k6, w->rpmax, i + 1 < c->cam_count ? "," : "");
	}
	fprintf(f, "        ],\n");

	fprintf(f, "        \"resolution\": [");
	for (int i = 0; i < c->cam_count; i++) {
		fprintf(f, "[%d, %d]%s", c->cams[i].base.image_size_pixels.w, c->cams[i].base.image_size_pixels.h,
		        i + 1 < c->cam_count ? ", " : "");
	}
	fprintf(f, "],\n");

	// IMU: the same packing Basalt's apply_imu_calibration() performs.
	const struct t_inertial_calibration *a = &c->imu.base.accel;
	const struct t_inertial_calibration *g = &c->imu.base.gyro;
	fprintf(f, "        \"calib_accel_bias\": [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g],\n", a->offset[0],
	        a->offset[1], a->offset[2], a->transform[0][0] - 1.0, a->transform[1][0], a->transform[2][0],
	        a->transform[1][1] - 1.0, a->transform[2][1], a->transform[2][2] - 1.0);
	fprintf(f,
	        "        \"calib_gyro_bias\": [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g],\n",
	        g->offset[0], g->offset[1], g->offset[2], g->transform[0][0] - 1.0, g->transform[1][0],
	        g->transform[2][0], g->transform[0][1], g->transform[1][1] - 1.0, g->transform[2][1],
	        g->transform[0][2], g->transform[1][2], g->transform[2][2] - 1.0);
	fprintf(f, "        \"imu_update_rate\": %.9g,\n", c->imu.frequency);
	write_vec3(f, "accel_noise_std", a->noise_std, ",");
	write_vec3(f, "gyro_noise_std", g->noise_std, ",");
	write_vec3(f, "accel_bias_std", a->bias_std, ",");
	write_vec3(f, "gyro_bias_std", g->bias_std, ",");
	fprintf(f, "        \"cam_time_offset_ns\": 0,\n");
	fprintf(f, "        \"vignette\": []\n");
	fprintf(f, "    }\n}\n");
	const bool ok = ferror(f) == 0;
	fclose(f);
	return ok;
}

/*
 * Write the three files and return the unified config's path (static
 * storage), or NULL. Everything lands in ~/Library/Caches/OXRSys/basalt/
 * (regenerated at every start). Basalt hands the path to CLI11 as a
 * whitespace-split command line, so it must not contain spaces; a home
 * directory with spaces falls back to /tmp.
 */
static const char *
write_basalt_config(const struct t_slam_calibration *calib, enum u_logging_level log_level)
{
	static char toml_path[1024];
	const char *home = getenv("HOME");
	char dir[1024];
	char parent[1024];
	if (home != NULL && strchr(home, ' ') == NULL) {
		snprintf(parent, sizeof(parent), "%s/Library/Caches/OXRSys", home);
		snprintf(dir, sizeof(dir), "%s/basalt", parent);
	} else {
		snprintf(parent, sizeof(parent), "/tmp/oxrsys-%u", (unsigned)getuid());
		snprintf(dir, sizeof(dir), "%s/basalt", parent);
	}
	(void)mkdir(parent, 0755);
	if (mkdir(dir, 0755) != 0 && errno != EEXIST) {
		U_LOG_IFL_W(log_level, "Cannot create %s: %s", dir, strerror(errno));
		return NULL;
	}

	char vio_path[1024];
	char calib_path[1024];
	snprintf(vio_path, sizeof(vio_path), "%s/wmr_vio_config.json", dir);
	snprintf(calib_path, sizeof(calib_path), "%s/wmr_calib.json", dir);
	snprintf(toml_path, sizeof(toml_path), "%s/wmr.toml", dir);

	FILE *f = fopen(vio_path, "w");
	if (f == NULL) {
		return NULL;
	}
	fputs(basalt_wmr_vio_config, f);
	fclose(f);

	if (!write_basalt_calibration(calib, calib_path)) {
		U_LOG_IFL_W(log_level, "Could not write the Basalt calibration file %s", calib_path);
		return NULL;
	}

	f = fopen(toml_path, "w");
	if (f == NULL) {
		return NULL;
	}
	fprintf(f,
	        "# Generated by oxrsys-headset-helper; edit wmr_vio_config.json to tune.\n"
	        "show-gui=0\n"
	        "cam-calib=\"%s\"\n"
	        "config-path=\"%s\"\n"
	        "marg-data=\"\"\n"
	        "print-queue=0\n"
	        "use-double=0\n"
	        "deterministic=0\n"
	        "num-threads=1\n",
	        calib_path, vio_path);
	fclose(f);
	return toml_path;
}

#endif /* XRT_FEATURE_SLAM */

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
	// Monado reads the calibration from the config file when one is given
	// (and does not send it through the plugin interface), so the generated
	// file carries the driver's calibration. SLAM_CONFIG in the environment
	// still wins, for experiments with hand-written configs.
	if (config.slam_config == NULL) {
		config.slam_config = write_basalt_config(&wh->tracking.slam_calib, log_level);
		if (config.slam_config != NULL) {
			U_LOG_IFL_I(log_level, "Basalt config for this headset: %s", config.slam_config);
		} else {
			U_LOG_IFL_W(log_level, "Could not write a Basalt config; using Basalt's generic defaults.");
		}
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
