// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Bring-up tool: open a Windows Mixed Reality headset and print its
 * IMU-driven orientation.
 *
 * Usage:
 *   oxrsys_wmr_probe [--list] [--seconds N] [--rate HZ] [--log-level LEVEL]
 *
 *   --list       Dump every HID device hidapi sees and exit.
 *   --seconds N  Stop after N seconds (default: run until Ctrl-C).
 *   --rate HZ    Pose print rate (default 10).
 *   --log-level  trace|debug|info|warn|error for the driver (default info).
 */

#include "wmr_macos.h"

#include "os/os_time.h"
#include "util/u_logging.h"
#include "xrt/xrt_device.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop = 0;

static void
on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static void
usage(const char *argv0)
{
	fprintf(stderr, "usage: %s [--list] [--seconds N] [--rate HZ] [--log-level LEVEL]\n", argv0);
}

static bool
parse_log_level(const char *text, enum u_logging_level *out_level)
{
	if (strcmp(text, "trace") == 0) {
		*out_level = U_LOGGING_TRACE;
	} else if (strcmp(text, "debug") == 0) {
		*out_level = U_LOGGING_DEBUG;
	} else if (strcmp(text, "info") == 0) {
		*out_level = U_LOGGING_INFO;
	} else if (strcmp(text, "warn") == 0) {
		*out_level = U_LOGGING_WARN;
	} else if (strcmp(text, "error") == 0) {
		*out_level = U_LOGGING_ERROR;
	} else {
		return false;
	}
	return true;
}

static double
rad_to_deg(double radians)
{
	return radians * 180.0 / M_PI;
}

/*
 * Tait-Bryan angles, yaw about +Y, pitch about +X, roll about +Z, matching
 * the OpenXR/Monado right-handed, Y-up convention.
 */
static void
quat_to_yaw_pitch_roll(const struct xrt_quat *q, double *yaw, double *pitch, double *roll)
{
	const double x = q->x, y = q->y, z = q->z, w = q->w;

	const double sinp = 2.0 * (w * x - y * z);
	*pitch = fabs(sinp) >= 1.0 ? copysign(M_PI / 2.0, sinp) : asin(sinp);
	*yaw = atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (x * x + y * y));
	*roll = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (x * x + z * z));
}

static void
print_hmd_info(const struct oxrsys_wmr_headset *headset)
{
	const struct xrt_device *hmd = headset->hmd;
	const struct xrt_hmd_parts *parts = hmd->hmd;

	printf("Headset:   %s\n", oxrsys_wmr_headset_type_str(headset->type));
	printf("Device:    %s (serial %s)\n", hmd->str, hmd->serial);
	printf("Companion: %s (%04x:%04x)\n", headset->companion_product, headset->companion_vid,
	       headset->companion_pid);
	printf("Tracking:  orientation=%s position=%s\n", hmd->supported.orientation_tracking ? "yes" : "no",
	       hmd->supported.position_tracking ? "yes" : "no");
	printf("Left ctrl: %s\n", headset->left != NULL ? headset->left->str : "none");
	printf("Right ctrl:%s\n", headset->right != NULL ? headset->right->str : "none");

	if (parts == NULL) {
		printf("Display:   (no HMD parts reported)\n");
		return;
	}

	const double refresh_hz =
	    parts->screens[0].nominal_frame_interval_ns > 0 ? 1e9 / (double)parts->screens[0].nominal_frame_interval_ns : 0.0;
	printf("Display:   %dx%d @ %.1f Hz, %zu views\n", parts->screens[0].w_pixels, parts->screens[0].h_pixels,
	       refresh_hz, parts->view_count);

	for (size_t i = 0; i < parts->view_count && i < XRT_MAX_VIEWS; i++) {
		const struct xrt_view *view = &parts->views[i];
		const struct xrt_fov *fov = &parts->distortion.fov[i];
		printf("  view %zu: viewport %ux%u at (%u,%u); fov L %.1f R %.1f U %.1f D %.1f deg\n", i,
		       view->viewport.w_pixels, view->viewport.h_pixels, view->viewport.x_pixels, view->viewport.y_pixels,
		       rad_to_deg(fov->angle_left), rad_to_deg(fov->angle_right), rad_to_deg(fov->angle_up),
		       rad_to_deg(fov->angle_down));
	}
	printf("Distortion models: 0x%x (preferred 0x%x)\n", (unsigned)parts->distortion.models,
	       (unsigned)parts->distortion.preferred);
}

int
main(int argc, char **argv)
{
	bool list_only = false;
	double seconds = -1.0;
	double rate_hz = 10.0;
	enum u_logging_level log_level = U_LOGGING_INFO;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--list") == 0) {
			list_only = true;
		} else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
			seconds = atof(argv[++i]);
		} else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc) {
			rate_hz = atof(argv[++i]);
			if (rate_hz <= 0.0) {
				rate_hz = 10.0;
			}
		} else if (strcmp(argv[i], "--log-level") == 0 && i + 1 < argc) {
			if (!parse_log_level(argv[++i], &log_level)) {
				usage(argv[0]);
				return 2;
			}
		} else {
			usage(argv[0]);
			return 2;
		}
	}

	if (list_only) {
		oxrsys_wmr_dump_hid_devices();
		return 0;
	}

	// The driver reads its own log level from the environment.
	static const char *level_names[] = {"trace", "debug", "info", "warn", "error"};
	if (log_level <= U_LOGGING_ERROR) {
		setenv("WMR_LOG", level_names[log_level], 0);
	}

	struct oxrsys_wmr_headset *headset = NULL;
	enum oxrsys_wmr_open_result result = oxrsys_wmr_headset_open(log_level, &headset);
	if (result != OXRSYS_WMR_OPEN_OK) {
		fprintf(stderr, "No usable headset: %s\n", oxrsys_wmr_open_result_str(result));
		return 1;
	}

	print_hmd_info(headset);
	printf("\nPrinting head orientation at %.0f Hz. Ctrl-C to stop.\n", rate_hz);
	printf("%10s  %8s %8s %8s %8s  %7s %7s %7s  %s\n", "t(s)", "qx", "qy", "qz", "qw", "yaw", "pitch", "roll",
	       "flags");

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	const int64_t start_ns = os_monotonic_get_ns();
	const int64_t period_ns = (int64_t)(1e9 / rate_hz);
	int64_t next_ns = start_ns;
	int printed = 0;
	int untracked = 0;

	while (!g_stop) {
		const int64_t now_ns = os_monotonic_get_ns();
		if (seconds >= 0.0 && (double)(now_ns - start_ns) / 1e9 >= seconds) {
			break;
		}
		if (now_ns < next_ns) {
			const int64_t sleep_ns = next_ns - now_ns;
			struct timespec ts = {.tv_sec = sleep_ns / 1000000000LL, .tv_nsec = sleep_ns % 1000000000LL};
			nanosleep(&ts, NULL);
			continue;
		}
		next_ns += period_ns;

		struct xrt_space_relation relation;
		memset(&relation, 0, sizeof(relation));
		xrt_result_t xret = xrt_device_get_tracked_pose(headset->hmd, XRT_INPUT_GENERIC_HEAD_POSE, now_ns, &relation);
		if (xret != XRT_SUCCESS) {
			fprintf(stderr, "get_tracked_pose failed: %d\n", (int)xret);
			untracked++;
			continue;
		}

		const bool orientation_valid = (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
		const bool orientation_tracked =
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;
		if (!orientation_valid) {
			untracked++;
		}

		double yaw, pitch, roll;
		quat_to_yaw_pitch_roll(&relation.pose.orientation, &yaw, &pitch, &roll);
		printf("%10.3f  %8.4f %8.4f %8.4f %8.4f  %7.1f %7.1f %7.1f  %s%s\n", (double)(now_ns - start_ns) / 1e9,
		       relation.pose.orientation.x, relation.pose.orientation.y, relation.pose.orientation.z,
		       relation.pose.orientation.w, rad_to_deg(yaw), rad_to_deg(pitch), rad_to_deg(roll),
		       orientation_valid ? "valid" : "INVALID", orientation_tracked ? ",tracked" : "");
		fflush(stdout);
		printed++;
	}

	printf("\n%d samples printed, %d without a valid orientation.\n", printed, untracked);
	oxrsys_wmr_headset_close(&headset);
	return 0;
}
