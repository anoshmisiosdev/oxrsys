// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Bring-up tool: open a Windows Mixed Reality headset and print its
 * IMU-driven orientation, or open the Bluetooth motion controllers and print
 * their orientation and input state.
 *
 * Usage:
 *   oxrsys_wmr_probe [--list] [--controllers] [--psmove] [--seconds N] [--rate HZ] [--log-level LEVEL]
 *
 *   --list         Dump every HID device hidapi sees and exit. With --psmove,
 *                  only the PS Move entries, with bus and serial.
 *   --controllers  Skip the headset; open the WMR controllers paired over
 *                  Bluetooth and print their state.
 *   --psmove       Skip the headset; open every PlayStation Move controller
 *                  paired over Bluetooth and print its state.
 *   --seconds N    Stop after N seconds (default: run until Ctrl-C).
 *   --rate HZ      Print rate (default 10).
 *   --log-level    trace|debug|info|warn|error for the driver (default info).
 */

#include "psmv_macos.h"
#include "wmr_macos.h"

#include "os/os_time.h"
#include "util/u_logging.h"
#include "util/u_pretty_print.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include <ctype.h>
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
	fprintf(stderr,
	        "usage: %s [--list] [--controllers] [--psmove] [--seconds N] [--rate HZ] [--log-level LEVEL]\n",
	        argv0);
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

/*
 * "XRT_INPUT_WMR_TRIGGER_VALUE" / "XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE" ->
 * "trigger_value". The tail after the device prefix is what varies per input.
 */
static void
input_short_name(enum xrt_input_name name, char *out, size_t out_size)
{
	const char *full = u_str_xrt_input_name(name);
	const char *tail = strstr(full, "_CONTROLLER_");
	if (tail != NULL) {
		tail += strlen("_CONTROLLER_");
	} else if (strncmp(full, "XRT_INPUT_WMR_", strlen("XRT_INPUT_WMR_")) == 0) {
		tail = full + strlen("XRT_INPUT_WMR_");
	} else {
		tail = full;
	}
	size_t i = 0;
	for (; i + 1 < out_size && tail[i] != '\0'; i++) {
		out[i] = (char)tolower((unsigned char)tail[i]);
	}
	out[i] = '\0';
}

static void
print_input_value(const struct xrt_input *input)
{
	switch (XRT_GET_INPUT_TYPE(input->name)) {
	case XRT_INPUT_TYPE_BOOLEAN: printf("%s", input->value.boolean ? "1" : "0"); break;
	case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE:
	case XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE: printf("%.2f", input->value.vec1.x); break;
	case XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE:
		printf("(%+.2f,%+.2f)", input->value.vec2.x, input->value.vec2.y);
		break;
	default: printf("?"); break;
	}
}

/*
 * The first pose-typed input is the grip pose on every WMR controller; the
 * driver returns the same relation for grip and aim anyway.
 */
static enum xrt_input_name
controller_pose_input(const struct xrt_device *xdev)
{
	for (size_t i = 0; i < xdev->input_count; i++) {
		if (XRT_GET_INPUT_TYPE(xdev->inputs[i].name) == XRT_INPUT_TYPE_POSE) {
			return xdev->inputs[i].name;
		}
	}
	return XRT_INPUT_GENERIC_HEAD_POSE;
}

static void
print_controller_info(const char *tag, const struct xrt_device *xdev)
{
	if (xdev == NULL) {
		printf("%s: none\n", tag);
		return;
	}
	printf("%s: %s (serial %s, %s), orientation=%s position=%s\n", tag, xdev->str, xdev->serial,
	       u_str_xrt_device_name(xdev->name), xdev->supported.orientation_tracking ? "yes" : "no",
	       xdev->supported.position_tracking ? "yes" : "no");
	printf("  inputs:");
	for (size_t i = 0; i < xdev->input_count; i++) {
		char name[64];
		input_short_name(xdev->inputs[i].name, name, sizeof(name));
		printf(" %s", name);
	}
	printf("\n");
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
	if (headset->left != NULL || headset->right != NULL) {
		printf("Controllers: %s\n", headset->controllers_bluetooth ? "Bluetooth" : "headset radio");
	}
	print_controller_info("Left ctrl", headset->left);
	print_controller_info("Right ctrl", headset->right);

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

/*
 * Print one pose line: "<t> <tag> qx qy qz qw yaw pitch roll flags".
 * Returns false when the orientation was not valid.
 */
static bool
print_pose(const char *tag, struct xrt_device *xdev, enum xrt_input_name pose_name, double t_s, int64_t now_ns)
{
	struct xrt_space_relation relation;
	memset(&relation, 0, sizeof(relation));
	xrt_result_t xret = xrt_device_get_tracked_pose(xdev, pose_name, now_ns, &relation);
	if (xret != XRT_SUCCESS) {
		fprintf(stderr, "%s: get_tracked_pose failed: %d\n", tag, (int)xret);
		return false;
	}

	const bool orientation_valid = (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
	const bool orientation_tracked = (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;

	double yaw, pitch, roll;
	quat_to_yaw_pitch_roll(&relation.pose.orientation, &yaw, &pitch, &roll);
	printf("%10.3f %-4s %8.4f %8.4f %8.4f %8.4f  %7.1f %7.1f %7.1f  %s%s\n", t_s, tag,
	       relation.pose.orientation.x, relation.pose.orientation.y, relation.pose.orientation.z,
	       relation.pose.orientation.w, rad_to_deg(yaw), rad_to_deg(pitch), rad_to_deg(roll),
	       orientation_valid ? "valid" : "INVALID", orientation_tracked ? ",tracked" : "");
	return orientation_valid;
}

/*
 * Pose line plus a second line with every non-pose input, name=value.
 */
static bool
print_controller_state(const char *tag, struct xrt_device *xdev, double t_s, int64_t now_ns)
{
	xrt_result_t xret = xrt_device_update_inputs(xdev);
	if (xret != XRT_SUCCESS) {
		fprintf(stderr, "%s: update_inputs failed: %d\n", tag, (int)xret);
	}

	const bool valid = print_pose(tag, xdev, controller_pose_input(xdev), t_s, now_ns);

	printf("%15s", "");
	for (size_t i = 0; i < xdev->input_count; i++) {
		const struct xrt_input *input = &xdev->inputs[i];
		if (XRT_GET_INPUT_TYPE(input->name) == XRT_INPUT_TYPE_POSE) {
			continue;
		}
		char name[64];
		input_short_name(input->name, name, sizeof(name));
		printf(" %s=", name);
		print_input_value(input);
	}
	printf("\n");
	return valid;
}

/*
 * One device to poll in the loop: a head (pose only) or a controller (pose
 * and every input). NULL devices are skipped.
 */
struct probe_device
{
	const char *tag;
	struct xrt_device *xdev;
	bool is_head;
};

#define PROBE_MAX_DEVICES 16

/*
 * Poll at rate_hz until seconds elapse or a signal arrives.
 */
static void
run_loop(double seconds, double rate_hz, const struct probe_device *devices, size_t device_count)
{
	printf("\nPrinting at %.0f Hz. Ctrl-C to stop.\n", rate_hz);
	printf("%10s %-4s %8s %8s %8s %8s  %7s %7s %7s  %s\n", "t(s)", "dev", "qx", "qy", "qz", "qw", "yaw", "pitch",
	       "roll", "flags");

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	const int64_t start_ns = os_monotonic_get_ns();
	const int64_t period_ns = (int64_t)(1e9 / rate_hz);
	int64_t next_ns = start_ns;
	int printed = 0;
	int untracked = 0;

	while (!g_stop) {
		const int64_t now_ns = os_monotonic_get_ns();
		const double t_s = (double)(now_ns - start_ns) / 1e9;
		if (seconds >= 0.0 && t_s >= seconds) {
			break;
		}
		if (now_ns < next_ns) {
			const int64_t sleep_ns = next_ns - now_ns;
			struct timespec ts = {.tv_sec = sleep_ns / 1000000000LL, .tv_nsec = sleep_ns % 1000000000LL};
			nanosleep(&ts, NULL);
			continue;
		}
		next_ns += period_ns;

		for (size_t i = 0; i < device_count; i++) {
			const struct probe_device *dev = &devices[i];
			if (dev->xdev == NULL) {
				continue;
			}
			if (dev->is_head) {
				untracked += !print_pose(dev->tag, dev->xdev, XRT_INPUT_GENERIC_HEAD_POSE, t_s, now_ns);
			} else {
				untracked += !print_controller_state(dev->tag, dev->xdev, t_s, now_ns);
			}
		}
		fflush(stdout);
		printed++;
	}

	printf("\n%d samples printed, %d without a valid orientation.\n", printed, untracked);
}

int
main(int argc, char **argv)
{
	bool list_only = false;
	bool controllers_only = false;
	bool psmove_only = false;
	double seconds = -1.0;
	double rate_hz = 10.0;
	enum u_logging_level log_level = U_LOGGING_INFO;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--list") == 0) {
			list_only = true;
		} else if (strcmp(argv[i], "--controllers") == 0) {
			controllers_only = true;
		} else if (strcmp(argv[i], "--psmove") == 0) {
			psmove_only = true;
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
		if (psmove_only) {
			oxrsys_psmv_dump_hid_devices();
		} else {
			oxrsys_wmr_dump_hid_devices();
		}
		return 0;
	}

	// The drivers read their own log level from the environment.
	static const char *level_names[] = {"trace", "debug", "info", "warn", "error"};
	if (log_level <= U_LOGGING_ERROR) {
		setenv("WMR_LOG", level_names[log_level], 0);
		setenv("PSMV_LOG", level_names[log_level], 0);
	}

	if (psmove_only) {
		struct oxrsys_psmv_controller *controllers[PROBE_MAX_DEVICES];
		size_t count = 0;
		enum oxrsys_psmv_open_result result =
		    oxrsys_psmv_open_all(log_level, controllers, PROBE_MAX_DEVICES, &count);
		if (result != OXRSYS_PSMV_OPEN_OK) {
			fprintf(stderr, "No usable PS Move controller: %s.\n", oxrsys_psmv_open_result_str(result));
			if (result == OXRSYS_PSMV_OPEN_NONE_FOUND) {
				fprintf(stderr,
				        "Pair the controller in System Settings > Bluetooth (hold PS until the LED blinks; a "
				        "ZCM1 must first be paired over USB) and check it with --psmove --list.\n");
			}
			return 1;
		}

		struct probe_device devices[PROBE_MAX_DEVICES];
		char tags[PROBE_MAX_DEVICES][8];
		for (size_t i = 0; i < count; i++) {
			snprintf(tags[i], sizeof(tags[i]), "M%zu", i);
			printf("Controller %zu: %s (%04x:%04x, serial '%s')\n", i, oxrsys_psmv_model_str(controllers[i]->model),
			       controllers[i]->vid, controllers[i]->pid, controllers[i]->serial);
			print_controller_info(tags[i], controllers[i]->xdev);
			devices[i].tag = tags[i];
			devices[i].xdev = controllers[i]->xdev;
			devices[i].is_head = false;
		}
		run_loop(seconds, rate_hz, devices, count);

		for (size_t i = 0; i < count; i++) {
			oxrsys_psmv_close(&controllers[i]);
		}
		return 0;
	}

	if (controllers_only) {
		struct xrt_device *left = NULL;
		struct xrt_device *right = NULL;
		int count = oxrsys_wmr_open_bt_controllers(log_level, &left, &right);
		if (count == 0) {
			fprintf(stderr,
			        "No Windows Mixed Reality controllers connected over Bluetooth.\n"
			        "Pair each one in System Settings > Bluetooth: hold the pairing button inside the\n"
			        "battery compartment until the LEDs flash, then connect 'Motion controller - Left'\n"
			        "and 'Motion controller - Right'. Run with --list to see what hidapi reports.\n");
			return 1;
		}

		print_controller_info("Left ctrl", left);
		print_controller_info("Right ctrl", right);
		const struct probe_device devices[] = {
		    {"L", left, false},
		    {"R", right, false},
		};
		run_loop(seconds, rate_hz, devices, 2);

		if (left != NULL) {
			xrt_device_destroy(&left);
		}
		if (right != NULL) {
			xrt_device_destroy(&right);
		}
		return 0;
	}

	struct oxrsys_wmr_headset *headset = NULL;
	enum oxrsys_wmr_open_result result = oxrsys_wmr_headset_open(log_level, &headset);
	if (result != OXRSYS_WMR_OPEN_OK) {
		fprintf(stderr, "No usable headset: %s\n", oxrsys_wmr_open_result_str(result));
		return 1;
	}

	print_hmd_info(headset);
	const struct probe_device devices[] = {
	    {"head", headset->hmd, true},
	    {"L", headset->left, false},
	    {"R", headset->right, false},
	};
	run_loop(seconds, rate_hz, devices, 3);
	oxrsys_wmr_headset_close(&headset);
	return 0;
}
