// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief No-op stand-in for Monado's EuRoC dataset recorder.
 *
 * The real recorder (auxiliary/tracking/t_euroc_recorder.cpp) writes camera
 * frames to disk through OpenCV, which this build does not carry. The WMR
 * source pushes every frame and IMU sample into the recorder's sinks
 * unconditionally, so the sinks must exist; they just discard the data.
 */

#include "tracking/t_euroc_recorder.h"

#include "util/u_misc.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

struct euroc_recorder_stub
{
	struct xrt_slam_sinks sinks;
	struct xrt_frame_sink cam_sinks[XRT_TRACKING_MAX_CAMS];
	struct xrt_imu_sink imu_sinks[XRT_TRACKING_MAX_IMUS];
	struct xrt_pose_sink gt_sink;
	struct xrt_hand_masks_sink hand_masks_sink;
	struct xrt_frame_node node;
};

static void
stub_push_frame(struct xrt_frame_sink *sink, struct xrt_frame *frame)
{
	(void)sink;
	(void)frame;
}

static void
stub_push_imu(struct xrt_imu_sink *sink, struct xrt_imu_sample *sample)
{
	(void)sink;
	(void)sample;
}

static void
stub_push_pose(struct xrt_pose_sink *sink, struct xrt_pose_sample *sample)
{
	(void)sink;
	(void)sample;
}

static void
stub_push_hand_masks(struct xrt_hand_masks_sink *sink, struct xrt_hand_masks_sample *sample)
{
	(void)sink;
	(void)sample;
}

static void
stub_break_apart(struct xrt_frame_node *node)
{
	(void)node;
}

static void
stub_destroy(struct xrt_frame_node *node)
{
	struct euroc_recorder_stub *er = (struct euroc_recorder_stub *)((char *)node - offsetof(struct euroc_recorder_stub, node));
	free(er);
}

struct xrt_slam_sinks *
euroc_recorder_create(
    struct xrt_frame_context *xfctx, const char *record_path, int cam_count, int imu_count, bool record_from_start)
{
	(void)record_path;
	(void)record_from_start;

	struct euroc_recorder_stub *er = U_TYPED_CALLOC(struct euroc_recorder_stub);

	if (cam_count > XRT_TRACKING_MAX_CAMS) {
		cam_count = XRT_TRACKING_MAX_CAMS;
	}
	if (imu_count > XRT_TRACKING_MAX_IMUS) {
		imu_count = XRT_TRACKING_MAX_IMUS;
	}

	er->sinks.cam_count = cam_count;
	er->sinks.imu_count = imu_count;
	for (int i = 0; i < XRT_TRACKING_MAX_CAMS; i++) {
		er->cam_sinks[i].push_frame = stub_push_frame;
		er->sinks.cams[i] = &er->cam_sinks[i];
	}
	for (int i = 0; i < XRT_TRACKING_MAX_IMUS; i++) {
		er->imu_sinks[i].push_imu = stub_push_imu;
		er->sinks.imus[i] = &er->imu_sinks[i];
	}
	er->gt_sink.push_pose = stub_push_pose;
	er->sinks.gt = &er->gt_sink;
	er->hand_masks_sink.push_hand_masks = stub_push_hand_masks;
	er->sinks.hand_masks = &er->hand_masks_sink;

	er->node.break_apart = stub_break_apart;
	er->node.destroy = stub_destroy;
	if (xfctx != NULL) {
		xrt_frame_context_add(xfctx, &er->node);
	}

	return &er->sinks;
}

void
euroc_recorder_start(struct xrt_slam_sinks *er_sinks)
{
	(void)er_sinks;
}

void
euroc_recorder_stop(struct xrt_slam_sinks *er_sinks)
{
	(void)er_sinks;
}

void
euroc_recorder_add_ui(struct xrt_slam_sinks *er_sinks, void *root, const char *prefix)
{
	(void)er_sinks;
	(void)root;
	(void)prefix;
}
