// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Offline check of the WMR controller LED tracking setup.
 *
 * Without hardware: renders a 1st-gen motion controller's LED constellation
 * (a real controller's calibration, embedded below) from a known pose into
 * the two headset cameras as blob observations, feeds them to Monado's
 * constellation tracker configured the way wmr_controller_tracking.c
 * configures it, and checks the solved pose. Also checks the LED timesync
 * packet's bit layout. Exit status 0 on success.
 */

#include "wmr_controller_tracking.h"

#include "constellation/t_constellation_tracker.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "tracking/t_camera_models.h"
#include "tracking/t_constellation.h"
#include "wmr/wmr_config.h"
#include "xrt/xrt_frame.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//! LED positions and normals from a left 1st-gen controller's calibration (metres, calibration axes).
static const float kLeds[][2][3] = {
    {{0.014315, 0.053210, -0.001485}, {0.241857, 0.909375, -0.338440}},
    {{0.043364, 0.029342, -0.007175}, {0.741332, 0.507968, -0.438630}},
    {{-0.041821, 0.040456, 0.006464}, {-0.702104, 0.683232, -0.200611}},
    {{-0.008861, 0.056053, 0.002733}, {-0.151016, 0.952275, -0.265268}},
    {{-0.009625, -0.050945, -0.009263}, {-0.165812, -0.865137, -0.473333}},
    {{0.051508, -0.002764, -0.009129}, {0.880616, -0.041886, -0.471976}},
    {{0.026373, -0.049791, 0.000356}, {0.446635, -0.840359, -0.307107}},
    {{-0.057443, 0.011010, 0.006732}, {-0.962171, 0.189477, -0.195770}},
    {{0.041668, -0.030739, -0.009175}, {0.711430, -0.520378, -0.472307}},
    {{-0.042819, -0.036851, 0.000194}, {-0.724308, -0.616853, -0.308010}},
    {{0.055130, -0.019640, 0.006305}, {0.923808, -0.324251, -0.203570}},
    {{-0.052963, -0.014138, -0.003320}, {-0.899431, -0.234355, -0.368919}},
    {{0.056479, 0.013682, 0.006374}, {0.950296, 0.235873, -0.203228}},
    {{-0.045756, 0.023803, -0.008867}, {-0.782836, 0.411969, -0.466314}},
    {{0.035332, 0.046088, 0.006426}, {0.592476, 0.779893, -0.201837}},
    {{-0.026734, -0.052133, 0.006109}, {-0.449653, -0.868952, -0.206723}},
    {{-0.025070, 0.044922, -0.008870}, {-0.429744, 0.773047, -0.466603}},
    {{0.006084, -0.058287, 0.006157}, {0.100827, -0.973309, -0.206164}},
    {{-0.045326, -0.003391, -0.005141}, {0.885889, 0.059969, 0.460005}},
    {{-0.031309, 0.031597, -0.008174}, {0.602739, -0.612862, 0.510985}},
    {{0.002208, 0.046338, -0.001111}, {-0.042063, -0.921506, 0.386079}},
    {{0.041781, -0.016308, -0.006901}, {-0.812136, 0.311407, 0.493418}},
    {{0.012147, -0.047961, 0.003920}, {-0.235494, 0.929681, 0.283258}},
    {{0.049314, 0.003656, 0.004368}, {-0.958641, -0.077365, 0.273903}},
    {{0.024621, 0.038074, -0.004133}, {-0.482659, -0.755009, 0.443849}},
    {{-0.027036, -0.036092, -0.006711}, {0.526982, 0.695283, 0.488744}},
    {{-0.011839, -0.049032, 0.005426}, {0.230012, 0.940237, 0.251095}},
    {{0.033708, -0.036555, 0.004459}, {-0.654573, 0.705349, 0.272061}},
    {{-0.046129, 0.018432, 0.004847}, {0.894022, -0.362763, 0.262921}},
    {{0.043039, 0.026642, 0.006474}, {-0.824409, -0.517354, 0.229553}},
    {{-0.044141, -0.024824, 0.005897}, {0.848939, 0.470482, 0.240726}},
    {{-0.020203, 0.046147, 0.006213}, {0.388899, -0.890971, 0.234369}},
};
#define LED_COUNT (sizeof(kLeds) / sizeof(kLeds[0]))

//! Camera poses in the head frame as logged for a Dell Visor by the helper.
static const struct xrt_pose kHeadCam[2] = {
    {{0.1785f, -0.2119f, -0.0346f, -0.9602f}, {-0.0596f, 0.0010f, -0.0618f}},
    {{0.1788f, 0.2150f, 0.0398f, -0.9593f}, {0.0604f, 0.0008f, -0.0610f}},
};

static const struct xrt_pose kFlip = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

static int g_failures = 0;

#define CHECK(cond, ...)                                                                                               \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "FAIL: " __VA_ARGS__);                                                         \
			fprintf(stderr, "\n");                                                                         \
			g_failures++;                                                                                  \
		}                                                                                                      \
	} while (0)

/*
 * Timesync packet layout: after the two header bytes, one little-endian bit
 * stream of counter (2), intensity (9), timestamp (55), U2 (11), flags (2).
 */
static uint64_t
get_bits(const uint8_t *buf, unsigned first_bit, unsigned count)
{
	uint64_t v = 0;
	for (unsigned i = 0; i < count; i++) {
		unsigned bit = first_bit + i;
		v |= (uint64_t)((buf[bit / 8] >> (bit % 8)) & 1) << i;
	}
	return v;
}

static void
test_timesync_packet(void)
{
	uint8_t pkt[12];
	const uint64_t ts = 0x5A5A5A5A5A5A5AULL & ((1ULL << 55) - 1);
	oxrsys_wmr_ct_fill_timesync_packet(pkt, 0x42, 2, 333, ts, 777, 1);
	CHECK(pkt[0] == 0x03, "timesync report id %02x", pkt[0]);
	CHECK(pkt[1] == 0x42, "timesync command counter %02x", pkt[1]);
	CHECK(get_bits(pkt, 16, 2) == 2, "timesync counter");
	CHECK(get_bits(pkt, 18, 9) == 333, "timesync intensity %llu", (unsigned long long)get_bits(pkt, 18, 9));
	CHECK(get_bits(pkt, 27, 55) == ts, "timesync timestamp");
	CHECK(get_bits(pkt, 82, 11) == 777, "timesync U2 %llu", (unsigned long long)get_bits(pkt, 82, 11));
	CHECK(get_bits(pkt, 93, 2) == 1, "timesync flags");
	oxrsys_wmr_ct_fill_timesync_packet(pkt, 0, 1, 5000, 0, 5000, 1);
	CHECK(get_bits(pkt, 18, 9) == 399, "timesync intensity clamp");
	CHECK(get_bits(pkt, 82, 11) == 1023, "timesync U2 clamp");
}

/*
 * Constellation.
 */

struct test_device
{
	struct t_constellation_tracker_device base;
	int samples;
	struct xrt_pose last_pose;
	size_t last_camera;
	uint32_t last_matched;
};

static void
test_device_push(struct t_constellation_tracker_device *dev, struct t_constellation_tracker_sample *sample)
{
	struct test_device *d = (struct test_device *)dev;
	d->samples++;
	d->last_pose = sample->pose;
	d->last_camera = sample->camera_index;
	d->last_matched = sample->metrics.matched_blob_count;
}

struct test_head
{
	struct t_constellation_tracker_tracking_source base;
	struct xrt_pose pose;
};

static void
test_head_get(struct t_constellation_tracker_tracking_source *src, int64_t when_ns, struct xrt_space_relation *out)
{
	(void)when_ns;
	struct test_head *h = (struct test_head *)src;
	memset(out, 0, sizeof(*out));
	out->pose = h->pose;
	out->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT;
}

static void
fake_mark(struct t_blobwatch *tbw, const struct t_blob_observation *tbo, t_constellation_device_id_t id)
{
	(void)tbw;
	(void)tbo;
	(void)id;
}

static void
fake_destroy(struct t_blobwatch *tbw)
{
	(void)tbw;
}

static struct t_camera_calibration
test_calibration(void)
{
	struct t_camera_calibration c;
	memset(&c, 0, sizeof(c));
	c.image_size_pixels.w = 640;
	c.image_size_pixels.h = 480;
	c.intrinsics[0][0] = 260.0;
	c.intrinsics[1][1] = 260.0;
	c.intrinsics[0][2] = 320.0;
	c.intrinsics[1][2] = 240.0;
	c.intrinsics[2][2] = 1.0;
	c.distortion_model = T_DISTORTION_OPENCV_RADTAN_8;
	return c;
}

/*!
 * Render the controller at @p P_head_dev (OpenXR axes) into camera @p cam as
 * blobs, applying the same facing and ring-occlusion rules as the tracker.
 */
static uint32_t
render_blobs(const struct t_constellation_tracker_led_model *model,
             const struct t_camera_model_params *params,
             const struct xrt_pose *head,
             const struct xrt_pose *P_head_dev,
             int cam,
             struct t_blob *out,
             uint32_t max)
{
	struct xrt_pose P_world_cam, P_world_dev, P_cam_world, P_cam_dev, tmp, P_cam_dev_cv, P_dev_cam_cv;
	math_pose_transform(head, &kHeadCam[cam], &P_world_cam);
	math_pose_transform(head, P_head_dev, &P_world_dev);
	math_pose_invert(&P_world_cam, &P_cam_world);
	math_pose_transform(&P_cam_world, &P_world_dev, &P_cam_dev);
	math_pose_transform(&kFlip, &P_cam_dev, &tmp);
	math_pose_transform(&tmp, &kFlip, &P_cam_dev_cv);
	math_pose_invert(&P_cam_dev_cv, &P_dev_cam_cv);

	// The model's LEDs in calibration (= tracker OpenCV) axes, for the visibility callback.
	struct t_constellation_tracker_led cv_leds[LED_COUNT];
	struct t_constellation_tracker_led_model cv_model = *model;
	for (size_t i = 0; i < LED_COUNT; i++) {
		cv_leds[i] = model->leds[i];
		cv_leds[i].position = (struct xrt_vec3){kLeds[i][0][0], kLeds[i][0][1], kLeds[i][0][2]};
		cv_leds[i].normal = (struct xrt_vec3){kLeds[i][1][0], kLeds[i][1][1], kLeds[i][1][2]};
	}
	cv_model.leds = cv_leds;

	uint32_t n = 0;
	for (size_t i = 0; i < LED_COUNT && n < max; i++) {
		struct xrt_vec3 p, nrm;
		math_pose_transform_point(&P_cam_dev_cv, &cv_leds[i].position, &p);
		math_quat_rotate_vec3(&P_cam_dev_cv.orientation, &cv_leds[i].normal, &nrm);
		if (p.z <= 0.05f) {
			continue;
		}
		struct xrt_vec3 view = m_vec3_normalize(p);
		if (m_vec3_dot(view, nrm) > cos(M_PI - cv_leds[i].visibility_angle)) {
			continue;
		}
		if (!cv_model.compute_led_visibility(&cv_model, i, P_dev_cam_cv.position)) {
			continue;
		}
		float u, v;
		if (!t_camera_models_project(params, p.x, p.y, p.z, &u, &v) || u < 2 || v < 2 || u > 637 || v > 477) {
			continue;
		}
		const float size = fmaxf(2.0f, 2.0f * params->fx * cv_leds[i].radius_m / p.z);
		out[n] = (struct t_blob){
		    .blob_id = (uint32_t)(i + 1),
		    .matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID,
		    .matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID,
		    .center = {u, v},
		    .bounding_box = {.offset = {(int)(u - size / 2), (int)(v - size / 2)}, .extent = {(int)size, (int)size}},
		    .size = {size, size},
		    .brightness = 0.5f,
		};
		n++;
	}
	return n;
}

static int
visible_best(const struct t_constellation_tracker_led_model *model,
             const struct t_camera_model_params *params,
             const struct xrt_pose *head,
             const struct xrt_pose *P_head_dev)
{
	struct t_blob blobs[LED_COUNT];
	uint32_t a = render_blobs(model, params, head, P_head_dev, 0, blobs, LED_COUNT);
	uint32_t b = render_blobs(model, params, head, P_head_dev, 1, blobs, LED_COUNT);
	return (int)(a > b ? a : b);
}

//! Pixel noise and spurious blobs for the noisy case.
static bool g_noisy = false;
static uint32_t g_rand = 12345;

static float
noise_px(void)
{
	g_rand = g_rand * 1103515245u + 12345u;
	return ((float)((g_rand >> 8) & 0xffff) / 65535.0f - 0.5f) * 1.2f; // +-0.6 px
}

static void
test_constellation(const struct xrt_pose *head, struct xrt_vec3 position)
{
	struct wmr_led_config leds[LED_COUNT];
	for (size_t i = 0; i < LED_COUNT; i++) {
		leds[i].pos = (struct xrt_vec3){kLeds[i][0][0], kLeds[i][0][1], kLeds[i][0][2]};
		leds[i].norm = (struct xrt_vec3){kLeds[i][1][0], kLeds[i][1][1], kLeds[i][1][2]};
	}
	struct t_constellation_tracker_led model_leds[LED_COUNT];
	struct t_constellation_tracker_device_params dev_params;
	memset(&dev_params, 0, sizeof(dev_params));
	oxrsys_wmr_ct_build_led_model(leds, LED_COUNT, model_leds, &dev_params.led_model);

	struct t_camera_calibration calib = test_calibration();
	struct t_camera_model_params cam_params;
	t_camera_model_params_from_t_camera_calibration(&calib, &cam_params);

	// The orientation that shows the most LEDs to both cameras.
	struct xrt_pose truth = {{0, 0, 0, 1}, position};
	int best = -1;
	for (int yaw = -180; yaw < 180; yaw += 15) {
		for (int pitch = -90; pitch <= 90; pitch += 15) {
			struct xrt_quat qy, qp, q;
			math_quat_from_angle_vector((float)(yaw * M_PI / 180), &(struct xrt_vec3){0, 1, 0}, &qy);
			math_quat_from_angle_vector((float)(pitch * M_PI / 180), &(struct xrt_vec3){1, 0, 0}, &qp);
			math_quat_rotate(&qy, &qp, &q);
			struct xrt_pose cand = {q, position};
			int vis = visible_best(&dev_params.led_model, &cam_params, head, &cand);
			if (vis > best) {
				best = vis;
				truth = cand;
			}
		}
	}
	CHECK(best >= 6, "no orientation shows 6 LEDs to a camera (best %d)", best);

	struct test_head head_src = {.base = {.get_tracked_pose = test_head_get}, .pose = *head};
	struct t_constellation_tracker_params params;
	memset(&params, 0, sizeof(params));
	params.flags = T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC;
	params.num_mosaics = 1;
	params.mosaics[0].tracking_origin = &head_src.base;
	params.mosaics[0].num_cameras = 2;
	for (int i = 0; i < 2; i++) {
		params.mosaics[0].cameras[i] = (struct t_constellation_tracker_camera){
		    .calibration = calib,
		    .pose_in_origin = kHeadCam[i],
		    .has_concrete_pose = true,
		};
	}
	struct xrt_frame_context xfctx = {0};
	struct t_constellation_tracker *tracker = NULL;
	if (t_constellation_tracker_create(&xfctx, &params, &tracker) != 0) {
		CHECK(false, "tracker create");
		return;
	}
	struct test_device dev = {.base = {.push_constellation_tracker_sample = test_device_push}};
	t_constellation_device_id_t id;
	CHECK(t_constellation_tracker_add_device(tracker, &dev_params, &dev.base, &id) == 0, "add device");

	struct t_blobwatch fake_bw = {.mark_blob_device = fake_mark, .destroy = fake_destroy};
	for (int frame = 0; frame < 4; frame++) {
		for (int cam = 0; cam < 2; cam++) {
			struct t_blob blobs[LED_COUNT];
			struct t_blob blobs_noisy[LED_COUNT + 2];
			uint32_t n = render_blobs(&dev_params.led_model, &cam_params, head, &truth, cam, blobs, LED_COUNT);
			if (g_noisy) {
				for (uint32_t i = 0; i < n; i++) {
					blobs_noisy[i] = blobs[i];
					blobs_noisy[i].center.x += noise_px();
					blobs_noisy[i].center.y += noise_px();
				}
				// Two stray lights away from the controller.
				blobs_noisy[n] = blobs[0];
				blobs_noisy[n].blob_id = 100;
				blobs_noisy[n].center = (struct xrt_vec2){60.0f, 70.0f};
				blobs_noisy[n + 1] = blobs[0];
				blobs_noisy[n + 1].blob_id = 101;
				blobs_noisy[n + 1].center = (struct xrt_vec2){590.0f, 420.0f};
			}
			struct t_blob_observation obs = {
			    .source = &fake_bw,
			    .id = (uint64_t)(frame * 2 + cam),
			    .timestamp_ns = 1000000000LL + frame * 16666667LL,
			    .blobs = g_noisy ? blobs_noisy : blobs,
			    .num_blobs = g_noisy ? n + 2 : n,
			};
			t_blob_sink_push_blobs(params.mosaics[0].cameras[cam].blob_sink, &obs);
		}
	}

	CHECK(dev.samples > 0, "no pose for the controller at (%.2f, %.2f, %.2f)", position.x, position.y,
	      position.z);
	if (dev.samples > 0) {
		struct xrt_pose head_inv, rel;
		math_pose_invert(head, &head_inv);
		math_pose_transform(&head_inv, &dev.last_pose, &rel);
		const float pos_err = m_vec3_len(m_vec3_sub(rel.position, truth.position));
		struct xrt_quat d;
		math_quat_unrotate(&truth.orientation, &rel.orientation, &d);
		const float rot_err_deg = (float)(2.0 * acos(fmin(1.0, fabs(d.w))) * 180.0 / M_PI);
		printf("controller at (%.3f, %.3f, %.3f): %d samples, solved (%.3f, %.3f, %.3f) cam%zu %u blobs, "
		       "error %.1f mm %.2f deg (%d LEDs visible to the better camera)\n",
		       truth.position.x, truth.position.y, truth.position.z, dev.samples, rel.position.x, rel.position.y,
		       rel.position.z, dev.last_camera, dev.last_matched, pos_err * 1000.0f, rot_err_deg, best);
		CHECK(pos_err < (g_noisy ? 0.03f : 0.01f), "position error %.1f mm", pos_err * 1000.0f);
		CHECK(rot_err_deg < (g_noisy ? 6.0f : 3.0f), "rotation error %.2f deg", rot_err_deg);
	}

	xrt_frame_context_destroy_nodes(&xfctx);
}

int
main(void)
{
	test_timesync_packet();

	struct xrt_pose level = XRT_POSE_IDENTITY;
	test_constellation(&level, (struct xrt_vec3){0.05f, -0.20f, -0.40f});
	test_constellation(&level, (struct xrt_vec3){-0.15f, -0.35f, -0.30f});

	// A turned and tilted head: poses are still reported relative to it.
	struct xrt_pose turned = XRT_POSE_IDENTITY;
	struct xrt_quat qy, qp;
	math_quat_from_angle_vector(0.6f, &(struct xrt_vec3){0, 1, 0}, &qy);
	math_quat_from_angle_vector(-0.3f, &(struct xrt_vec3){1, 0, 0}, &qp);
	math_quat_rotate(&qy, &qp, &turned.orientation);
	test_constellation(&turned, (struct xrt_vec3){0.10f, -0.25f, -0.50f});

	// Sub-pixel noise on every blob and two stray lights.
	g_noisy = true;
	test_constellation(&level, (struct xrt_vec3){0.05f, -0.20f, -0.40f});

	if (g_failures != 0) {
		fprintf(stderr, "%d check(s) failed\n", g_failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
