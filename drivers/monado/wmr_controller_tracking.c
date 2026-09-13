// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Optical (LED constellation) tracking of 1st-gen WMR motion controllers
 *        with the headset's own tracking cameras.
 *
 * The LED sync protocol (timesync / keepalive packets and their timing) and the
 * ring occlusion model follow Jan Schmidt's and Beyley Cardellio's WMR
 * controller tracking work in Monado (the dev-constellation-controller-tracking
 * branch); the blob detector and constellation tracker are upstream Monado's.
 */

#include "wmr_controller_tracking.h"

#include "xrt/xrt_config_have.h"

#include "util/u_misc.h"

#include <stdlib.h>
#include <string.h>

#ifdef XRT_HAVE_OPENCV

#include "constellation/t_constellation_tracker.h"
#include "constellation/t_rift_blobwatch.h"
#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_mathinclude.h"
#include "math/m_vec3.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "tracking/t_constellation.h"
#include "tracking/t_tracking.h"
#include "util/u_debug.h"
#include "util/u_sink.h"
#include "util/u_time.h"
#include "wmr/wmr_config.h"
#include "wmr/wmr_controller_base.h"
#include "wmr/wmr_controller_protocol.h"
#include "wmr/wmr_hmd.h"
#include "wmr_slam.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"

#include <inttypes.h>

#define CT_D(t, ...) U_LOG_IFL_D((t)->log_level, __VA_ARGS__)
#define CT_I(t, ...) U_LOG_IFL_I((t)->log_level, __VA_ARGS__)
#define CT_W(t, ...) U_LOG_IFL_W((t)->log_level, __VA_ARGS__)
#define CT_E(t, ...) U_LOG_IFL_E((t)->log_level, __VA_ARGS__)

//! Controller frames are camera sinks tcam_count..2*tcam_count-1 of the source.
#define CONTROLLER_SINK_OFFSET 2

//! 1st-gen (HoloLens-era) motion controller status report: id byte + 44 bytes.
#define OG_STATUS_REPORT_SIZE 45
//! Offset of the little-endian 32-bit IMU tick counter in that report.
#define OG_STATUS_TICKS_OFFSET 29

//! Report ids upstream's wmr_controller_protocol.h does not name yet.
#define CT_CONTROLLER_LED_CONTROL 0x03
#define CT_CONTROLLER_KEEPALIVE 0x05

#define KEEPALIVE_INTERVAL_NS (125 * U_TIME_1MS_IN_NS)
#define LED_INTENSITY_DEFAULT 200
#define LED_INTENSITY_MAX 399

//! Headset frames are ~90 Hz; the frame timestamp is mid-way through a frame period.
#define HALF_FRAME_NS (U_TIME_1S_IN_NS / 180)

#define HEAD_POSE_CACHE 128

DEBUG_GET_ONCE_NUM_OPTION(ct_pixel_threshold, "OXRSYS_WMR_CT_PIXEL_THRESHOLD", 0x04)
DEBUG_GET_ONCE_NUM_OPTION(ct_blob_threshold, "OXRSYS_WMR_CT_BLOB_THRESHOLD", 0x10)
//! Extra delay added to the LED timesync, in 0.5 ms steps (0..44), for tuning.
DEBUG_GET_ONCE_NUM_OPTION(ct_time_offset, "OXRSYS_WMR_CT_TIME_OFFSET", 0)
DEBUG_GET_ONCE_BOOL_OPTION(ct_led_sync, "OXRSYS_WMR_CT_LED_SYNC", true)
//! Run the blob detector even with no controller to track (camera diagnostics).
DEBUG_GET_ONCE_BOOL_OPTION(ct_without_controllers, "OXRSYS_WMR_CT_WITHOUT_CONTROLLERS", false)

struct oxrsys_wmr_controller_tracking;

struct ct_hand
{
	struct oxrsys_wmr_controller_tracking *t;
	int index; //!< 0 left, 1 right
	struct wmr_controller_base *wcb;
	void (*original_receive)(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t size);

	struct t_constellation_tracker_device device;
	t_constellation_device_id_t id;
	struct t_constellation_tracker_led leds[WMR_MAX_LEDS];
	size_t led_count;

	//! Protects everything below.
	struct os_mutex lock;

	// LED sync.
	struct m_clock_windowed_skew_tracker *clock;
	uint64_t ticks;
	bool have_ticks;
	bool clock_locked;
	uint64_t last_frame_sequence;
	uint64_t timesync_device_time_us;
	bool timesync_updated;
	uint8_t cmd_counter;
	uint8_t timesync_counter;
	uint16_t led_intensity;
	int64_t next_keepalive_ns;
	uint64_t timesyncs_sent;
	uint64_t unexpected_reports;

	// Output.
	struct oxrsys_wmr_ct_hand state;
	uint64_t pose_count;
	uint64_t rate_pose_count;
};

struct ct_camera
{
	struct oxrsys_wmr_controller_tracking *t;
	uint32_t index;
	//! First sink in the chain: counts frames, drives LED sync (camera 0).
	struct xrt_frame_sink entry;
	//! Downstream of entry: queue -> blobwatch.
	struct xrt_frame_sink *queued;
	//! Between the blobwatch and the tracker, to record blobs for display.
	struct t_blob_sink blob_tap;
	struct t_blob_sink *tracker_sink;
	struct t_blobwatch *blobwatch;
};

struct head_pose_entry
{
	int64_t when_ns;
	struct xrt_pose pose;
};

struct oxrsys_wmr_controller_tracking
{
	enum u_logging_level log_level;
	struct xrt_frame_context xfctx;
	struct wmr_hmd *wh;
	struct xrt_fs *source;
	struct xrt_slam_sinks shared;
	struct xrt_slam_sinks current;

	struct t_constellation_tracker *tracker;
	struct t_constellation_tracker_tracking_source head_source;
	struct xrt_pose P_head_cam[OXRSYS_WMR_CT_CAMERAS];

	struct ct_camera cams[OXRSYS_WMR_CT_CAMERAS];
	uint32_t cam_count;
	struct ct_hand hands[2];

	//! Protects head_poses, views, frame counters.
	struct os_mutex lock;
	struct head_pose_entry head_poses[HEAD_POSE_CACHE];
	uint32_t head_pose_next;
	struct oxrsys_wmr_ct_camera_view views[OXRSYS_WMR_CT_CAMERAS];
	uint64_t controller_frames;
	uint64_t rate_frames;
	int64_t rate_start_ns;
	float controller_fps;
	int64_t last_log_ns;

	oxrsys_wmr_ct_frame_cb frame_cb;
	void *frame_cb_userdata;
};

/*
 *
 * Receive hook registry: Monado calls wcb->receive_bytes with the controller,
 * this finds our per-hand state for it.
 *
 */

static struct ct_hand *g_hooked[2];
static struct os_mutex g_hook_lock;
static bool g_hook_lock_init;

static struct ct_hand *
find_hooked(struct wmr_controller_base *wcb)
{
	for (int i = 0; i < 2; i++) {
		struct ct_hand *h = g_hooked[i];
		if (h != NULL && h->wcb == wcb) {
			return h;
		}
	}
	return NULL;
}

/*
 *
 * LED sync.
 *
 */

/*
 * Timesync packet (report 0x03), 12 bytes:
 *   [0] 0x03  [1] command counter (shared with keepalives)
 *   [2] bits 0-1: 1,2,3 counter; bits 2-7: LED intensity bits 0-5
 *   [3] bits 0-2: LED intensity bits 6-8; bits 3-7: timestamp bits 0-4
 *   [4..9] timestamp bits 5-52, [10] bits 0-1: timestamp bits 53-54, bits 2-7: U2 bits 0-5
 *   [11] bits 0-4: U2 bits 6-10, bits 5-6: flags (1 = pulse LEDs)
 * The timestamp is the controller-clock time (us) of the next controller exposure.
 */
static void
fill_timesync_packet(uint8_t buf[12], uint8_t cmd_ctr, uint8_t ts_ctr, int led_intensity, uint64_t ts, int u2, uint8_t flags)
{
	ts_ctr = ts_ctr & 0x3;
	led_intensity = led_intensity < 1 ? 1 : (led_intensity > LED_INTENSITY_MAX ? LED_INTENSITY_MAX : led_intensity);
	u2 = u2 < 0 ? 0 : (u2 > 1023 ? 1023 : u2);

	buf[0] = CT_CONTROLLER_LED_CONTROL;
	buf[1] = cmd_ctr;
	buf[2] = ts_ctr | ((led_intensity & 0x3f) << 2);
	buf[3] = ((led_intensity >> 6) & 0x7) | ((ts & 0x1f) << 3);
	buf[4] = (uint8_t)(ts >> 5);
	buf[5] = (uint8_t)(ts >> 13);
	buf[6] = (uint8_t)(ts >> 21);
	buf[7] = (uint8_t)(ts >> 29);
	buf[8] = (uint8_t)(ts >> 37);
	buf[9] = (uint8_t)(ts >> 45);
	buf[10] = (uint8_t)(((ts >> 53) & 0x3) | (u2 << 2));
	buf[11] = (uint8_t)(((u2 >> 6) & 0x1f) | ((flags & 0x3) << 5));
}

static bool
controller_send(struct wmr_controller_base *wcb, const uint8_t *buf, uint32_t size)
{
	bool ok = false;
	os_mutex_lock(&wcb->conn_lock);
	if (wcb->wcc != NULL && wcb->wcc->send_bytes != NULL) {
		ok = wmr_controller_connection_send_bytes(wcb->wcc, buf, size);
	}
	os_mutex_unlock(&wcb->conn_lock);
	return ok;
}

//! On the controller's Bluetooth read thread, for every report it delivers.
static void
hooked_receive(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t size)
{
	struct ct_hand *h = find_hooked(wcb);
	if (h == NULL) {
		// Being unhooked; nothing of ours to do.
		return;
	}

	if (size >= 1 && buffer[0] == WMR_MOTION_CONTROLLER_STATUS_MSG) {
		uint8_t pkt[12];
		uint32_t pkt_size = 0;
		uint8_t keepalive[2];
		bool send_keepalive = false;

		os_mutex_lock(&h->lock);
		if (size == OG_STATUS_REPORT_SIZE) {
			const uint8_t *p = buffer + OG_STATUS_TICKS_OFFSET;
			uint32_t ticks32 = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
			if (!h->have_ticks) {
				h->ticks = ticks32;
				h->have_ticks = true;
			} else {
				uint32_t delta = ticks32 - (uint32_t)h->ticks;
				if (delta > 0x80000000u) {
					// Went backwards: the controller restarted its clock.
					h->ticks = ticks32;
					m_clock_windowed_skew_tracker_reset(h->clock);
					h->clock_locked = false;
				} else {
					h->ticks += delta;
				}
			}
			const int64_t hw_ns = (int64_t)(h->ticks * WMR_MOTION_CONTROLLER_NS_PER_TICK);
			m_clock_windowed_skew_tracker_push(h->clock, (timepoint_ns)time_ns, hw_ns);
			timepoint_ns dummy;
			h->clock_locked = m_clock_windowed_skew_tracker_to_local(h->clock, hw_ns, &dummy);
		} else {
			h->unexpected_reports++;
		}

		if (h->timesync_updated && h->clock_locked) {
			// Counter runs 1, 2, 3, 1, ...; never 0.
			uint8_t ts_ctr = h->timesync_counter++;
			if (h->timesync_counter == 4) {
				h->timesync_counter = 1;
			}
			const uint64_t offset_us = (uint64_t)debug_get_num_option_ct_time_offset() * 500;
			fill_timesync_packet(pkt, h->cmd_counter++, ts_ctr, h->led_intensity,
			                     h->timesync_device_time_us + offset_us, 500, 1);
			pkt_size = sizeof(pkt);
			h->timesync_updated = false;
			h->timesyncs_sent++;
		}
		if (h->clock_locked && (int64_t)time_ns >= h->next_keepalive_ns) {
			keepalive[0] = CT_CONTROLLER_KEEPALIVE;
			keepalive[1] = h->cmd_counter++;
			send_keepalive = true;
			h->next_keepalive_ns = (h->next_keepalive_ns == 0 || (int64_t)time_ns - h->next_keepalive_ns > KEEPALIVE_INTERVAL_NS)
			                           ? (int64_t)time_ns + KEEPALIVE_INTERVAL_NS
			                           : h->next_keepalive_ns + KEEPALIVE_INTERVAL_NS;
		}
		os_mutex_unlock(&h->lock);

		if (pkt_size > 0) {
			controller_send(wcb, pkt, pkt_size);
		}
		if (send_keepalive) {
			controller_send(wcb, keepalive, sizeof(keepalive));
		}
	}

	h->original_receive(wcb, time_ns, buffer, size);
}

/*!
 * On the USB camera thread, for each controller frame (camera 0's half).
 * The cadence is SLAM, controller, controller. On the second controller frame,
 * tell the controllers when the next first controller frame starts, minus a
 * third of a frame, in their own clock.
 */
static void
notify_controller_frame(struct oxrsys_wmr_controller_tracking *t, const struct xrt_frame *xf)
{
	const int64_t frame_start_ns = xf->timestamp - HALF_FRAME_NS;
	for (int i = 0; i < 2; i++) {
		struct ct_hand *h = &t->hands[i];
		if (h->wcb == NULL) {
			continue;
		}
		os_mutex_lock(&h->lock);
		const bool second = h->last_frame_sequence + 1 == xf->source_sequence;
		if (second && h->clock_locked) {
			const timepoint_ns next_mono_ns = frame_start_ns + 18 * U_TIME_1MS_IN_NS;
			timepoint_ns next_device_ns;
			if (m_clock_windowed_skew_tracker_to_remote(h->clock, next_mono_ns, &next_device_ns)) {
				const uint64_t us = (uint64_t)next_device_ns / 1000;
				if (us != h->timesync_device_time_us) {
					h->timesync_device_time_us = us;
					h->timesync_updated = true;
				}
			}
		}
		h->last_frame_sequence = xf->source_sequence;
		os_mutex_unlock(&h->lock);
	}
}

/*
 *
 * Camera chain.
 *
 */

static void
log_summary_locked(struct oxrsys_wmr_controller_tracking *t, int64_t now)
{
	char line[512];
	int n = snprintf(line, sizeof(line), "controller tracking: %.1f controller fps, blobs cam0 %u cam1 %u",
	                 t->controller_fps, t->views[0].blob_count, t->cam_count > 1 ? t->views[1].blob_count : 0);
	for (int i = 0; i < 2 && n > 0 && (size_t)n < sizeof(line); i++) {
		struct ct_hand *h = &t->hands[i];
		if (h->wcb == NULL) {
			continue;
		}
		os_mutex_lock(&h->lock);
		const bool recent = h->state.pose_valid && now - h->state.pose_timestamp_ns < U_TIME_1S_IN_NS;
		n += snprintf(line + n, sizeof(line) - (size_t)n, "; %c: sync %s (%" PRIu64 " sent, LED %u), %.1f poses/s",
		              i == 0 ? 'L' : 'R', h->clock_locked ? "on" : "waiting", h->timesyncs_sent,
		              h->led_intensity, h->state.poses_per_second);
		if (recent && (size_t)n < sizeof(line)) {
			const struct xrt_vec3 p = h->state.head_relative.position;
			n += snprintf(line + n, sizeof(line) - (size_t)n, " at (%.2f, %.2f, %.2f) cam%u %u/%u LEDs err %.2fpx",
			              p.x, p.y, p.z, h->state.pose_camera, h->state.matched_blobs, h->state.visible_leds,
			              h->state.reprojection_error);
		}
		if (h->unexpected_reports > 0 && (size_t)n < sizeof(line)) {
			n += snprintf(line + n, sizeof(line) - (size_t)n, " (%" PRIu64 " non-1st-gen reports)",
			              h->unexpected_reports);
		}
		os_mutex_unlock(&h->lock);
	}
	CT_I(t, "%s", line);
}

static void
camera_entry_push_frame(struct xrt_frame_sink *xfs, struct xrt_frame *xf)
{
	struct ct_camera *c = container_of(xfs, struct ct_camera, entry);
	struct oxrsys_wmr_controller_tracking *t = c->t;

	if (c->index == 0) {
		if (debug_get_bool_option_ct_led_sync()) {
			notify_controller_frame(t, xf);
		}

		const int64_t now = (int64_t)os_monotonic_get_ns();
		os_mutex_lock(&t->lock);
		t->controller_frames++;
		if (t->rate_start_ns == 0) {
			t->rate_start_ns = now;
			t->rate_frames = t->controller_frames;
		} else if (now - t->rate_start_ns >= U_TIME_1S_IN_NS) {
			const float secs = (float)(now - t->rate_start_ns) / 1e9f;
			t->controller_fps = (float)(t->controller_frames - t->rate_frames) / secs;
			t->rate_frames = t->controller_frames;
			for (int i = 0; i < 2; i++) {
				struct ct_hand *h = &t->hands[i];
				if (h->wcb == NULL) {
					continue;
				}
				os_mutex_lock(&h->lock);
				h->state.poses_per_second = (float)(h->pose_count - h->rate_pose_count) / secs;
				h->rate_pose_count = h->pose_count;
				os_mutex_unlock(&h->lock);
			}
			t->rate_start_ns = now;
			if (now - t->last_log_ns >= 2 * U_TIME_1S_IN_NS) {
				t->last_log_ns = now;
				log_summary_locked(t, now);
			}
		}
		os_mutex_unlock(&t->lock);
	}

	oxrsys_wmr_ct_frame_cb cb = t->frame_cb;
	if (cb != NULL) {
		cb(t->frame_cb_userdata, c->index, xf);
	}

	xrt_sink_push_frame(c->queued, xf);
}

static void
blob_tap_push_blobs(struct t_blob_sink *tbs, struct t_blob_observation *obs)
{
	struct ct_camera *c = container_of(tbs, struct ct_camera, blob_tap);
	struct oxrsys_wmr_controller_tracking *t = c->t;

	os_mutex_lock(&t->lock);
	struct oxrsys_wmr_ct_camera_view *v = &t->views[c->index];
	v->timestamp_ns = obs->timestamp_ns;
	v->blob_count = obs->num_blobs;
	v->stored = 0;
	bool seen[2] = {false, false};
	for (uint32_t i = 0; i < obs->num_blobs && v->stored < OXRSYS_WMR_CT_MAX_BLOBS; i++) {
		const struct t_blob *b = &obs->blobs[i];
		struct oxrsys_wmr_ct_blob *out = &v->blobs[v->stored++];
		out->x = b->center.x;
		out->y = b->center.y;
		out->w = b->size.x;
		out->h = b->size.y;
		out->hand = -1;
		for (int hnd = 0; hnd < 2; hnd++) {
			if (t->hands[hnd].wcb != NULL && b->matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID &&
			    b->matched_device_id == t->hands[hnd].id) {
				out->hand = (int8_t)hnd;
				seen[hnd] = true;
			}
		}
	}
	os_mutex_unlock(&t->lock);

	for (int hnd = 0; hnd < 2; hnd++) {
		if (seen[hnd]) {
			struct ct_hand *h = &t->hands[hnd];
			os_mutex_lock(&h->lock);
			h->state.last_seen_ns[c->index] = obs->timestamp_ns;
			os_mutex_unlock(&h->lock);
		}
	}

	t_blob_sink_push_blobs(c->tracker_sink, obs);
}

static void
blob_tap_destroy(struct t_blob_sink *tbs)
{
	(void)tbs;
}

/*
 *
 * Tracker callbacks.
 *
 */

static void
head_source_get_tracked_pose(struct t_constellation_tracker_tracking_source *src,
                             int64_t when_ns,
                             struct xrt_space_relation *out)
{
	struct oxrsys_wmr_controller_tracking *t =
	    container_of(src, struct oxrsys_wmr_controller_tracking, head_source);

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret = xrt_device_get_tracked_pose(&t->wh->base, XRT_INPUT_GENERIC_HEAD_POSE, when_ns, &rel);
	if (xret != XRT_SUCCESS || (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		*out = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return;
	}
	// Only the orientation is used: poses are reported relative to the
	// headset, which just needs a gravity-aligned frame. Dropping the position
	// keeps a wandering or diverged SLAM estimate (float metres far from the
	// origin) from costing precision, and works the same with IMU-only heads.
	rel.pose.position = (struct xrt_vec3){0.0f, 0.0f, 0.0f};
	rel.relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	*out = rel;

	os_mutex_lock(&t->lock);
	struct head_pose_entry *e = &t->head_poses[t->head_pose_next];
	t->head_pose_next = (t->head_pose_next + 1) % HEAD_POSE_CACHE;
	e->when_ns = when_ns;
	e->pose = rel.pose;
	os_mutex_unlock(&t->lock);
}

static bool
lookup_head_pose(struct oxrsys_wmr_controller_tracking *t, int64_t when_ns, struct xrt_pose *out)
{
	bool found = false;
	os_mutex_lock(&t->lock);
	for (uint32_t i = 0; i < HEAD_POSE_CACHE; i++) {
		if (t->head_poses[i].when_ns == when_ns && when_ns != 0) {
			*out = t->head_poses[i].pose;
			found = true;
			break;
		}
	}
	os_mutex_unlock(&t->lock);
	return found;
}

static void
device_push_sample(struct t_constellation_tracker_device *dev, struct t_constellation_tracker_sample *sample)
{
	struct ct_hand *h = container_of(dev, struct ct_hand, device);
	struct oxrsys_wmr_controller_tracking *t = h->t;

	struct xrt_pose head;
	if (!lookup_head_pose(t, sample->timestamp_ns, &head)) {
		struct xrt_space_relation rel;
		head_source_get_tracked_pose(&t->head_source, sample->timestamp_ns, &rel);
		head = rel.pose;
	}
	struct xrt_pose head_inv;
	math_pose_invert(&head, &head_inv);
	struct xrt_pose rel_pose;
	math_pose_transform(&head_inv, &sample->pose, &rel_pose);

	os_mutex_lock(&h->lock);
	if (!h->state.pose_valid || sample->timestamp_ns >= h->state.pose_timestamp_ns) {
		h->state.pose_valid = true;
		h->state.pose_timestamp_ns = sample->timestamp_ns;
		h->state.head_relative = rel_pose;
		h->state.pose_camera = (uint32_t)sample->camera_index;
		h->state.matched_blobs = sample->metrics.matched_blob_count;
		h->state.visible_leds = sample->metrics.visible_led_count;
		h->state.reprojection_error = (float)sample->metrics.reprojection_error;
	}
	if (sample->camera_index < OXRSYS_WMR_CT_CAMERAS) {
		h->state.last_seen_ns[sample->camera_index] = sample->timestamp_ns;
	}
	h->pose_count++;

	// Brightness feedback: keep the matched LEDs visible but not saturated.
	const float brightness = sample->average_brightness * 255.0f;
	if (brightness > 70.0f) {
		h->led_intensity -= h->led_intensity > 3 ? 3 : h->led_intensity - 1;
	} else if (brightness < 30.0f) {
		h->led_intensity = h->led_intensity + 10 > LED_INTENSITY_MAX ? LED_INTENSITY_MAX : h->led_intensity + 10;
	}
	os_mutex_unlock(&h->lock);
}

#define WMR_RING_HEIGHT 0.02194146618190565f
#define WMR_RING_TOP_RADIUS (0.11277887330599087f / 2.0f)
#define WMR_RING_BOTTOM_RADIUS (0.09375531956362483f / 2.0f)

static bool
conical_frustum_ray_intersect(struct xrt_vec3 ray_origin,
                              struct xrt_vec3 ray_dir,
                              struct xrt_vec3 base_center,
                              struct xrt_vec3 axis,
                              float h,
                              float r1,
                              float r2)
{
	const float k = (r1 - r2) / h;
	const float k2 = k * k;
	const float cone_height = r1 / k;
	struct xrt_vec3 apex = m_vec3_sub(base_center, m_vec3_mul_scalar(axis, cone_height));
	struct xrt_vec3 delta_p = m_vec3_sub(ray_origin, apex);

	const float dv = m_vec3_dot(ray_dir, axis);
	const float pv = m_vec3_dot(delta_p, axis);
	const float a = m_vec3_dot(ray_dir, ray_dir) - (1 + k2) * dv * dv;
	const float b = 2 * (m_vec3_dot(ray_dir, delta_p) - (1 + k2) * dv * pv);
	const float c = m_vec3_dot(delta_p, delta_p) - (1 + k2) * pv * pv;

	float disc = b * b - 4 * a * c;
	if (disc < -1e-6f) {
		return false;
	}
	if (disc < 0.0f) {
		disc = 0.0f;
	}
	const float sqrt_d = sqrtf(disc);
	const float t0 = (-b - sqrt_d) / (2 * a);
	const float t1 = (-b + sqrt_d) / (2 * a);
	const float tt = t0 > 0 ? t0 : t1;
	if (tt < 0) {
		return false;
	}
	struct xrt_vec3 p = m_vec3_add(ray_origin, m_vec3_mul_scalar(ray_dir, tt));
	const float u = m_vec3_dot(m_vec3_sub(p, base_center), axis);
	return u >= 0 && u <= h;
}

/*!
 * The inward-facing LEDs on the ring are hidden by the ring itself from most
 * directions. Called with the LED model in the tracker's (OpenCV) axes, which
 * are the controller calibration's own axes.
 */
static bool
controller_led_visibility(struct t_constellation_tracker_led_model *led_model, size_t led_index, struct xrt_vec3 T_obj_cam)
{
	const struct t_constellation_tracker_led *led = &led_model->leds[led_index];

	struct xrt_vec3 led_dir = led->normal;
	led_dir.z = 0;
	math_vec3_normalize(&led_dir);

	struct xrt_vec3 to_axis = {-led->position.x, -led->position.y, 0};
	math_vec3_normalize(&to_axis);

	const float away = fabsf(acosf(m_vec3_dot(to_axis, led_dir)));
	const struct xrt_vec3 ring_base = {0, 0, WMR_RING_HEIGHT / 2.0f};
	const struct xrt_vec3 ring_axis = {0, 0, -1};

	if (away < DEG_TO_RAD(30.0) &&
	    conical_frustum_ray_intersect(led->position, m_vec3_normalize(m_vec3_sub(T_obj_cam, led->position)), ring_base,
	                                  ring_axis, WMR_RING_HEIGHT, WMR_RING_TOP_RADIUS, WMR_RING_BOTTOM_RADIUS)) {
		return false;
	}
	return true;
}

/*
 *
 * Setup.
 *
 */

//! Same numbers wmr_hmd.c derives (wmr_hmd_get_cam_calib is static there).
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

static void
log_pose(struct oxrsys_wmr_controller_tracking *t, const char *name, const struct xrt_pose *p)
{
	CT_I(t, "  %s: pos (%.4f, %.4f, %.4f) rot (%.4f, %.4f, %.4f, %.4f)", name, p->position.x, p->position.y,
	     p->position.z, p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w);
}

/*!
 * Pose of tracking camera @p i in the headset frame the driver reports poses
 * for (OpenXR axes), with the camera in OpenXR camera axes (+Y up, -Z forward).
 * Camera extrinsics as the SLAM calibration uses them (wmr_hmd.c).
 */
static struct xrt_pose
camera_pose_in_head(struct oxrsys_wmr_controller_tracking *t, int i)
{
	struct wmr_hmd *wh = t->wh;
	const struct xrt_pose flip = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

	// Raw accelerometer frame -> camera (OpenCV axes).
	struct xrt_pose P_acc_c0 = wh->config.sensors.accel.pose;
	struct xrt_pose P_acc_ci = P_acc_c0;
	if (i > 0) {
		struct xrt_pose P_c0_ci;
		math_pose_invert(&wh->config.tcams[i]->pose, &P_c0_ci);
		math_pose_transform(&P_acc_c0, &P_c0_ci, &P_acc_ci);
	}

	// Accelerometer in the head frame: P_oxr_acc is the pose of the raw
	// accelerometer frame in the "middle of the eyes" frame with OpenXR axes.
	// The driver rotates IMU samples with it (3DoF), and its SLAM path ends in
	// the same frame (IMU pose flipped to OpenXR axes, then P_imu_me).
	struct xrt_pose P_head_acc = wh->config.sensors.transforms.P_oxr_acc;

	struct xrt_pose P_head_ci_cv;
	math_pose_transform(&P_head_acc, &P_acc_ci, &P_head_ci_cv);
	struct xrt_pose P_head_ci;
	math_pose_transform(&P_head_ci_cv, &flip, &P_head_ci);
	return P_head_ci;
}

static bool
add_hand(struct oxrsys_wmr_controller_tracking *t, int index, struct xrt_device *xdev)
{
	struct ct_hand *h = &t->hands[index];
	h->t = t;
	h->index = index;
	h->id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	if (xdev == NULL) {
		return false;
	}
	if (xdev->name != XRT_DEVICE_WMR_CONTROLLER) {
		CT_W(t, "%s is not a WMR motion controller; not tracking it.", xdev->str);
		return false;
	}
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)xdev;
	if (wcb->config.led_count < 4 || wcb->config.led_count > XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE) {
		CT_W(t, "%s: calibration lists %d LEDs; not tracking it.", xdev->str, wcb->config.led_count);
		return false;
	}

	h->led_count = (size_t)wcb->config.led_count;
	for (size_t i = 0; i < h->led_count; i++) {
		const struct wmr_led_config *l = &wcb->config.leds[i];
		// Calibration axes are OpenCV-like (X right, Y down, Z forward); the
		// tracker takes OpenXR axes and flips them back.
		h->leds[i] = (struct t_constellation_tracker_led){
		    .position = {l->pos.x, -l->pos.y, -l->pos.z},
		    .normal = {l->norm.x, -l->norm.y, -l->norm.z},
		    .radius_m = 0.003f,
		    .visibility_angle = DEG_TO_RAD(82.0f),
		    .id = (t_constellation_led_id_it)i,
		};
	}

	struct t_constellation_tracker_device_params params = {
	    .led_model =
	        {
	            .leds = h->leds,
	            .led_count = h->led_count,
	            .match_parameters =
	                {
	                    .min_leds_for_correspondence_search_without_prior = 5,
	                    .min_leds_for_correspondence_search_with_prior = 4,
	                },
	            .compute_led_visibility = controller_led_visibility,
	        },
	    .tracking_source = NULL,
	    .imu_sink = NULL,
	};
	h->device.push_constellation_tracker_sample = device_push_sample;
	if (t_constellation_tracker_add_device(t->tracker, &params, &h->device, &h->id) != 0) {
		CT_E(t, "%s: the constellation tracker refused it.", xdev->str);
		return false;
	}

	os_mutex_init(&h->lock);
	h->clock = m_clock_windowed_skew_tracker_alloc(200);
	h->timesync_counter = 1;
	h->led_intensity = LED_INTENSITY_DEFAULT;
	h->state.present = true;
	h->wcb = wcb;

	if (debug_get_bool_option_ct_led_sync()) {
		os_mutex_lock(&g_hook_lock);
		g_hooked[index] = h;
		h->original_receive = wcb->receive_bytes;
		wcb->receive_bytes = hooked_receive;
		os_mutex_unlock(&g_hook_lock);
	}

	const struct wmr_led_config *l0 = &wcb->config.leds[0];
	CT_I(t, "%s: %zu LEDs (LED 0 at %.3f, %.3f, %.3f normal %.2f, %.2f, %.2f), device id %d", xdev->str,
	     h->led_count, l0->pos.x, l0->pos.y, l0->pos.z, l0->norm.x, l0->norm.y, l0->norm.z, h->id);
	return true;
}

struct oxrsys_wmr_controller_tracking *
oxrsys_wmr_controller_tracking_create(struct xrt_device *hmd,
                                      struct xrt_device *left,
                                      struct xrt_device *right,
                                      const struct xrt_slam_sinks *share_with,
                                      enum u_logging_level log_level)
{
	if (hmd == NULL || hmd->name != XRT_DEVICE_GENERIC_HMD) {
		return NULL;
	}
	struct wmr_hmd *wh = wmr_hmd(hmd);
	if (wh->tracking.source == NULL || wh->config.tcam_count < 1) {
		U_LOG_IFL_W(log_level, "Controller tracking: the headset has no tracking cameras.");
		return NULL;
	}
	const bool diagnostics = debug_get_bool_option_ct_without_controllers();
	if (left == NULL && right == NULL && !diagnostics) {
		return NULL;
	}
	if (!g_hook_lock_init) {
		os_mutex_init(&g_hook_lock);
		g_hook_lock_init = true;
	}

	struct oxrsys_wmr_controller_tracking *t = U_TYPED_CALLOC(struct oxrsys_wmr_controller_tracking);
	t->log_level = log_level;
	t->wh = wh;
	t->cam_count = (uint32_t)wh->config.tcam_count;
	if (t->cam_count > OXRSYS_WMR_CT_CAMERAS) {
		t->cam_count = OXRSYS_WMR_CT_CAMERAS;
	}
	os_mutex_init(&t->lock);
	t->head_source.get_tracked_pose = head_source_get_tracked_pose;

	CT_I(t, "Controller tracking calibration (imu2me %d):", wh->tracking.imu2me);
	log_pose(t, "accel.pose", &wh->config.sensors.accel.pose);
	log_pose(t, "P_oxr_acc", &wh->config.sensors.transforms.P_oxr_acc);
	log_pose(t, "P_imu_me", &wh->config.sensors.transforms.P_imu_me);

	struct t_constellation_tracker_params params;
	memset(&params, 0, sizeof(params));
	params.num_mosaics = 1;
	struct t_constellation_tracker_camera_mosaic *mosaic = &params.mosaics[0];
	mosaic->tracking_origin = &t->head_source;
	mosaic->num_cameras = t->cam_count;
	for (uint32_t i = 0; i < t->cam_count; i++) {
		t->P_head_cam[i] = camera_pose_in_head(t, (int)i);
		char name[32];
		snprintf(name, sizeof(name), "P_head_cam%u", i);
		log_pose(t, name, &t->P_head_cam[i]);
		mosaic->cameras[i] = (struct t_constellation_tracker_camera){
		    .calibration = camera_calibration(wh->config.tcams[i]),
		    .pose_in_origin = t->P_head_cam[i],
		    .has_concrete_pose = true,
		};
	}
	if (t_constellation_tracker_create(&t->xfctx, &params, &t->tracker) != 0 || t->tracker == NULL) {
		CT_E(t, "Controller tracking: could not create the constellation tracker.");
		xrt_frame_context_destroy_nodes(&t->xfctx);
		os_mutex_destroy(&t->lock);
		free(t);
		return NULL;
	}

	bool any = add_hand(t, 0, left);
	any = add_hand(t, 1, right) || any;
	if (!any && !diagnostics) {
		oxrsys_wmr_controller_tracking_destroy(&t);
		return NULL;
	}

	if (share_with != NULL) {
		t->shared = *share_with;
		t->current = *share_with;
	}
	const struct t_rift_blobwatch_params bw_params = {
	    .pixel_threshold = (uint8_t)debug_get_num_option_ct_pixel_threshold(),
	    .blob_required_threshold = (uint8_t)debug_get_num_option_ct_blob_threshold(),
	    .max_match_dist = RIFT_BLOBWATCH_DEFAULT_MAX_MATCH_DIST,
	    .max_blob_width = RIFT_BLOBWATCH_DEFAULT_MAX_BLOB_WIDTH,
	};
	for (uint32_t i = 0; i < t->cam_count; i++) {
		struct ct_camera *c = &t->cams[i];
		c->t = t;
		c->index = i;
		c->tracker_sink = mosaic->cameras[i].blob_sink;
		c->blob_tap.push_blobs = blob_tap_push_blobs;
		c->blob_tap.destroy = blob_tap_destroy;
		struct xrt_frame_sink *bw_sink = NULL;
		if (t_rift_blobwatch_create(&bw_params, &t->xfctx, &c->blob_tap, &bw_sink, &c->blobwatch) != 0 ||
		    bw_sink == NULL) {
			CT_E(t, "Controller tracking: could not create the blob detector for camera %u.", i);
			oxrsys_wmr_controller_tracking_destroy(&t);
			return NULL;
		}
		// Blob detection runs off the USB thread; late frames are dropped.
		u_sink_simple_queue_create(&t->xfctx, bw_sink, &c->queued);
		c->entry.push_frame = camera_entry_push_frame;

		const uint32_t slot = CONTROLLER_SINK_OFFSET + i;
		struct xrt_frame_sink *ours = &c->entry;
		if (t->current.cams[slot] != NULL) {
			u_sink_split_create(&t->xfctx, t->current.cams[slot], ours, &ours);
		}
		t->current.cams[slot] = ours;
	}
	if (t->current.cam_count < (int)(CONTROLLER_SINK_OFFSET + t->cam_count)) {
		t->current.cam_count = (int)(CONTROLLER_SINK_OFFSET + t->cam_count);
	}

	if (!oxrsys_wmr_camera_route(wh->tracking.source, &t->current)) {
		CT_E(t, "Controller tracking: could not attach to the headset cameras.");
		oxrsys_wmr_controller_tracking_destroy(&t);
		return NULL;
	}
	t->source = wh->tracking.source;

	CT_I(t, "Controller tracking on %u camera(s): left %s, right %s, LED sync %s, blob thresholds 0x%02x/0x%02x.",
	     t->cam_count, t->hands[0].wcb != NULL ? "yes" : "no", t->hands[1].wcb != NULL ? "yes" : "no",
	     debug_get_bool_option_ct_led_sync() ? "on" : "off", bw_params.pixel_threshold,
	     bw_params.blob_required_threshold);
	return t;
}

const struct xrt_slam_sinks *
oxrsys_wmr_controller_tracking_sinks(const struct oxrsys_wmr_controller_tracking *t)
{
	return t != NULL ? &t->current : NULL;
}

bool
oxrsys_wmr_controller_tracking_get_pose(struct oxrsys_wmr_controller_tracking *t,
                                        int hand,
                                        int64_t now_ns,
                                        int64_t max_age_ns,
                                        struct xrt_pose *out_head_relative,
                                        int64_t *out_timestamp_ns)
{
	if (t == NULL || hand < 0 || hand > 1 || t->hands[hand].wcb == NULL) {
		return false;
	}
	struct ct_hand *h = &t->hands[hand];
	bool ok = false;
	os_mutex_lock(&h->lock);
	if (h->state.pose_valid && now_ns - h->state.pose_timestamp_ns <= max_age_ns) {
		*out_head_relative = h->state.head_relative;
		if (out_timestamp_ns != NULL) {
			*out_timestamp_ns = h->state.pose_timestamp_ns;
		}
		ok = true;
	}
	os_mutex_unlock(&h->lock);
	return ok;
}

void
oxrsys_wmr_controller_tracking_set_frame_callback(struct oxrsys_wmr_controller_tracking *t,
                                                  oxrsys_wmr_ct_frame_cb callback,
                                                  void *userdata)
{
	if (t == NULL) {
		return;
	}
	t->frame_cb = NULL;
	t->frame_cb_userdata = userdata;
	t->frame_cb = callback;
}

void
oxrsys_wmr_controller_tracking_snapshot(struct oxrsys_wmr_controller_tracking *t, struct oxrsys_wmr_ct_snapshot *out)
{
	memset(out, 0, sizeof(*out));
	if (t == NULL) {
		return;
	}
	os_mutex_lock(&t->lock);
	out->camera_count = t->cam_count;
	for (uint32_t i = 0; i < t->cam_count; i++) {
		out->cams[i] = t->views[i];
	}
	out->controller_frames = t->controller_frames;
	out->controller_fps = t->controller_fps;
	os_mutex_unlock(&t->lock);
	for (int i = 0; i < 2; i++) {
		struct ct_hand *h = &t->hands[i];
		if (h->wcb == NULL) {
			continue;
		}
		os_mutex_lock(&h->lock);
		out->hands[i] = h->state;
		out->hands[i].led_sync = h->clock_locked && h->timesyncs_sent > 0;
		out->hands[i].led_intensity = h->led_intensity;
		out->hands[i].timesyncs_sent = h->timesyncs_sent;
		os_mutex_unlock(&h->lock);
	}
}

void
oxrsys_wmr_controller_tracking_destroy(struct oxrsys_wmr_controller_tracking **t_ptr)
{
	struct oxrsys_wmr_controller_tracking *t = *t_ptr;
	if (t == NULL) {
		return;
	}
	// Unhook the controllers first; a report being handled right now finishes
	// with the original handler.
	os_mutex_lock(&g_hook_lock);
	for (int i = 0; i < 2; i++) {
		struct ct_hand *h = &t->hands[i];
		if (h->wcb != NULL && h->original_receive != NULL) {
			h->wcb->receive_bytes = h->original_receive;
		}
		if (g_hooked[i] == h) {
			g_hooked[i] = NULL;
		}
	}
	os_mutex_unlock(&g_hook_lock);
	os_nanosleep(50 * U_TIME_1MS_IN_NS);

	if (t->source != NULL) {
		oxrsys_wmr_camera_route(t->source, &t->shared);
		t->source = NULL;
	}
	xrt_frame_context_destroy_nodes(&t->xfctx);

	for (int i = 0; i < 2; i++) {
		struct ct_hand *h = &t->hands[i];
		if (h->wcb != NULL) {
			m_clock_windowed_skew_tracker_destroy(h->clock);
			os_mutex_destroy(&h->lock);
		}
	}
	os_mutex_destroy(&t->lock);
	free(t);
	*t_ptr = NULL;
}

bool
oxrsys_wmr_controller_tracking_available(void)
{
	return true;
}

#else /* !XRT_HAVE_OPENCV */

struct oxrsys_wmr_controller_tracking *
oxrsys_wmr_controller_tracking_create(struct xrt_device *hmd,
                                      struct xrt_device *left,
                                      struct xrt_device *right,
                                      const struct xrt_slam_sinks *share_with,
                                      enum u_logging_level log_level)
{
	(void)hmd;
	(void)left;
	(void)right;
	(void)share_with;
	U_LOG_IFL_I(log_level, "Controller tracking not built (configure with -DOXRSYS_WMR_OPENCV=ON).");
	return NULL;
}

const struct xrt_slam_sinks *
oxrsys_wmr_controller_tracking_sinks(const struct oxrsys_wmr_controller_tracking *t)
{
	(void)t;
	return NULL;
}

bool
oxrsys_wmr_controller_tracking_get_pose(struct oxrsys_wmr_controller_tracking *t,
                                        int hand,
                                        int64_t now_ns,
                                        int64_t max_age_ns,
                                        struct xrt_pose *out_head_relative,
                                        int64_t *out_timestamp_ns)
{
	(void)t;
	(void)hand;
	(void)now_ns;
	(void)max_age_ns;
	(void)out_head_relative;
	(void)out_timestamp_ns;
	return false;
}

void
oxrsys_wmr_controller_tracking_set_frame_callback(struct oxrsys_wmr_controller_tracking *t,
                                                  oxrsys_wmr_ct_frame_cb callback,
                                                  void *userdata)
{
	(void)t;
	(void)callback;
	(void)userdata;
}

void
oxrsys_wmr_controller_tracking_snapshot(struct oxrsys_wmr_controller_tracking *t, struct oxrsys_wmr_ct_snapshot *out)
{
	(void)t;
	memset(out, 0, sizeof(*out));
}

void
oxrsys_wmr_controller_tracking_destroy(struct oxrsys_wmr_controller_tracking **t_ptr)
{
	*t_ptr = NULL;
}

bool
oxrsys_wmr_controller_tracking_available(void)
{
	return false;
}

#endif
