// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Check an optical controller rotation against the controller's gyro.
 */

#include "wmr_ct_gyro_check.h"

#include <math.h>
#include <string.h>

static const struct oxrsys_wmr_ct_gyro_sample *
sample_at(const struct oxrsys_wmr_ct_gyro_history *h, uint32_t i)
{
	// i = 0 is the oldest valid sample.
	uint32_t first = (h->next + OXRSYS_WMR_CT_GYRO_SAMPLES - h->count) % OXRSYS_WMR_CT_GYRO_SAMPLES;
	return &h->samples[(first + i) % OXRSYS_WMR_CT_GYRO_SAMPLES];
}

static void
quat_mul(const struct xrt_quat *a, const struct xrt_quat *b, struct xrt_quat *out)
{
	struct xrt_quat r = {
	    .x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y,
	    .y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x,
	    .z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w,
	    .w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z,
	};
	*out = r;
}

static void
quat_normalize(struct xrt_quat *q)
{
	double n = sqrt((double)q->x * q->x + (double)q->y * q->y + (double)q->z * q->z + (double)q->w * q->w);
	if (n <= 0.0) {
		*q = (struct xrt_quat){0.0f, 0.0f, 0.0f, 1.0f};
		return;
	}
	q->x = (float)(q->x / n);
	q->y = (float)(q->y / n);
	q->z = (float)(q->z / n);
	q->w = (float)(q->w / n);
}

//! Rotation by angular velocity @p w over @p dt seconds.
static void
quat_from_rotation_vector(const struct xrt_vec3 *w, double dt, struct xrt_quat *out)
{
	const double wx = (double)w->x * dt, wy = (double)w->y * dt, wz = (double)w->z * dt;
	const double angle = sqrt(wx * wx + wy * wy + wz * wz);
	if (angle < 1e-12) {
		*out = (struct xrt_quat){(float)(wx * 0.5), (float)(wy * 0.5), (float)(wz * 0.5), 1.0f};
		quat_normalize(out);
		return;
	}
	const double s = sin(angle * 0.5) / angle;
	*out = (struct xrt_quat){(float)(wx * s), (float)(wy * s), (float)(wz * s), (float)cos(angle * 0.5)};
}

static float
quat_angle_deg(const struct xrt_quat *q)
{
	double w = fabs((double)q->w);
	const double v = sqrt((double)q->x * q->x + (double)q->y * q->y + (double)q->z * q->z);
	return (float)(2.0 * atan2(v, w) * 180.0 / M_PI);
}

void
oxrsys_wmr_ct_gyro_reset(struct oxrsys_wmr_ct_gyro_history *h)
{
	h->next = 0;
	h->count = 0;
}

void
oxrsys_wmr_ct_gyro_push(struct oxrsys_wmr_ct_gyro_history *h, int64_t timestamp_ns, const struct xrt_vec3 *gyro)
{
	if (h->count > 0) {
		const struct oxrsys_wmr_ct_gyro_sample *newest = sample_at(h, h->count - 1);
		if (timestamp_ns < newest->timestamp_ns) {
			oxrsys_wmr_ct_gyro_reset(h);
		} else if (timestamp_ns == newest->timestamp_ns) {
			return;
		}
	}
	h->samples[h->next] = (struct oxrsys_wmr_ct_gyro_sample){timestamp_ns, *gyro};
	h->next = (h->next + 1) % OXRSYS_WMR_CT_GYRO_SAMPLES;
	if (h->count < OXRSYS_WMR_CT_GYRO_SAMPLES) {
		h->count++;
	}
}

/*
 * Each sample's rate holds from half-way to the previous sample to half-way
 * to the next; before the first and after the last sample it is extended by
 * at most OXRSYS_WMR_CT_GYRO_END_SLACK_NS.
 */
bool
oxrsys_wmr_ct_gyro_integrate(const struct oxrsys_wmr_ct_gyro_history *h,
                             int64_t t0_ns,
                             int64_t t1_ns,
                             struct xrt_quat *out_delta)
{
	*out_delta = (struct xrt_quat){0.0f, 0.0f, 0.0f, 1.0f};
	if (h->count == 0 || t1_ns < t0_ns) {
		return false;
	}
	const int64_t oldest = sample_at(h, 0)->timestamp_ns;
	const int64_t newest = sample_at(h, h->count - 1)->timestamp_ns;
	if (oldest > t0_ns + OXRSYS_WMR_CT_GYRO_END_SLACK_NS || newest < t1_ns - OXRSYS_WMR_CT_GYRO_END_SLACK_NS) {
		return false;
	}

	struct xrt_quat q = {0.0f, 0.0f, 0.0f, 1.0f};
	for (uint32_t i = 0; i < h->count; i++) {
		const struct oxrsys_wmr_ct_gyro_sample *s = sample_at(h, i);
		int64_t lo, hi;
		if (i == 0) {
			lo = s->timestamp_ns - OXRSYS_WMR_CT_GYRO_END_SLACK_NS;
		} else {
			const int64_t prev = sample_at(h, i - 1)->timestamp_ns;
			if (s->timestamp_ns - prev > OXRSYS_WMR_CT_GYRO_MAX_GAP_NS && prev < t1_ns && s->timestamp_ns > t0_ns) {
				return false;
			}
			lo = prev + (s->timestamp_ns - prev) / 2;
		}
		if (i + 1 == h->count) {
			hi = s->timestamp_ns + OXRSYS_WMR_CT_GYRO_END_SLACK_NS;
		} else {
			const int64_t next = sample_at(h, i + 1)->timestamp_ns;
			hi = s->timestamp_ns + (next - s->timestamp_ns) / 2;
		}
		if (lo < t0_ns) {
			lo = t0_ns;
		}
		if (hi > t1_ns) {
			hi = t1_ns;
		}
		if (hi <= lo) {
			continue;
		}
		struct xrt_quat step;
		quat_from_rotation_vector(&s->gyro, (double)(hi - lo) / 1e9, &step);
		// Body-frame rates compose on the right.
		quat_mul(&q, &step, &q);
	}
	quat_normalize(&q);
	*out_delta = q;
	return true;
}

enum oxrsys_wmr_ct_gyro_result
oxrsys_wmr_ct_gyro_check(const struct oxrsys_wmr_ct_gyro_history *h,
                         int64_t t0_ns,
                         const struct xrt_quat *q0,
                         int64_t t1_ns,
                         const struct xrt_quat *q1,
                         float base_deg,
                         float *out_mismatch_deg,
                         float *out_gyro_deg)
{
	const int64_t dt = t1_ns - t0_ns;
	if (dt <= 0 || dt > OXRSYS_WMR_CT_GYRO_MAX_INTERVAL_NS) {
		return OXRSYS_WMR_CT_GYRO_UNKNOWN;
	}
	struct xrt_quat gyro_delta;
	if (!oxrsys_wmr_ct_gyro_integrate(h, t0_ns, t1_ns, &gyro_delta)) {
		return OXRSYS_WMR_CT_GYRO_UNKNOWN;
	}

	// Optical rotation in the body frame: q1 = q0 * delta.
	struct xrt_quat q0_inv = {-q0->x, -q0->y, -q0->z, q0->w};
	struct xrt_quat optical_delta;
	quat_mul(&q0_inv, q1, &optical_delta);
	quat_normalize(&optical_delta);

	struct xrt_quat gyro_inv = {-gyro_delta.x, -gyro_delta.y, -gyro_delta.z, gyro_delta.w};
	struct xrt_quat diff;
	quat_mul(&gyro_inv, &optical_delta, &diff);

	const float mismatch = quat_angle_deg(&diff);
	const float gyro_angle = quat_angle_deg(&gyro_delta);
	if (out_mismatch_deg != NULL) {
		*out_mismatch_deg = mismatch;
	}
	if (out_gyro_deg != NULL) {
		*out_gyro_deg = gyro_angle;
	}
	const float allowed = base_deg + OXRSYS_WMR_CT_GYRO_ROTATION_FRACTION * gyro_angle;
	return mismatch <= allowed ? OXRSYS_WMR_CT_GYRO_AGREES : OXRSYS_WMR_CT_GYRO_DISAGREES;
}
