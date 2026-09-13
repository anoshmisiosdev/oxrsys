// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Check an optical controller rotation against the controller's gyro.
 *
 * The left and right 1st-gen WMR LED rings are mirror images, so a mirrored
 * fit can pass the gravity check when both controllers are tilted alike. Its
 * rotation between two optical poses still has to match what the controller's
 * own gyro measured over the same interval; a pose that jumps between the
 * right and the mirrored model, or follows the other controller's motion,
 * does not.
 *
 * Not thread-safe: the caller serialises push and check (one lock per hand).
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Gyro history kept per controller; 200 Hz reports give about 1.3 s.
#define OXRSYS_WMR_CT_GYRO_SAMPLES 256

//! Longest interval between two optical poses that is checked.
#define OXRSYS_WMR_CT_GYRO_MAX_INTERVAL_NS (200 * 1000 * 1000LL)

//! Largest gap between gyro samples inside a checked interval.
#define OXRSYS_WMR_CT_GYRO_MAX_GAP_NS (30 * 1000 * 1000LL)

//! How far the history may fall short of either end of the interval.
#define OXRSYS_WMR_CT_GYRO_END_SLACK_NS (10 * 1000 * 1000LL)

//! Default allowed mismatch: base degrees plus this fraction of the rotation.
#define OXRSYS_WMR_CT_GYRO_BASE_DEG 20.0f
#define OXRSYS_WMR_CT_GYRO_ROTATION_FRACTION 0.25f

struct oxrsys_wmr_ct_gyro_sample
{
	//! Monotonic time of the IMU sample, ns.
	int64_t timestamp_ns;
	//! Angular velocity in the controller body axes, rad/s.
	struct xrt_vec3 gyro;
};

struct oxrsys_wmr_ct_gyro_history
{
	struct oxrsys_wmr_ct_gyro_sample samples[OXRSYS_WMR_CT_GYRO_SAMPLES];
	//! Index of the next write.
	uint32_t next;
	//! Valid entries (at most OXRSYS_WMR_CT_GYRO_SAMPLES).
	uint32_t count;
};

enum oxrsys_wmr_ct_gyro_result
{
	//! The history does not cover the interval, or it is too long: no verdict.
	OXRSYS_WMR_CT_GYRO_UNKNOWN = 0,
	OXRSYS_WMR_CT_GYRO_AGREES,
	OXRSYS_WMR_CT_GYRO_DISAGREES,
};

void
oxrsys_wmr_ct_gyro_reset(struct oxrsys_wmr_ct_gyro_history *h);

/*!
 * Append a sample. Samples must arrive in time order; one older than the
 * newest resets the history (the controller clock restarted).
 */
void
oxrsys_wmr_ct_gyro_push(struct oxrsys_wmr_ct_gyro_history *h, int64_t timestamp_ns, const struct xrt_vec3 *gyro);

/*!
 * Integrate the gyro over [@p t0_ns, @p t1_ns] into a body-frame rotation
 * (the orientation at t1 is q(t0) * @p out_delta). False when the history
 * does not cover the interval (gaps, too old, not yet arrived).
 */
bool
oxrsys_wmr_ct_gyro_integrate(const struct oxrsys_wmr_ct_gyro_history *h,
                             int64_t t0_ns,
                             int64_t t1_ns,
                             struct xrt_quat *out_delta);

/*!
 * Compare the optical rotation from @p q0 (at @p t0_ns) to @p q1 (at
 * @p t1_ns), both world-from-controller orientations, with the gyro.
 * Allowed mismatch: @p base_deg plus OXRSYS_WMR_CT_GYRO_ROTATION_FRACTION of
 * the gyro's rotation over the interval. @p out_mismatch_deg and
 * @p out_gyro_deg get the angles when the result is not UNKNOWN (may be NULL).
 */
enum oxrsys_wmr_ct_gyro_result
oxrsys_wmr_ct_gyro_check(const struct oxrsys_wmr_ct_gyro_history *h,
                         int64_t t0_ns,
                         const struct xrt_quat *q0,
                         int64_t t1_ns,
                         const struct xrt_quat *q1,
                         float base_deg,
                         float *out_mismatch_deg,
                         float *out_gyro_deg);

#ifdef __cplusplus
}
#endif
