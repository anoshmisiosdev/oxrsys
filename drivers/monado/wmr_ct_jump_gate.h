// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Reject optical controller poses that jump further than the controller
 *        could have moved.
 *
 * A wrong constellation solve that passes the other checks (a mirrored fit
 * with both controllers tilted alike, a few LEDs matched to stray lights)
 * usually lands well away from the previous pose. The gate compares each
 * candidate with the last accepted pose: displacement is allowed up to
 * max(min_m, dt * speed), where speed is raised while the controller's IMU
 * shows hard motion. It never locks onto a wrong earlier pose: once the last
 * accepted pose is older than the stale window, or enough rejected candidates
 * agree among themselves, the new position is accepted.
 *
 * Plain state and functions; the caller does the locking.
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct oxrsys_wmr_ct_jump_gate_params
{
	//! Displacement always allowed, metres (solve noise, camera disagreement).
	float min_m;
	//! Speed allowed while the IMU shows normal motion, m/s.
	float max_speed_mps;
	//! Speed allowed while the IMU shows hard motion, m/s.
	float fast_speed_mps;
	//! Hard motion: |accel| differs from gravity by more than this, m/s^2.
	float fast_accel_dev_mps2;
	//! Hard motion: |gyro| above this, rad/s.
	float fast_gyro_rps;
	//! A last accepted pose older than this no longer constrains, ns.
	int64_t stale_ns;
	//! Rejected candidates consistent with each other that re-lock the gate.
	uint32_t relock_count;
};

struct oxrsys_wmr_ct_jump_gate
{
	bool have_accepted;
	int64_t accepted_ns;
	struct xrt_vec3 accepted_pos;

	//! Run of rejected candidates that agree with each other.
	bool have_candidate;
	int64_t candidate_ns;
	struct xrt_vec3 candidate_pos;
	uint32_t candidate_count;
};

enum oxrsys_wmr_ct_jump_gate_result
{
	//! Within reach of the last accepted pose.
	OXRSYS_WMR_CT_JUMP_ACCEPT,
	//! No recent accepted pose to compare with.
	OXRSYS_WMR_CT_JUMP_ACCEPT_STALE,
	//! Too far, but enough agreeing candidates: the controller really is there.
	OXRSYS_WMR_CT_JUMP_ACCEPT_RELOCK,
	//! Too far: drop this pose.
	OXRSYS_WMR_CT_JUMP_REJECT,
};

//! Defaults tuned for 60 Hz controller frames and hand-held motion.
void
oxrsys_wmr_ct_jump_gate_default_params(struct oxrsys_wmr_ct_jump_gate_params *params);

void
oxrsys_wmr_ct_jump_gate_reset(struct oxrsys_wmr_ct_jump_gate *gate);

//! Largest displacement allowed over @p dt_ns for the given IMU magnitudes.
float
oxrsys_wmr_ct_jump_gate_allowed_m(const struct oxrsys_wmr_ct_jump_gate_params *params,
                                  int64_t dt_ns,
                                  float accel_len_mps2,
                                  float gyro_len_rps);

/*!
 * Judge a candidate at @p when_ns, @p pos (any fixed frame, metres), with the
 * controller's current accelerometer and gyro magnitudes. Updates only the
 * rejected-candidate run; call oxrsys_wmr_ct_jump_gate_accept() once the pose
 * is finally kept (other checks may still drop it).
 */
enum oxrsys_wmr_ct_jump_gate_result
oxrsys_wmr_ct_jump_gate_check(struct oxrsys_wmr_ct_jump_gate *gate,
                              const struct oxrsys_wmr_ct_jump_gate_params *params,
                              int64_t when_ns,
                              struct xrt_vec3 pos,
                              float accel_len_mps2,
                              float gyro_len_rps);

//! Record a kept pose as the new reference. Older timestamps are ignored.
void
oxrsys_wmr_ct_jump_gate_accept(struct oxrsys_wmr_ct_jump_gate *gate, int64_t when_ns, struct xrt_vec3 pos);

#ifdef __cplusplus
}
#endif
