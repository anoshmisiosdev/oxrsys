// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Reject optical controller poses that jump further than the controller
 *        could have moved.
 */

#include "wmr_ct_jump_gate.h"

#include <math.h>
#include <string.h>

#define GRAVITY_MPS2 9.80665f

static float
distance(struct xrt_vec3 a, struct xrt_vec3 b)
{
	const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
	return sqrtf(dx * dx + dy * dy + dz * dz);
}

static int64_t
abs_ns(int64_t v)
{
	return v < 0 ? -v : v;
}

void
oxrsys_wmr_ct_jump_gate_default_params(struct oxrsys_wmr_ct_jump_gate_params *params)
{
	*params = (struct oxrsys_wmr_ct_jump_gate_params){
	    .min_m = 0.05f,
	    .max_speed_mps = 3.0f,
	    .fast_speed_mps = 10.0f,
	    .fast_accel_dev_mps2 = 5.0f,
	    .fast_gyro_rps = 4.0f,
	    .stale_ns = 150 * 1000 * 1000LL,
	    .relock_count = 3,
	};
}

void
oxrsys_wmr_ct_jump_gate_reset(struct oxrsys_wmr_ct_jump_gate *gate)
{
	memset(gate, 0, sizeof(*gate));
}

float
oxrsys_wmr_ct_jump_gate_allowed_m(const struct oxrsys_wmr_ct_jump_gate_params *params,
                                  int64_t dt_ns,
                                  float accel_len_mps2,
                                  float gyro_len_rps)
{
	const bool fast = fabsf(accel_len_mps2 - GRAVITY_MPS2) > params->fast_accel_dev_mps2 ||
	                  gyro_len_rps > params->fast_gyro_rps;
	const float speed = fast ? params->fast_speed_mps : params->max_speed_mps;
	const float reach = (float)((double)abs_ns(dt_ns) / 1e9) * speed;
	return reach > params->min_m ? reach : params->min_m;
}

enum oxrsys_wmr_ct_jump_gate_result
oxrsys_wmr_ct_jump_gate_check(struct oxrsys_wmr_ct_jump_gate *gate,
                              const struct oxrsys_wmr_ct_jump_gate_params *params,
                              int64_t when_ns,
                              struct xrt_vec3 pos,
                              float accel_len_mps2,
                              float gyro_len_rps)
{
	if (!gate->have_accepted || abs_ns(when_ns - gate->accepted_ns) > params->stale_ns) {
		gate->have_candidate = false;
		gate->candidate_count = 0;
		return OXRSYS_WMR_CT_JUMP_ACCEPT_STALE;
	}

	const float allowed = oxrsys_wmr_ct_jump_gate_allowed_m(params, when_ns - gate->accepted_ns, accel_len_mps2,
	                                                         gyro_len_rps);
	if (distance(pos, gate->accepted_pos) <= allowed) {
		gate->have_candidate = false;
		gate->candidate_count = 0;
		return OXRSYS_WMR_CT_JUMP_ACCEPT;
	}

	// Too far from the reference. Follow the run of rejected candidates: one
	// that agrees with the previous candidate extends it, anything else
	// starts a new run.
	if (gate->have_candidate && abs_ns(when_ns - gate->candidate_ns) <= params->stale_ns &&
	    distance(pos, gate->candidate_pos) <= oxrsys_wmr_ct_jump_gate_allowed_m(params, when_ns - gate->candidate_ns,
	                                                                           accel_len_mps2, gyro_len_rps)) {
		gate->candidate_count++;
	} else {
		gate->have_candidate = true;
		gate->candidate_count = 1;
	}
	gate->candidate_ns = when_ns;
	gate->candidate_pos = pos;

	if (gate->candidate_count >= params->relock_count) {
		gate->have_candidate = false;
		gate->candidate_count = 0;
		return OXRSYS_WMR_CT_JUMP_ACCEPT_RELOCK;
	}
	return OXRSYS_WMR_CT_JUMP_REJECT;
}

void
oxrsys_wmr_ct_jump_gate_accept(struct oxrsys_wmr_ct_jump_gate *gate, int64_t when_ns, struct xrt_vec3 pos)
{
	if (gate->have_accepted && when_ns < gate->accepted_ns) {
		return;
	}
	gate->have_accepted = true;
	gate->accepted_ns = when_ns;
	gate->accepted_pos = pos;
}
