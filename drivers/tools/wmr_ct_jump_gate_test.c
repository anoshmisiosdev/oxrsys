// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Offline checks for the WMR controller pose jump gate.
 */

#include "wmr_ct_jump_gate.h"

#include <stdio.h>

#define MS (1000 * 1000LL)
//! 60 Hz controller frames.
#define FRAME_NS (16666667LL)
#define G 9.81f

static int g_failures = 0;

#define CHECK(cond, ...)                                                                                               \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "FAIL %s:%d: ", __func__, __LINE__);                                           \
			fprintf(stderr, __VA_ARGS__);                                                                  \
			fprintf(stderr, "\n");                                                                         \
			g_failures++;                                                                                  \
		}                                                                                                      \
	} while (0)

static bool
accepted(enum oxrsys_wmr_ct_jump_gate_result r)
{
	return r != OXRSYS_WMR_CT_JUMP_REJECT;
}

//! Check a candidate and, when accepted, keep it like the tracker does.
static enum oxrsys_wmr_ct_jump_gate_result
feed(struct oxrsys_wmr_ct_jump_gate *gate,
     const struct oxrsys_wmr_ct_jump_gate_params *p,
     int64_t t,
     struct xrt_vec3 pos,
     float accel,
     float gyro)
{
	enum oxrsys_wmr_ct_jump_gate_result r = oxrsys_wmr_ct_jump_gate_check(gate, p, t, pos, accel, gyro);
	if (accepted(r)) {
		oxrsys_wmr_ct_jump_gate_accept(gate, t, pos);
	}
	return r;
}

static void
test_smooth_motion(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);

	// 1.5 m/s sweep with 3 mm solve noise.
	int rejects = 0;
	for (int i = 0; i < 120; i++) {
		const float noise = (i % 3 - 1) * 0.003f;
		struct xrt_vec3 pos = {-0.3f + 1.5f * (float)i * (float)FRAME_NS / 1e9f, -0.2f + noise, -0.4f};
		enum oxrsys_wmr_ct_jump_gate_result r = feed(&gate, &p, i * FRAME_NS, pos, G, 0.5f);
		if (i == 0) {
			CHECK(r == OXRSYS_WMR_CT_JUMP_ACCEPT_STALE, "first pose should be accepted as stale, got %d", r);
		}
		rejects += !accepted(r);
	}
	CHECK(rejects == 0, "smooth motion had %d rejects", rejects);
}

static void
test_single_outlier(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);

	struct xrt_vec3 still = {0.1f, -0.3f, -0.4f};
	for (int i = 0; i < 10; i++) {
		feed(&gate, &p, i * FRAME_NS, still, G, 0.1f);
	}
	struct xrt_vec3 outlier = {0.4f, -0.3f, -0.4f}; // 30 cm away
	CHECK(feed(&gate, &p, 10 * FRAME_NS, outlier, G, 0.1f) == OXRSYS_WMR_CT_JUMP_REJECT, "30 cm outlier accepted");
	CHECK(accepted(feed(&gate, &p, 11 * FRAME_NS, still, G, 0.1f)), "return to the real pose rejected");
	// Isolated outliers interleaved with good poses never re-lock.
	int outliers_accepted = 0;
	for (int i = 12; i < 40; i += 2) {
		outliers_accepted += accepted(feed(&gate, &p, i * FRAME_NS, outlier, G, 0.1f));
		feed(&gate, &p, (i + 1) * FRAME_NS, still, G, 0.1f);
	}
	CHECK(outliers_accepted == 0, "%d interleaved outliers accepted", outliers_accepted);
}

static void
test_relocation(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);

	// The gate was wrongly holding a pose; the controller is really 40 cm away.
	struct xrt_vec3 wrong = {-0.2f, -0.3f, -0.4f};
	for (int i = 0; i < 10; i++) {
		feed(&gate, &p, i * FRAME_NS, wrong, G, 0.1f);
	}
	struct xrt_vec3 real = {0.2f, -0.3f, -0.4f};
	int first_accept = -1;
	for (int i = 0; i < 10; i++) {
		struct xrt_vec3 pos = real;
		pos.y += (i % 2) * 0.004f;
		if (accepted(feed(&gate, &p, (10 + i) * FRAME_NS, pos, G, 0.1f)) && first_accept < 0) {
			first_accept = i;
		}
	}
	CHECK(first_accept >= 0 && first_accept <= (int)p.relock_count - 1,
	      "relocation accepted after %d samples (relock_count %u)", first_accept, p.relock_count);
	// And it stays there.
	CHECK(feed(&gate, &p, 21 * FRAME_NS, real, G, 0.1f) == OXRSYS_WMR_CT_JUMP_ACCEPT, "not locked to the new pose");
	CHECK(feed(&gate, &p, 22 * FRAME_NS, wrong, G, 0.1f) == OXRSYS_WMR_CT_JUMP_REJECT, "old pose not rejected");
}

static void
test_stale(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);

	feed(&gate, &p, 0, (struct xrt_vec3){0.0f, -0.3f, -0.4f}, G, 0.1f);
	// Out of view for 200 ms, reappears 50 cm away.
	enum oxrsys_wmr_ct_jump_gate_result r =
	    feed(&gate, &p, 200 * MS, (struct xrt_vec3){0.5f, -0.3f, -0.4f}, G, 0.1f);
	CHECK(r == OXRSYS_WMR_CT_JUMP_ACCEPT_STALE, "stale previous pose did not accept, got %d", r);
	// Within the stale window the same jump is rejected.
	r = feed(&gate, &p, 300 * MS, (struct xrt_vec3){0.0f, -0.3f, -0.4f}, G, 0.1f);
	CHECK(r == OXRSYS_WMR_CT_JUMP_REJECT, "fresh 50 cm jump over 100 ms accepted, got %d", r);
}

static void
test_fast_imu(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);

	const float calm = oxrsys_wmr_ct_jump_gate_allowed_m(&p, 2 * FRAME_NS, G, 0.2f);
	const float accel = oxrsys_wmr_ct_jump_gate_allowed_m(&p, 2 * FRAME_NS, G + 12.0f, 0.2f);
	const float spin = oxrsys_wmr_ct_jump_gate_allowed_m(&p, 2 * FRAME_NS, G, 8.0f);
	CHECK(accel > calm * 2.0f && spin > calm * 2.0f, "fast IMU did not widen: calm %.3f accel %.3f spin %.3f", calm,
	      accel, spin);

	// A punch: 6 m/s, 10 cm between frames. Rejected when the IMU is calm,
	// accepted when it shows the motion.
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);
	feed(&gate, &p, 0, (struct xrt_vec3){0.0f, -0.2f, -0.3f}, G, 0.1f);
	CHECK(oxrsys_wmr_ct_jump_gate_check(&gate, &p, FRAME_NS, (struct xrt_vec3){0.0f, -0.2f, -0.4f}, G, 0.1f) ==
	          OXRSYS_WMR_CT_JUMP_REJECT,
	      "10 cm in one frame accepted with a calm IMU");
	oxrsys_wmr_ct_jump_gate_reset(&gate);
	feed(&gate, &p, 0, (struct xrt_vec3){0.0f, -0.2f, -0.3f}, G, 0.1f);
	CHECK(oxrsys_wmr_ct_jump_gate_check(&gate, &p, FRAME_NS, (struct xrt_vec3){0.0f, -0.2f, -0.4f}, G + 25.0f,
	                                    6.0f) == OXRSYS_WMR_CT_JUMP_ACCEPT,
	      "10 cm in one frame rejected during a punch");
}

static void
test_out_of_order(void)
{
	struct oxrsys_wmr_ct_jump_gate_params p;
	oxrsys_wmr_ct_jump_gate_default_params(&p);
	struct oxrsys_wmr_ct_jump_gate gate;
	oxrsys_wmr_ct_jump_gate_reset(&gate);

	// Two cameras deliver the same frame out of order, 2 cm apart.
	feed(&gate, &p, 2 * FRAME_NS, (struct xrt_vec3){0.0f, -0.2f, -0.3f}, G, 0.1f);
	CHECK(accepted(feed(&gate, &p, FRAME_NS, (struct xrt_vec3){0.02f, -0.2f, -0.3f}, G, 0.1f)),
	      "older sample from the other camera rejected");
	CHECK(gate.accepted_ns == 2 * FRAME_NS, "older sample replaced the newer reference");
}

int
main(void)
{
	test_smooth_motion();
	test_single_outlier();
	test_relocation();
	test_stale();
	test_fast_imu();
	test_out_of_order();
	if (g_failures != 0) {
		fprintf(stderr, "%d check(s) failed\n", g_failures);
		return 1;
	}
	printf("jump gate: all checks passed\n");
	return 0;
}
