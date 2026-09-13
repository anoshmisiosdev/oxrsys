// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Offline check of the optical controller position smoothing.
 *
 * Feeds wmr_ct_smoothing synthetic 60 Hz controller positions: a still
 * controller with +-3 mm noise (about 1-2 px of reprojection error at
 * 0.3-0.5 m), a 1 m/s sweep, a reacquisition after a gap, a duplicate
 * timestamp and the disabled pass-through. Exit status 0 on success.
 * `wmr_ct_smoothing_test sweep` also prints jitter and lag for a range of
 * parameters.
 */

#include "wmr_ct_smoothing.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RATE_HZ 60
#define STEP_NS (1000000000LL / RATE_HZ)
#define NOISE_M 0.003f

static int g_failures = 0;

#define CHECK(cond, ...)                                                                                               \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "FAIL: " __VA_ARGS__);                                                         \
			fprintf(stderr, "\n");                                                                         \
			g_failures++;                                                                                  \
		}                                                                                                      \
	} while (0)

static uint32_t g_rand = 987654321u;

//! Uniform in [-1, 1].
static float
urand(void)
{
	g_rand = g_rand * 1664525u + 1013904223u;
	return (float)((g_rand >> 8) & 0xffffff) / (float)0xffffff * 2.0f - 1.0f;
}

struct result
{
	double jitter_in_mm;  //!< RMS of the still input around its true position.
	double jitter_out_mm; //!< RMS of the still output around its true position.
	double lag_ms;        //!< Distance behind a 1 m/s sweep, as time.
};

static struct result
measure(double min_cutoff, double d_cutoff, double beta, bool noisy_sweep)
{
	struct oxrsys_wmr_ct_smoothing s;
	oxrsys_wmr_ct_smoothing_init_params(&s, true, min_cutoff, d_cutoff, beta);
	const struct xrt_vec3 rest = {0.10f, -0.30f, -0.40f};
	int64_t t = 1000000000LL;

	// Still for 4 s; measure over the last 3 s.
	double in2 = 0.0, out2 = 0.0;
	int n = 0;
	for (int i = 0; i < 4 * RATE_HZ; i++, t += STEP_NS) {
		struct xrt_vec3 m = {rest.x + NOISE_M * urand(), rest.y + NOISE_M * urand(), rest.z + NOISE_M * urand()};
		struct xrt_vec3 o = oxrsys_wmr_ct_smoothing_run(&s, t, m);
		if (i >= RATE_HZ) {
			for (int k = 0; k < 3; k++) {
				const double mi = k == 0 ? m.x - rest.x : k == 1 ? m.y - rest.y : m.z - rest.z;
				const double oi = k == 0 ? o.x - rest.x : k == 1 ? o.y - rest.y : o.z - rest.z;
				in2 += mi * mi;
				out2 += oi * oi;
			}
			n++;
		}
	}

	// Sweep at 1 m/s along x for 0.5 s; lag averaged over its last 0.2 s.
	const float speed = 1.0f;
	struct xrt_vec3 truth = rest;
	double lag_sum = 0.0;
	int lag_n = 0;
	for (int i = 0; i < RATE_HZ / 2; i++, t += STEP_NS) {
		truth.x += speed / RATE_HZ;
		struct xrt_vec3 m = truth;
		if (noisy_sweep) {
			m.x += NOISE_M * urand();
			m.y += NOISE_M * urand();
			m.z += NOISE_M * urand();
		}
		struct xrt_vec3 o = oxrsys_wmr_ct_smoothing_run(&s, t, m);
		if (i >= RATE_HZ / 2 - RATE_HZ / 5) {
			lag_sum += (truth.x - o.x) / speed;
			lag_n++;
		}
	}

	struct result r = {
	    .jitter_in_mm = sqrt(in2 / (3.0 * n)) * 1000.0,
	    .jitter_out_mm = sqrt(out2 / (3.0 * n)) * 1000.0,
	    .lag_ms = lag_sum / lag_n * 1000.0,
	};
	return r;
}

static void
sweep(void)
{
	static const double min_cutoffs[] = {0.5, 1.0, 1.5, 2.0, 3.0};
	static const double betas[] = {0.0, 2.0, 4.0, 6.0, 10.0, 20.0};
	static const double d_cutoffs[] = {1.0, 3.0};
	printf("mincutoff dcutoff beta  jitter in/out mm  lag ms (1 m/s)\n");
	for (size_t d = 0; d < 2; d++) {
		for (size_t i = 0; i < 5; i++) {
			for (size_t j = 0; j < 6; j++) {
				g_rand = 987654321u;
				struct result r = measure(min_cutoffs[i], d_cutoffs[d], betas[j], false);
				printf("%9.1f %7.1f %4.0f  %5.2f / %5.2f     %6.1f\n", min_cutoffs[i], d_cutoffs[d], betas[j],
				       r.jitter_in_mm, r.jitter_out_mm, r.lag_ms);
			}
		}
	}
}

int
main(int argc, char **argv)
{
	if (argc > 1 && strcmp(argv[1], "sweep") == 0) {
		sweep();
		return 0;
	}

	// Defaults: jitter and lag.
	struct result r = measure(OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF, OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF,
	                          OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA, false);
	struct result rn = measure(OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF, OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF,
	                           OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA, true);
	printf("defaults mincutoff %.1f Hz, dcutoff %.1f Hz, beta %.1f: still jitter %.2f -> %.2f mm RMS (x%.1f), "
	       "lag at 1 m/s %.1f ms (%.1f ms with noise)\n",
	       OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF, OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF,
	       OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA, r.jitter_in_mm, r.jitter_out_mm, r.jitter_in_mm / r.jitter_out_mm,
	       r.lag_ms, rn.lag_ms);
	CHECK(r.jitter_out_mm < r.jitter_in_mm / 3.0, "still jitter only reduced from %.2f to %.2f mm", r.jitter_in_mm,
	      r.jitter_out_mm);
	CHECK(r.lag_ms < 30.0, "lag %.1f ms at 1 m/s", r.lag_ms);
	CHECK(rn.lag_ms < 30.0, "lag %.1f ms at 1 m/s with noise", rn.lag_ms);

	// Reacquisition after a gap snaps to the new position.
	struct oxrsys_wmr_ct_smoothing s;
	oxrsys_wmr_ct_smoothing_init_params(&s, true, OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF,
	                                    OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF, OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA);
	int64_t t = 5000000000LL;
	for (int i = 0; i < RATE_HZ; i++, t += STEP_NS) {
		oxrsys_wmr_ct_smoothing_run(&s, t, (struct xrt_vec3){0.0f, 0.0f, -0.3f});
	}
	const struct xrt_vec3 moved = {0.4f, -0.2f, -0.5f};
	t += OXRSYS_WMR_CT_SMOOTH_RESET_NS + STEP_NS;
	struct xrt_vec3 o = oxrsys_wmr_ct_smoothing_run(&s, t, moved);
	CHECK(o.x == moved.x && o.y == moved.y && o.z == moved.z, "after a gap the output is (%.3f, %.3f, %.3f)", o.x,
	      o.y, o.z);
	printf("after a %lld ms gap: output (%.3f, %.3f, %.3f) for input (%.3f, %.3f, %.3f)\n",
	       (long long)((OXRSYS_WMR_CT_SMOOTH_RESET_NS + STEP_NS) / 1000000), o.x, o.y, o.z, moved.x, moved.y,
	       moved.z);

	// A short gap does not reset: the output moves only part way.
	t += STEP_NS;
	oxrsys_wmr_ct_smoothing_run(&s, t, moved);
	t += 50000000LL;
	o = oxrsys_wmr_ct_smoothing_run(&s, t, (struct xrt_vec3){0.5f, -0.2f, -0.5f});
	CHECK(o.x > 0.4f && o.x < 0.5f, "after a 50 ms gap x is %.3f, expected between 0.4 and 0.5", o.x);

	// The same timestamp again (the other camera) keeps the previous output.
	struct xrt_vec3 again = oxrsys_wmr_ct_smoothing_run(&s, t, (struct xrt_vec3){9.0f, 9.0f, 9.0f});
	CHECK(again.x == o.x && again.y == o.y && again.z == o.z, "duplicate timestamp changed the output");
	CHECK(isfinite(again.x) && isfinite(again.y) && isfinite(again.z), "non-finite output");

	// Disabled passes through.
	oxrsys_wmr_ct_smoothing_init_params(&s, false, 1.0, 1.0, 1.0);
	o = oxrsys_wmr_ct_smoothing_run(&s, t, moved);
	CHECK(o.x == moved.x && o.y == moved.y && o.z == moved.z, "disabled filter changed the input");

	if (g_failures != 0) {
		fprintf(stderr, "%d check(s) failed\n", g_failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
