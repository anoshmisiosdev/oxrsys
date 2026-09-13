// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Offline test of the WMR controller gyro rotation check.
 *
 * Simulates a controller rotating at known body rates, samples its gyro at
 * 200 Hz, and checks oxrsys_wmr_ct_gyro_check() against optical orientations:
 * the true ones (with noise) must agree; a pose from the mirrored LED model
 * (the other hand's ring fitted to this controller's LEDs, solved here with
 * Horn's method like a least-squares tracker would) must disagree whether it
 * follows a true pose or another mirrored one; the other controller's motion
 * must disagree; uncovered intervals must give no verdict. Exit status 0 on
 * success.
 */

#include "wmr_ct_gyro_check.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//! LED positions of a left 1st-gen controller's calibration (metres, calibration axes).
static const float kLeftLeds[][3] = {
    {0.014315f, 0.053210f, -0.001485f},
    {0.043364f, 0.029342f, -0.007175f},
    {-0.041821f, 0.040456f, 0.006464f},
    {-0.008861f, 0.056053f, 0.002733f},
    {-0.009625f, -0.050945f, -0.009263f},
    {0.051508f, -0.002764f, -0.009129f},
    {0.026373f, -0.049791f, 0.000356f},
    {-0.057443f, 0.011010f, 0.006732f},
    {0.041668f, -0.030739f, -0.009175f},
    {-0.042819f, -0.036851f, 0.000194f},
    {0.055130f, -0.019640f, 0.006305f},
    {-0.052963f, -0.014138f, -0.003320f},
    {0.056479f, 0.013682f, 0.006374f},
    {-0.045756f, 0.023803f, -0.008867f},
    {0.035332f, 0.046088f, 0.006426f},
    {-0.026734f, -0.052133f, 0.006109f},
    {-0.025070f, 0.044922f, -0.008870f},
    {0.006084f, -0.058287f, 0.006157f},
    {-0.045326f, -0.003391f, -0.005141f},
    {-0.031309f, 0.031597f, -0.008174f},
    {0.002208f, 0.046338f, -0.001111f},
    {0.041781f, -0.016308f, -0.006901f},
    {0.012147f, -0.047961f, 0.003920f},
    {0.049314f, 0.003656f, 0.004368f},
    {0.024621f, 0.038074f, -0.004133f},
    {-0.027036f, -0.036092f, -0.006711f},
    {-0.011839f, -0.049032f, 0.005426f},
    {0.033708f, -0.036555f, 0.004459f},
    {-0.046129f, 0.018432f, 0.004847f},
    {0.043039f, 0.026642f, 0.006474f},
    {-0.044141f, -0.024824f, 0.005897f},
    {-0.020203f, 0.046147f, 0.006213f},
};
#define LED_COUNT (sizeof(kLeftLeds) / sizeof(kLeftLeds[0]))

static int g_failures = 0;

#define CHECK(cond, ...)                                                                                               \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "FAIL: " __VA_ARGS__);                                                         \
			fprintf(stderr, "\n");                                                                         \
			g_failures++;                                                                                  \
		}                                                                                                      \
	} while (0)

#define MS (1000 * 1000LL)

static const char *
result_str(enum oxrsys_wmr_ct_gyro_result r)
{
	switch (r) {
	case OXRSYS_WMR_CT_GYRO_AGREES: return "agrees";
	case OXRSYS_WMR_CT_GYRO_DISAGREES: return "disagrees";
	default: return "unknown";
	}
}

static struct xrt_quat
quat_axis_angle(float x, float y, float z, float deg)
{
	struct xrt_vec3 axis = {x, y, z};
	math_vec3_normalize(&axis);
	struct xrt_quat q;
	math_quat_from_angle_vector((float)(deg * M_PI / 180.0), &axis, &q);
	return q;
}

//! Body-frame motion: q(t) = q_start * exp(w * t), t in seconds from t = 0.
struct motion
{
	struct xrt_quat start;
	struct xrt_vec3 w;
};

static struct xrt_quat
orientation_at(const struct motion *m, int64_t t_ns)
{
	const double t = (double)t_ns / 1e9;
	const double wx = m->w.x * t, wy = m->w.y * t, wz = m->w.z * t;
	const double angle = sqrt(wx * wx + wy * wy + wz * wz);
	struct xrt_quat step = {0, 0, 0, 1};
	if (angle > 1e-12) {
		const double s = sin(angle / 2) / angle;
		step = (struct xrt_quat){(float)(wx * s), (float)(wy * s), (float)(wz * s), (float)cos(angle / 2)};
	}
	struct xrt_quat q;
	math_quat_rotate(&m->start, &step, &q);
	math_quat_normalize(&q);
	return q;
}

static void
fill_history(struct oxrsys_wmr_ct_gyro_history *h, const struct motion *m, int64_t from_ns, int64_t to_ns, int64_t step_ns)
{
	oxrsys_wmr_ct_gyro_reset(h);
	for (int64_t t = from_ns; t <= to_ns; t += step_ns) {
		oxrsys_wmr_ct_gyro_push(h, t, &m->w);
	}
}

//! Tracker (OpenXR) axes of a calibration LED; @p mirror gives the other hand's ring.
static struct xrt_vec3
model_led(size_t i, bool mirror)
{
	const float x = kLeftLeds[i][0];
	return (struct xrt_vec3){mirror ? -x : x, -kLeftLeds[i][1], -kLeftLeds[i][2]};
}

/*!
 * Best proper rotation Q minimising sum |Q a_i - b_i|^2 over centred point
 * sets (Horn 1987), by Jacobi eigen-decomposition of the 4x4 matrix.
 */
static struct xrt_quat
horn_fit(const struct xrt_vec3 *a, const struct xrt_vec3 *b, size_t n, double *out_rms)
{
	struct xrt_vec3 ca = {0}, cb = {0};
	for (size_t i = 0; i < n; i++) {
		ca = m_vec3_add(ca, a[i]);
		cb = m_vec3_add(cb, b[i]);
	}
	ca = m_vec3_mul_scalar(ca, 1.0f / (float)n);
	cb = m_vec3_mul_scalar(cb, 1.0f / (float)n);
	double S[3][3] = {{0}};
	for (size_t i = 0; i < n; i++) {
		const struct xrt_vec3 p = m_vec3_sub(a[i], ca), q = m_vec3_sub(b[i], cb);
		const double pa[3] = {p.x, p.y, p.z}, qa[3] = {q.x, q.y, q.z};
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				S[r][c] += pa[r] * qa[c];
			}
		}
	}
	const double Sxx = S[0][0], Sxy = S[0][1], Sxz = S[0][2], Syx = S[1][0], Syy = S[1][1], Syz = S[1][2],
	             Szx = S[2][0], Szy = S[2][1], Szz = S[2][2];
	double N[4][4] = {
	    {Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx},
	    {Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz},
	    {Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy},
	    {Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz},
	};
	double V[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
	for (int sweep = 0; sweep < 50; sweep++) {
		for (int p = 0; p < 4; p++) {
			for (int q = p + 1; q < 4; q++) {
				if (fabs(N[p][q]) < 1e-15) {
					continue;
				}
				const double theta = (N[q][q] - N[p][p]) / (2 * N[p][q]);
				const double t = (theta >= 0 ? 1 : -1) / (fabs(theta) + sqrt(theta * theta + 1));
				const double c = 1 / sqrt(t * t + 1), s = t * c;
				for (int k = 0; k < 4; k++) {
					const double nkp = N[k][p], nkq = N[k][q];
					N[k][p] = c * nkp - s * nkq;
					N[k][q] = s * nkp + c * nkq;
				}
				for (int k = 0; k < 4; k++) {
					const double npk = N[p][k], nqk = N[q][k];
					N[p][k] = c * npk - s * nqk;
					N[q][k] = s * npk + c * nqk;
				}
				for (int k = 0; k < 4; k++) {
					const double vkp = V[k][p], vkq = V[k][q];
					V[k][p] = c * vkp - s * vkq;
					V[k][q] = s * vkp + c * vkq;
				}
			}
		}
	}
	int best = 0;
	for (int i = 1; i < 4; i++) {
		if (N[i][i] > N[best][best]) {
			best = i;
		}
	}
	struct xrt_quat Q = {(float)V[1][best], (float)V[2][best], (float)V[3][best], (float)V[0][best]};
	math_quat_normalize(&Q);
	double err = 0;
	for (size_t i = 0; i < n; i++) {
		struct xrt_vec3 r;
		const struct xrt_vec3 p = m_vec3_sub(a[i], ca);
		math_quat_rotate_vec3(&Q, &p, &r);
		const struct xrt_vec3 d = m_vec3_sub(r, m_vec3_sub(b[i], cb));
		err += m_vec3_dot(d, d);
	}
	*out_rms = sqrt(err / (double)n);
	return Q;
}

/*!
 * Orientation the tracker would report when it fits the LEFT model to the
 * LEDs of a RIGHT controller at orientation @p q_right.
 */
static struct xrt_quat
mirrored_fit(const struct xrt_quat *q_right, double *out_rms)
{
	struct xrt_vec3 model[LED_COUNT], seen[LED_COUNT];
	for (size_t i = 0; i < LED_COUNT; i++) {
		model[i] = model_led(i, false);
		const struct xrt_vec3 r = model_led(i, true);
		math_quat_rotate_vec3(q_right, &r, &seen[i]);
	}
	return horn_fit(model, seen, LED_COUNT, out_rms);
}

static float
angle_between_deg(const struct xrt_quat *a, const struct xrt_quat *b)
{
	struct xrt_quat d;
	math_quat_unrotate(a, b, &d);
	return (float)(2.0 * acos(fmin(1.0, fabs((double)d.w))) * 180.0 / M_PI);
}

static enum oxrsys_wmr_ct_gyro_result
check(const struct oxrsys_wmr_ct_gyro_history *h,
      int64_t t0,
      const struct xrt_quat *q0,
      int64_t t1,
      const struct xrt_quat *q1,
      const char *what)
{
	float mismatch = -1, gyro = -1;
	enum oxrsys_wmr_ct_gyro_result r =
	    oxrsys_wmr_ct_gyro_check(h, t0, q0, t1, q1, OXRSYS_WMR_CT_GYRO_BASE_DEG, &mismatch, &gyro);
	if (r == OXRSYS_WMR_CT_GYRO_UNKNOWN) {
		printf("%-58s %s\n", what, result_str(r));
	} else {
		printf("%-58s %s (mismatch %.1f deg, gyro rotated %.1f deg)\n", what, result_str(r), mismatch, gyro);
	}
	return r;
}

int
main(void)
{
	struct oxrsys_wmr_ct_gyro_history h;
	const struct xrt_quat start = quat_axis_angle(0.3f, 1.0f, -0.2f, 70.0f);

	// 1. A constant body rate integrates to the true rotation.
	{
		struct motion m = {start, {1.5f, -2.0f, 0.7f}};
		fill_history(&h, &m, 0, 500 * MS, 5 * MS);
		struct xrt_quat delta;
		CHECK(oxrsys_wmr_ct_gyro_integrate(&h, 100 * MS, 220 * MS, &delta), "integrate covered interval");
		struct xrt_quat q0 = orientation_at(&m, 100 * MS), q1 = orientation_at(&m, 220 * MS), expect;
		math_quat_unrotate(&q0, &q1, &expect);
		const float err = angle_between_deg(&delta, &expect);
		printf("constant rate over 120 ms: integration error %.3f deg\n", err);
		CHECK(err < 0.5f, "integration error %.3f deg", err);
	}

	// 2. True optical poses, fast rotation, with a few degrees of pose noise: agree.
	{
		struct motion m = {start, {3.0f, 4.0f, -2.0f}}; // ~310 deg/s
		fill_history(&h, &m, 0, 1000 * MS, 5 * MS);
		struct xrt_quat q0 = orientation_at(&m, 300 * MS), q1 = orientation_at(&m, 400 * MS);
		struct xrt_quat n0 = quat_axis_angle(1, 0.2f, 0, 4.0f), n1 = quat_axis_angle(-0.3f, 1, 0.5f, 4.0f);
		math_quat_rotate(&q0, &n0, &q0);
		math_quat_rotate(&q1, &n1, &q1);
		CHECK(check(&h, 300 * MS, &q0, 400 * MS, &q1, "true poses, 310 deg/s, 4 deg noise each") ==
		          OXRSYS_WMR_CT_GYRO_AGREES,
		      "true poses should agree");
	}

	// 3. Still controller, true poses: agree.
	{
		struct motion m = {start, {0.01f, -0.01f, 0.0f}}; // gyro bias only
		fill_history(&h, &m, 0, 1000 * MS, 5 * MS);
		struct xrt_quat q0 = start, q1 = start;
		CHECK(check(&h, 500 * MS, &q0, 680 * MS, &q1, "still controller, true poses") == OXRSYS_WMR_CT_GYRO_AGREES,
		      "still controller should agree");
	}

	// 4. Mirrored fits: the left model fitted to the right controller's LEDs.
	{
		struct motion m = {start, {0.0f, 0.0f, 0.0f}};
		double rms = 0;
		struct xrt_quat truth = orientation_at(&m, 0);
		struct xrt_quat mirrored = mirrored_fit(&truth, &rms);
		// With LED i matched to mirrored LED i; the tracker may relabel LEDs
		// and fit even closer, but any such fit flips the near-planar ring.
		printf("mirrored fit: %.2f mm RMS residual on a %.0f mm ring, %.1f deg from the true orientation\n",
		       rms * 1000, 112.0, angle_between_deg(&truth, &mirrored));
		CHECK(rms < 0.02, "mirrored fit residual %.2f mm too large to be a plausible false fit", rms * 1000);

		// 4a. Still controller, true pose then mirrored pose: disagree.
		fill_history(&h, &m, 0, 1000 * MS, 5 * MS);
		CHECK(check(&h, 400 * MS, &truth, 450 * MS, &mirrored, "true pose then mirrored pose, still") ==
		          OXRSYS_WMR_CT_GYRO_DISAGREES,
		      "true then mirrored should disagree");

		// 4b. Mirrored pose then true pose (re-acquired): disagree too.
		CHECK(check(&h, 400 * MS, &mirrored, 450 * MS, &truth, "mirrored pose then true pose, still") ==
		          OXRSYS_WMR_CT_GYRO_DISAGREES,
		      "mirrored then true should disagree");
	}

	// 4c. Two consecutive mirrored fits while the controller rolls and pitches.
	{
		struct motion m = {start, {2.5f, 0.3f, 2.0f}}; // ~185 deg/s, mostly about body x and z
		fill_history(&h, &m, 0, 1000 * MS, 5 * MS);
		double rms = 0;
		struct xrt_quat t0 = orientation_at(&m, 600 * MS), t1 = orientation_at(&m, 700 * MS);
		struct xrt_quat m0 = mirrored_fit(&t0, &rms), m1 = mirrored_fit(&t1, &rms);
		CHECK(check(&h, 600 * MS, &m0, 700 * MS, &m1, "mirrored then mirrored, rolling 185 deg/s") ==
		          OXRSYS_WMR_CT_GYRO_DISAGREES,
		      "consecutive mirrored fits while rolling should disagree");

		// ...and while only turning about the axis the mirror flips the ring around (body y,
		// in the ring plane) they cannot be told apart.
		struct motion yaw = {start, {0.0f, 3.0f, 0.0f}};
		fill_history(&h, &yaw, 0, 1000 * MS, 5 * MS);
		t0 = orientation_at(&yaw, 600 * MS);
		t1 = orientation_at(&yaw, 700 * MS);
		m0 = mirrored_fit(&t0, &rms);
		m1 = mirrored_fit(&t1, &rms);
		enum oxrsys_wmr_ct_gyro_result r =
		    check(&h, 600 * MS, &m0, 700 * MS, &m1, "mirrored then mirrored, turning about body y only");
		printf("  (expected: no rotation check can separate these; the gravity check must)\n");
		(void)r;
	}

	// 5. Tracking the other controller: optical rotates 40 deg, this controller is still.
	{
		struct motion still = {start, {0.0f, 0.0f, 0.0f}};
		fill_history(&h, &still, 0, 1000 * MS, 5 * MS);
		struct xrt_quat turn = quat_axis_angle(0.2f, 1, 0.4f, 40.0f), q1;
		math_quat_rotate(&start, &turn, &q1);
		CHECK(check(&h, 200 * MS, &start, 300 * MS, &q1, "optical turns 40 deg, this gyro still") ==
		          OXRSYS_WMR_CT_GYRO_DISAGREES,
		      "other controller's motion should disagree");
	}

	// 6. No verdict when the history cannot cover the interval.
	{
		struct motion m = {start, {1.0f, 0.0f, 0.0f}};
		fill_history(&h, &m, 0, 1000 * MS, 5 * MS);
		struct xrt_quat q = start;
		CHECK(check(&h, 500 * MS, &q, 750 * MS, &q, "interval of 250 ms") == OXRSYS_WMR_CT_GYRO_UNKNOWN,
		      "long interval should be unknown");
		CHECK(check(&h, 500 * MS, &q, 500 * MS, &q, "zero interval") == OXRSYS_WMR_CT_GYRO_UNKNOWN,
		      "zero interval should be unknown");
		CHECK(check(&h, 960 * MS, &q, 1050 * MS, &q, "interval ends 50 ms after the newest sample") ==
		          OXRSYS_WMR_CT_GYRO_UNKNOWN,
		      "not yet arrived should be unknown");

		// Older than the ring buffer holds (256 samples at 5 ms = 1.28 s).
		fill_history(&h, &m, 0, 3000 * MS, 5 * MS);
		CHECK(check(&h, 100 * MS, &q, 200 * MS, &q, "interval older than the history") == OXRSYS_WMR_CT_GYRO_UNKNOWN,
		      "too old should be unknown");

		// A 50 ms Bluetooth gap inside the interval.
		oxrsys_wmr_ct_gyro_reset(&h);
		for (int64_t t = 0; t <= 1000 * MS; t += 5 * MS) {
			if (t > 400 * MS && t < 450 * MS) {
				continue;
			}
			oxrsys_wmr_ct_gyro_push(&h, t, &m.w);
		}
		CHECK(check(&h, 350 * MS, &q, 500 * MS, &q, "50 ms gap in the gyro inside the interval") ==
		          OXRSYS_WMR_CT_GYRO_UNKNOWN,
		      "gap should be unknown");
		struct xrt_quat q5 = orientation_at(&m, 500 * MS), q6 = orientation_at(&m, 600 * MS);
		CHECK(check(&h, 500 * MS, &q5, 600 * MS, &q6, "same history, true poses after the gap") ==
		          OXRSYS_WMR_CT_GYRO_AGREES,
		      "after the gap the check applies again");

		// Time going backwards resets the history.
		oxrsys_wmr_ct_gyro_push(&h, 10 * MS, &m.w);
		CHECK(h.count == 1, "backwards timestamp should reset the history (count %u)", h.count);
	}

	if (g_failures != 0) {
		fprintf(stderr, "%d check(s) failed\n", g_failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
