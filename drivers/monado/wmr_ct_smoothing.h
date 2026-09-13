// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Jitter smoothing for optically tracked WMR controller positions.
 *
 * A One Euro filter (Casiez et al., CHI 2012; Monado's m_filter_euro_vec3):
 * a low-pass filter whose cutoff rises with the filtered speed, so a still
 * controller's reprojection jitter is smoothed heavily while a moving one
 * follows with little lag. Run it on positions in a frame that does not turn
 * with the head (the constellation tracker's world), so head rotation is not
 * delayed.
 *
 * Defaults, overridable from the environment:
 *   OXRSYS_WMR_CT_SMOOTH=0            pass positions through unfiltered
 *   OXRSYS_WMR_CT_SMOOTH_MINCUTOFF    cutoff at rest, Hz
 *   OXRSYS_WMR_CT_SMOOTH_BETA         extra cutoff per m/s of speed, Hz
 */

#pragma once

#include "math/m_filter_one_euro.h"
#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Cutoff frequency at rest (Hz).
#define OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF 1.0
//! Cutoff of the speed estimate (Hz).
#define OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF 1.0
//! Cutoff increase per m/s (Hz / (m/s)).
#define OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA 6.0
//! A gap longer than this since the last sample restarts the filter at the new position.
#define OXRSYS_WMR_CT_SMOOTH_RESET_NS (200 * 1000 * 1000LL)

struct oxrsys_wmr_ct_smoothing
{
	bool enabled;
	struct m_filter_euro_vec3 filter;
	bool have_output;
	int64_t last_timestamp_ns;
	struct xrt_vec3 last_output;
};

//! Set up with the defaults, or the environment overrides.
void
oxrsys_wmr_ct_smoothing_init(struct oxrsys_wmr_ct_smoothing *s);

//! Set up with explicit parameters (tests).
void
oxrsys_wmr_ct_smoothing_init_params(
    struct oxrsys_wmr_ct_smoothing *s, bool enabled, double min_cutoff_hz, double d_cutoff_hz, double beta);

/*!
 * Filter @p position measured at @p timestamp_ns (monotonic). Samples older
 * than or at the same time as the previous one (for example the same frame
 * seen by the other camera) return the previous output without updating the
 * filter.
 */
struct xrt_vec3
oxrsys_wmr_ct_smoothing_run(struct oxrsys_wmr_ct_smoothing *s, int64_t timestamp_ns, struct xrt_vec3 position);

#ifdef __cplusplus
}
#endif
