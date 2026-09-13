// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Jitter smoothing for optically tracked WMR controller positions.
 */

#include "wmr_ct_smoothing.h"

#include "util/u_debug.h"

#include <string.h>

DEBUG_GET_ONCE_BOOL_OPTION(ct_smooth, "OXRSYS_WMR_CT_SMOOTH", true)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_smooth_mincutoff, "OXRSYS_WMR_CT_SMOOTH_MINCUTOFF",
                            (float)OXRSYS_WMR_CT_SMOOTH_DEFAULT_MINCUTOFF)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_smooth_beta, "OXRSYS_WMR_CT_SMOOTH_BETA", (float)OXRSYS_WMR_CT_SMOOTH_DEFAULT_BETA)

void
oxrsys_wmr_ct_smoothing_init_params(
    struct oxrsys_wmr_ct_smoothing *s, bool enabled, double min_cutoff_hz, double d_cutoff_hz, double beta)
{
	memset(s, 0, sizeof(*s));
	s->enabled = enabled;
	m_filter_euro_vec3_init(&s->filter, min_cutoff_hz, d_cutoff_hz, beta);
}

void
oxrsys_wmr_ct_smoothing_init(struct oxrsys_wmr_ct_smoothing *s)
{
	oxrsys_wmr_ct_smoothing_init_params(s, debug_get_bool_option_ct_smooth(),
	                                    debug_get_float_option_ct_smooth_mincutoff(),
	                                    OXRSYS_WMR_CT_SMOOTH_DEFAULT_DCUTOFF, debug_get_float_option_ct_smooth_beta());
}

struct xrt_vec3
oxrsys_wmr_ct_smoothing_run(struct oxrsys_wmr_ct_smoothing *s, int64_t timestamp_ns, struct xrt_vec3 position)
{
	if (!s->enabled) {
		return position;
	}
	if (s->have_output) {
		if (timestamp_ns <= s->last_timestamp_ns) {
			// Same frame from the other camera, or out of order: the filter
			// divides by the time step, so leave it alone.
			return s->last_output;
		}
		if (timestamp_ns - s->last_timestamp_ns > OXRSYS_WMR_CT_SMOOTH_RESET_NS) {
			// Reacquired after a gap: start at the new position.
			const struct m_filter_one_euro_base b = s->filter.base;
			m_filter_euro_vec3_init(&s->filter, b.fc_min, b.fc_min_d, b.beta);
		}
	}
	// m_filter_euro takes nanosecond timestamps.
	m_filter_euro_vec3_run(&s->filter, (uint64_t)timestamp_ns, &position, &s->last_output);
	s->last_timestamp_ns = timestamp_ns;
	s->have_output = true;
	return s->last_output;
}
