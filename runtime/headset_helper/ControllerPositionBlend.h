// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Decide when to trust a controller's optical (camera) position and
 *        blend between it and the arm model instead of snapping.
 *
 * One instance per hand, updated every tracking tick:
 *
 * - **Confirm.** Optical positions are used only after a run of distinct,
 *   fresh samples (new timestamps, close together in time and space). A
 *   single stray solve never moves the controller.
 * - **Blend.** Entering and leaving optical tracking ramps a weight between
 *   0 (arm model) and 1 (optical) with a smoothstep over a short time, so the
 *   position never jumps. While leaving, the last optical position (head
 *   relative) is faded out. A reversal mid-ramp continues from the current
 *   weight.
 * - **Dropouts.** The caller reports a sample only while it is younger than
 *   the stale window; a gap shorter than that keeps the controller optical.
 * - **Jumps.** While trusted, a new sample farther from the previous one than
 *   the controller could plausibly move ends the trust: the last good
 *   position fades out and a new run has to confirm again. A run confirmed
 *   somewhere the fading position could not have reached waits until the
 *   fade has finished.
 *
 * Plain C++ with no Monado or AppKit types, so it can be unit tested.
 */

#pragma once

#include <cstdint>

namespace oxrsys {

struct BlendVec3
{
	float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct ControllerBlendParams
{
	//! Distinct consistent samples needed before optical positions are used.
	int confirmSamples = 3;
	//! Largest gap between consecutive samples of a confirming run.
	int64_t maxSampleGapNs = 100000000LL;
	//! Largest distance between consecutive samples, plus maxSpeedMps per
	//! second of gap between them.
	float consistencyM = 0.10f;
	float maxSpeedMps = 3.0f;
	//! Duration of a full arm-model <-> optical ramp.
	int64_t blendNs = 100000000LL;
};

class ControllerPositionBlend
{
public:
	explicit ControllerPositionBlend(const ControllerBlendParams &params = ControllerBlendParams());

	/*!
	 * Advance to monotonic time @p nowNs. @p haveSample: the tracker has an
	 * optical sample younger than the stale window, taken at
	 * @p sampleTimestampNs with head-relative position @p headRelative
	 * (ignored otherwise). The same sample may be reported on many ticks.
	 */
	void
	Update(int64_t nowNs, bool haveSample, int64_t sampleTimestampNs, const BlendVec3 &headRelative);

	//! 0 = arm model only, 1 = optical only.
	float
	Weight() const;

	//! Optical positions are confirmed and current (the weight is rising or 1).
	bool
	Trusted() const
	{
		return trusted_;
	}

	/*!
	 * Head-relative optical position to blend with (the latest confirmed
	 * sample, or the last one while fading out). Valid while Weight() > 0.
	 */
	const BlendVec3 &
	OpticalHeadRelative() const
	{
		return optical_;
	}

	//! arm * (1 - w) + optical * w.
	static BlendVec3
	Mix(const BlendVec3 &arm, const BlendVec3 &optical, float weight);

	void
	Reset();

private:
	ControllerBlendParams params_;
	bool trusted_ = false;
	//! Linear ramp position in [0, 1]; the weight is its smoothstep.
	float ramp_ = 0.0f;
	int64_t lastUpdateNs_ = 0;
	bool haveLastUpdate_ = false;

	// Confirming run.
	int runCount_ = 0;
	int64_t lastSampleNs_ = 0;
	BlendVec3 lastSamplePos_;
	bool haveLastSample_ = false;
	int64_t runStartNs_ = 0;
	BlendVec3 runStartPos_;

	BlendVec3 optical_;
	int64_t opticalNs_ = 0;
};

} // namespace oxrsys
