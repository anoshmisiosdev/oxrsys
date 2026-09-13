// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Confirm-then-blend between a controller's arm model and optical position.
 */

#include "ControllerPositionBlend.h"

#include <algorithm>
#include <cmath>

namespace oxrsys {

ControllerPositionBlend::ControllerPositionBlend(const ControllerBlendParams &params) : params_(params)
{
	params_.confirmSamples = std::max(params_.confirmSamples, 1);
	params_.blendNs = std::max<int64_t>(params_.blendNs, 1);
}

void
ControllerPositionBlend::Reset()
{
	*this = ControllerPositionBlend(params_);
}

void
ControllerPositionBlend::Update(int64_t nowNs, bool haveSample, int64_t sampleTimestampNs, const BlendVec3 &headRelative)
{
	int64_t dt = 0;
	if (haveLastUpdate_) {
		dt = std::clamp<int64_t>(nowNs - lastUpdateNs_, 0, params_.blendNs);
	}
	lastUpdateNs_ = nowNs;
	haveLastUpdate_ = true;

	if (!haveSample) {
		// Stale: stop trusting, fade out from the last optical position.
		trusted_ = false;
		runCount_ = 0;
		haveLastSample_ = false;
	} else if (!haveLastSample_ || sampleTimestampNs != lastSampleNs_) {
		// A new sample: extend the confirming run or start a new one.
		bool consistent = false;
		bool closeInTime = false;
		if (haveLastSample_) {
			const int64_t gap = sampleTimestampNs - lastSampleNs_;
			const float dx = headRelative.x - lastSamplePos_.x;
			const float dy = headRelative.y - lastSamplePos_.y;
			const float dz = headRelative.z - lastSamplePos_.z;
			const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
			consistent = gap > 0 && dist <= params_.consistencyM + params_.maxSpeedMps * (float)gap / 1e9f;
			closeInTime = gap <= params_.maxSampleGapNs;
		}
		if (trusted_ && !consistent) {
			// Implausible jump: fade out the last good position, confirm again.
			trusted_ = false;
		}
		runCount_ = (consistent && closeInTime) ? runCount_ + 1 : 1;
		if (runCount_ == 1) {
			runStartNs_ = sampleTimestampNs;
			runStartPos_ = headRelative;
		}
		lastSampleNs_ = sampleTimestampNs;
		lastSamplePos_ = headRelative;
		haveLastSample_ = true;

		if (trusted_) {
			optical_ = headRelative;
			opticalNs_ = sampleTimestampNs;
		}
	}

	if (haveSample && !trusted_ && runCount_ >= params_.confirmSamples) {
		// Confirmed. While an earlier optical position is still fading out,
		// take over only if the new one is where that one could have moved;
		// otherwise (the other controller, a bad solve) finish fading to the
		// arm model first, so the output never jumps. Judged from where the
		// run started, so waiting does not make a far position acceptable.
		bool continuous = ramp_ <= 0.0f;
		if (!continuous) {
			const int64_t gap = std::max<int64_t>(runStartNs_ - opticalNs_, 0);
			const float dx = runStartPos_.x - optical_.x;
			const float dy = runStartPos_.y - optical_.y;
			const float dz = runStartPos_.z - optical_.z;
			const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
			continuous = dist <= params_.consistencyM + params_.maxSpeedMps * (float)gap / 1e9f;
		}
		if (continuous) {
			trusted_ = true;
			optical_ = lastSamplePos_;
			opticalNs_ = lastSampleNs_;
		}
	}

	const float step = (float)dt / (float)params_.blendNs;
	ramp_ = trusted_ ? std::min(1.0f, ramp_ + step) : std::max(0.0f, ramp_ - step);
}

float
ControllerPositionBlend::Weight() const
{
	return ramp_ * ramp_ * (3.0f - 2.0f * ramp_);
}

BlendVec3
ControllerPositionBlend::Mix(const BlendVec3 &arm, const BlendVec3 &optical, float weight)
{
	const float w = std::clamp(weight, 0.0f, 1.0f);
	return {arm.x + (optical.x - arm.x) * w, arm.y + (optical.y - arm.y) * w, arm.z + (optical.z - arm.z) * w};
}

} // namespace oxrsys
