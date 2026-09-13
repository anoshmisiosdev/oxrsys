// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "ControllerPositionBlend.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using oxrsys::BlendVec3;
using oxrsys::ControllerPositionBlend;

namespace
{

constexpr int64_t kMs = 1000000LL;
//! The helper's tracking loop runs at 250 Hz.
constexpr int64_t kTickNs = 4 * kMs;
//! Controller frames arrive at 60 Hz.
constexpr int64_t kSampleNs = 16667000LL;
//! How long the tracker keeps reporting a sample.
constexpr int64_t kStaleNs = 150 * kMs;

const BlendVec3 kArm = {0.18f, -0.35f, -0.40f};
const BlendVec3 kOptical = {-0.10f, -0.20f, -0.30f};

float
Distance(const BlendVec3& a, const BlendVec3& b)
{
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/*!
 * Drives a blend the way the helper does: a controller whose optical samples
 * come at 60 Hz while `visible` says so, reported until they are stale.
 */
struct Sim
{
    ControllerPositionBlend blend;
    int64_t now = 1000 * kMs;
    int64_t lastSampleNs = 0;
    BlendVec3 lastSamplePos;
    BlendVec3 output = kArm;
    float maxStep = 0.0f;

    BlendVec3 Output() const
    {
        return blend.Weight() > 0.0f ? ControllerPositionBlend::Mix(kArm, blend.OpticalHeadRelative(), blend.Weight())
                                     : kArm;
    }

    //! Advance one tick; a new sample at `pos` is produced when `visible` and one is due.
    void Tick(bool visible, BlendVec3 pos = kOptical)
    {
        now += kTickNs;
        if (visible && (lastSampleNs == 0 || now - lastSampleNs >= kSampleNs)) {
            lastSampleNs = now;
            lastSamplePos = pos;
        }
        const bool have = lastSampleNs != 0 && now - lastSampleNs < kStaleNs;
        blend.Update(now, have, lastSampleNs, lastSamplePos);
        const BlendVec3 out = Output();
        maxStep = std::max(maxStep, Distance(out, output));
        output = out;
    }

    void Run(int64_t durationNs, bool visible, BlendVec3 pos = kOptical)
    {
        for (int64_t t = 0; t < durationNs; t += kTickNs) {
            Tick(visible, pos);
        }
    }
};

//! Largest per-tick move a 100 ms smoothstep ramp between kArm and kOptical allows.
float
BlendStepBound()
{
    return 1.5f * (float)kTickNs / (float)(100 * kMs) * Distance(kArm, kOptical) + 1e-4f;
}

} // namespace

TEST_CASE("Controller blend needs several consistent samples before trusting", "[controller_blend]")
{
    ControllerPositionBlend blend;
    int64_t now = 1000 * kMs;
    blend.Update(now, true, now, kOptical);
    REQUIRE_FALSE(blend.Trusted());
    for (int i = 0; i < 20; i++) {
        blend.Update(now + (i + 1) * kTickNs, true, now, kOptical); // same sample repeated
    }
    REQUIRE_FALSE(blend.Trusted());
    REQUIRE(blend.Weight() == 0.0f);

    now += kSampleNs;
    blend.Update(now, true, now, kOptical);
    REQUIRE_FALSE(blend.Trusted());
    now += kSampleNs;
    blend.Update(now, true, now, kOptical);
    REQUIRE(blend.Trusted());
}

TEST_CASE("Controller blend ignores samples that jump around", "[controller_blend]")
{
    ControllerPositionBlend blend;
    int64_t now = 1000 * kMs;
    for (int i = 0; i < 12; i++) {
        now += kSampleNs;
        // Alternating left and right controller: 40 cm apart every frame.
        BlendVec3 p = kOptical;
        p.x += (i % 2 == 0) ? 0.20f : -0.20f;
        blend.Update(now, true, now, p);
        REQUIRE_FALSE(blend.Trusted());
    }
    REQUIRE(blend.Weight() == 0.0f);

    // Samples too far apart in time do not confirm either.
    ControllerPositionBlend sparse;
    for (int i = 0; i < 6; i++) {
        now += 120 * kMs;
        sparse.Update(now, true, now, kOptical);
        REQUIRE_FALSE(sparse.Trusted());
    }
}

TEST_CASE("Controller blend ramps continuously into and out of optical", "[controller_blend]")
{
    Sim sim;
    sim.Run(200 * kMs, false);
    REQUIRE(sim.blend.Weight() == 0.0f);

    sim.Run(300 * kMs, true);
    REQUIRE(sim.blend.Trusted());
    REQUIRE(sim.blend.Weight() == 1.0f);
    REQUIRE(Distance(sim.output, kOptical) < 1e-5f);

    sim.Run(400 * kMs, false);
    REQUIRE_FALSE(sim.blend.Trusted());
    REQUIRE(sim.blend.Weight() == 0.0f);
    REQUIRE(Distance(sim.output, kArm) < 1e-5f);

    REQUIRE(sim.maxStep <= BlendStepBound());
    REQUIRE(sim.maxStep > 0.0f);
}

TEST_CASE("Controller blend rides out a dropout shorter than the stale window", "[controller_blend]")
{
    Sim sim;
    sim.Run(300 * kMs, true);
    REQUIRE(sim.blend.Weight() == 1.0f);

    // 100 ms without new samples: the tracker still reports the last one.
    sim.maxStep = 0.0f;
    sim.Run(100 * kMs, false);
    REQUIRE(sim.blend.Trusted());
    REQUIRE(sim.blend.Weight() == 1.0f);

    sim.Run(100 * kMs, true);
    REQUIRE(sim.blend.Weight() == 1.0f);
    REQUIRE(sim.maxStep < 1e-5f);
}

TEST_CASE("Controller blend reverses mid-ramp without jumping", "[controller_blend]")
{
    Sim sim;
    sim.Run(300 * kMs, true);
    REQUIRE(sim.blend.Weight() == 1.0f);

    // Lose it long enough to go stale and fade part of the way out...
    sim.Run(190 * kMs, false);
    const float w = sim.blend.Weight();
    REQUIRE(w > 0.0f);
    REQUIRE(w < 1.0f);

    // ...then see it again, a little elsewhere: it confirms and ramps back.
    BlendVec3 moved = kOptical;
    moved.x += 0.03f;
    sim.Run(300 * kMs, true, moved);
    REQUIRE(sim.blend.Weight() == 1.0f);
    REQUIRE(Distance(sim.output, moved) < 1e-5f);

    // The only discontinuity allowed is the optical target moving (3 cm) while
    // the weight is below one; bound it by that plus the ramp step.
    REQUIRE(sim.maxStep <= BlendStepBound() + 0.03f);
}

TEST_CASE("Controller blend fades out on an implausible jump", "[controller_blend]")
{
    Sim sim;
    sim.Run(300 * kMs, true);
    REQUIRE(sim.blend.Weight() == 1.0f);
    const float stepBefore = sim.maxStep;

    // The other controller's pose: 40 cm away in one frame.
    BlendVec3 other = kOptical;
    other.x += 0.40f;
    sim.Run(kSampleNs + kTickNs, true, other); // at least one new sample, fewer than three
    REQUIRE(Distance(sim.lastSamplePos, other) < 1e-6f);
    REQUIRE_FALSE(sim.blend.Trusted());

    // It stays there: it confirms, but only takes over after the old position
    // has faded out, so the output moves through the arm model without jumps.
    sim.Run(400 * kMs, true, other);
    REQUIRE(sim.blend.Trusted());
    REQUIRE(sim.blend.Weight() == 1.0f);
    REQUIRE(Distance(sim.output, other) < 1e-5f);
    const float otherBound = 1.5f * (float)kTickNs / (float)(100 * kMs) *
                                 std::max(Distance(kArm, kOptical), Distance(kArm, other)) +
                             1e-4f;
    REQUIRE(sim.maxStep <= std::max(stepBefore, otherBound));
}
