// SPDX-License-Identifier: MPL-2.0

#pragma once

// AudioRingReader — reader side of the OXAudioRing shared-memory PCM ring that
// the OXRSys launcher fills from a Core Audio process tap (see OXAudioRing.h).
//
// The launcher runs as its own app, so macOS attributes the "System Audio
// Recording" permission to it rather than to CrossOver (which cannot hold that
// permission). The runtime only maps the file and copies samples out, so it
// never blocks on the writer and never touches Core Audio for this path.
//
// Pure C++ with no platform audio dependencies so it can be unit-tested.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct OXAudioRingHeader;

namespace oxrsys
{

// Stateful linear-interpolation resampler for interleaved float PCM. Used only
// when the tap rate differs from the stream rate (normally both are 48 kHz,
// in which case Process() is a copy).
class LinearResampler
{
public:
    void Reset();
    // Appends resampled frames to out. channels must stay constant between
    // calls; a change of rates or channels resets the filter state.
    void Process(const float* in, size_t frames, uint32_t inRate, uint32_t outRate,
                 uint16_t channels, std::vector<float>& out);

private:
    uint32_t inRate_ = 0;
    uint32_t outRate_ = 0;
    uint16_t channels_ = 0;
    double position_ = 0.0;       // fractional read position relative to prev_
    std::vector<float> prev_;     // last input frame of the previous block
    bool havePrev_ = false;
};

class AudioRingReader
{
public:
    // Upper bound on queued-but-unread audio. Past this the reader skips ahead
    // so the headset hears current audio instead of drifting behind.
    static constexpr uint32_t kMaxBacklogMs = 80;
    // After a (re)sync start this far behind the writer.
    static constexpr uint32_t kResyncLeadMs = 10;

    enum class State
    {
        NoRing,   // file missing, wrong magic/version or unmappable
        Inactive, // mapped but the writer is stopped or its heartbeat is stale
        Live,     // writer is active
    };

    explicit AudioRingReader(std::string path);
    ~AudioRingReader();

    AudioRingReader(const AudioRingReader&) = delete;
    AudioRingReader& operator=(const AudioRingReader&) = delete;

    // Default ring path under $HOME (empty if HOME is unset).
    static std::string DefaultPath();

    // Copies any newly written frames (interleaved float32) into out, replacing
    // its contents, and reports the ring's format. nowRealtimeNs is the caller's
    // CLOCK_REALTIME in ns (injectable for tests). Returns the ring state.
    State Poll(uint64_t nowRealtimeNs, std::vector<float>& out, uint32_t& sampleRateHz,
               uint16_t& channels);

    // Diagnostics.
    uint64_t OverrunCount() const { return overruns_; }
    uint64_t SkipCount() const { return skips_; }
    uint32_t Scope() const { return scope_; }

private:
    bool EnsureMapped();
    void Unmap();

    std::string path_;
    int fd_ = -1;
    void* map_ = nullptr;
    size_t mapBytes_ = 0;
    OXAudioRingHeader* header_ = nullptr;
    const float* samples_ = nullptr;
    uint32_t capacityFrames_ = 0;
    uint32_t channels_ = 0;
    uint32_t sampleRateHz_ = 0;
    uint32_t scope_ = 0;
    uint32_t generation_ = 0;
    uint64_t readFrame_ = 0;
    bool synced_ = false;
    uint64_t overruns_ = 0;
    uint64_t skips_ = 0;
};

} // namespace oxrsys
