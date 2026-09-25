// SPDX-License-Identifier: MPL-2.0

#include "AudioRingReader.h"

#include "OXAudioRing.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace oxrsys
{

// ─── LinearResampler ────────────────────────────────────────────────────────

void LinearResampler::Reset()
{
    inRate_ = outRate_ = 0;
    channels_ = 0;
    position_ = 0.0;
    prev_.clear();
    havePrev_ = false;
}

void LinearResampler::Process(const float* in, size_t frames, uint32_t inRate, uint32_t outRate,
                              uint16_t channels, std::vector<float>& out)
{
    if (in == nullptr || frames == 0 || channels == 0 || inRate == 0 || outRate == 0)
    {
        return;
    }
    if (inRate == outRate)
    {
        Reset();
        out.insert(out.end(), in, in + frames * channels);
        return;
    }
    if (inRate != inRate_ || outRate != outRate_ || channels != channels_)
    {
        Reset();
        inRate_ = inRate;
        outRate_ = outRate;
        channels_ = channels;
        prev_.assign(channels, 0.0f);
    }

    // Virtual input: index -1 is prev_ (last frame of the previous block, when
    // available), indices 0..frames-1 are this block. position_ is the next
    // output position in that virtual index space, starting at -1 when we have
    // history so the block boundary is interpolated seamlessly.
    const double step = static_cast<double>(inRate) / static_cast<double>(outRate);
    double pos = havePrev_ ? position_ - 1.0 : 0.0;
    auto sample = [&](long idx, uint16_t c) -> float {
        return idx < 0 ? prev_[c] : in[static_cast<size_t>(idx) * channels + c];
    };
    const double last = static_cast<double>(frames - 1);
    while (pos <= last)
    {
        const long i0 = static_cast<long>(std::floor(pos));
        const double frac = pos - static_cast<double>(i0);
        const long i1 = std::min<long>(i0 + 1, static_cast<long>(frames - 1));
        for (uint16_t c = 0; c < channels; ++c)
        {
            const float a = sample(i0, c);
            const float b = sample(i1, c);
            out.push_back(a + static_cast<float>(frac) * (b - a));
        }
        pos += step;
    }
    // Carry the fractional position into the next block (relative to its -1).
    position_ = pos - last;
    for (uint16_t c = 0; c < channels; ++c)
    {
        prev_[c] = in[(frames - 1) * channels + c];
    }
    havePrev_ = true;
}

// ─── AudioRingReader ────────────────────────────────────────────────────────

AudioRingReader::AudioRingReader(std::string path) : path_(std::move(path)) {}

AudioRingReader::~AudioRingReader() { Unmap(); }

std::string AudioRingReader::DefaultPath()
{
    const char* home = std::getenv("HOME");
    if (home == nullptr || *home == '\0')
    {
        return {};
    }
    return std::string(home) + "/" + OX_AUDIO_RING_RELATIVE_PATH;
}

void AudioRingReader::Unmap()
{
    if (map_ != nullptr)
    {
        munmap(map_, mapBytes_);
    }
    if (fd_ >= 0)
    {
        close(fd_);
    }
    fd_ = -1;
    map_ = nullptr;
    mapBytes_ = 0;
    header_ = nullptr;
    samples_ = nullptr;
    capacityFrames_ = 0;
    synced_ = false;
}

bool AudioRingReader::EnsureMapped()
{
    if (header_ != nullptr)
    {
        return true;
    }
    if (path_.empty())
    {
        return false;
    }
    fd_ = open(path_.c_str(), O_RDONLY);
    if (fd_ < 0)
    {
        return false;
    }
    struct stat st = {};
    if (fstat(fd_, &st) != 0 || st.st_size < static_cast<off_t>(OX_AUDIO_RING_HEADER_BYTES))
    {
        Unmap();
        return false;
    }
    mapBytes_ = static_cast<size_t>(st.st_size);
    void* map = mmap(nullptr, mapBytes_, PROT_READ, MAP_SHARED, fd_, 0);
    if (map == MAP_FAILED)
    {
        map_ = nullptr;
        Unmap();
        return false;
    }
    map_ = map;
    auto* header = static_cast<OXAudioRingHeader*>(map_);
    const uint32_t capacity = header->capacityFrames;
    const uint32_t channels = header->channels;
    const bool powerOfTwo = capacity != 0 && (capacity & (capacity - 1)) == 0;
    if (header->magic != OX_AUDIO_RING_MAGIC || header->version != OX_AUDIO_RING_VERSION ||
        !powerOfTwo || channels == 0 || channels > 8 ||
        OX_AUDIO_RING_HEADER_BYTES + static_cast<size_t>(capacity) * channels * sizeof(float) >
            mapBytes_)
    {
        Unmap();
        return false;
    }
    header_ = header;
    samples_ = reinterpret_cast<const float*>(static_cast<const uint8_t*>(map_) +
                                              OX_AUDIO_RING_HEADER_BYTES);
    capacityFrames_ = capacity;
    channels_ = channels;
    synced_ = false;
    return true;
}

AudioRingReader::State AudioRingReader::Poll(uint64_t nowRealtimeNs, std::vector<float>& out,
                                             uint32_t& sampleRateHz, uint16_t& channels)
{
    out.clear();
    if (!EnsureMapped())
    {
        return State::NoRing;
    }

    const uint32_t active = header_->active.load(std::memory_order_acquire);
    const uint64_t heartbeat = header_->heartbeatNs.load(std::memory_order_relaxed);
    const bool stale = nowRealtimeNs > heartbeat &&
                       nowRealtimeNs - heartbeat > OX_AUDIO_RING_STALE_NS;
    if (active == 0 || stale)
    {
        synced_ = false;
        // A restarted writer may have re-created the file; remap next time.
        if (active == 0)
        {
            Unmap();
        }
        return State::Inactive;
    }

    // Format and geometry can change only across a generation bump, and the
    // writer clears `active` while it rewrites them.
    const uint32_t generation = header_->generation.load(std::memory_order_acquire);
    if (!synced_ || generation != generation_ || header_->capacityFrames != capacityFrames_ ||
        header_->channels != channels_)
    {
        if (header_->capacityFrames != capacityFrames_ || header_->channels != channels_)
        {
            Unmap();
            return State::Inactive;
        }
        generation_ = generation;
        sampleRateHz_ = header_->sampleRateHz;
        scope_ = header_->scope;
        const uint64_t write = header_->writeFrame.load(std::memory_order_acquire);
        const uint64_t lead = static_cast<uint64_t>(sampleRateHz_) * kResyncLeadMs / 1000;
        readFrame_ = write > lead ? write - lead : 0;
        synced_ = true;
    }

    sampleRateHz = sampleRateHz_;
    channels = static_cast<uint16_t>(channels_);

    uint64_t write = header_->writeFrame.load(std::memory_order_acquire);
    if (write < readFrame_)
    {
        // Writer restarted its counter without a generation bump; resync.
        readFrame_ = write;
        return State::Live;
    }
    const uint64_t maxBacklog =
        std::min<uint64_t>(static_cast<uint64_t>(sampleRateHz_) * kMaxBacklogMs / 1000,
                           capacityFrames_ / 2);
    if (write - readFrame_ > maxBacklog)
    {
        const uint64_t lead = static_cast<uint64_t>(sampleRateHz_) * kResyncLeadMs / 1000;
        readFrame_ = write - std::min(lead, write);
        ++skips_;
    }
    const uint64_t available = write - readFrame_;
    if (available == 0)
    {
        return State::Live;
    }

    out.resize(static_cast<size_t>(available) * channels_);
    const uint32_t mask = capacityFrames_ - 1;
    const uint32_t start = static_cast<uint32_t>(readFrame_ & mask);
    const uint32_t first = static_cast<uint32_t>(
        std::min<uint64_t>(available, static_cast<uint64_t>(capacityFrames_ - start)));
    std::memcpy(out.data(), samples_ + static_cast<size_t>(start) * channels_,
                static_cast<size_t>(first) * channels_ * sizeof(float));
    if (first < available)
    {
        std::memcpy(out.data() + static_cast<size_t>(first) * channels_, samples_,
                    static_cast<size_t>(available - first) * channels_ * sizeof(float));
    }

    // If the writer lapped us while copying, part of what we copied is newer
    // audio from the next lap: discard and resync rather than emit garbage.
    const uint64_t writeAfter = header_->writeFrame.load(std::memory_order_acquire);
    if (writeAfter - readFrame_ > capacityFrames_)
    {
        out.clear();
        ++overruns_;
        synced_ = false;
        return State::Live;
    }
    readFrame_ = write;
    return State::Live;
}

} // namespace oxrsys
