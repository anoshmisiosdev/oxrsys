// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "AudioRingReader.h"
#include "OXAudioRing.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using oxrsys::AudioRingReader;
using oxrsys::LinearResampler;

namespace
{
constexpr uint64_t kNow = 1'000'000'000'000ull;

// Minimal writer mirroring the launcher's OXAudioRingWriter.
class TestRingWriter
{
public:
    TestRingWriter(const std::string& path, uint32_t rate, uint32_t channels, uint32_t capacity)
        : channels_(channels), capacity_(capacity)
    {
        bytes_ = OX_AUDIO_RING_HEADER_BYTES + static_cast<size_t>(capacity) * channels * 4;
        fd_ = open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        REQUIRE(fd_ >= 0);
        REQUIRE(ftruncate(fd_, static_cast<off_t>(bytes_)) == 0);
        map_ = mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        REQUIRE(map_ != MAP_FAILED);
        header_ = static_cast<OXAudioRingHeader*>(map_);
        samples_ = reinterpret_cast<float*>(static_cast<uint8_t*>(map_) +
                                            OX_AUDIO_RING_HEADER_BYTES);
        header_->magic = OX_AUDIO_RING_MAGIC;
        header_->version = OX_AUDIO_RING_VERSION;
        header_->sampleRateHz = rate;
        header_->channels = channels;
        header_->capacityFrames = capacity;
        header_->scope = 1;
        header_->generation.store(1);
        header_->heartbeatNs.store(kNow);
        header_->active.store(1);
    }
    ~TestRingWriter()
    {
        munmap(map_, bytes_);
        close(fd_);
    }

    // Writes frames whose sample value is (frame index + channel * 0.5).
    void Write(uint32_t frames)
    {
        uint64_t w = header_->writeFrame.load();
        for (uint32_t f = 0; f < frames; ++f)
        {
            const uint64_t index = w + f;
            for (uint32_t c = 0; c < channels_; ++c)
            {
                samples_[(index & (capacity_ - 1)) * channels_ + c] =
                    static_cast<float>(index) + 0.5f * static_cast<float>(c);
            }
        }
        header_->writeFrame.store(w + frames);
    }

    OXAudioRingHeader* header_ = nullptr;

private:
    int fd_ = -1;
    void* map_ = nullptr;
    size_t bytes_ = 0;
    float* samples_ = nullptr;
    uint32_t channels_;
    uint32_t capacity_;
};

std::string TempRingPath(const char* name)
{
    return std::string("/tmp/oxrsys_ring_test_") + name + "_" + std::to_string(getpid());
}
} // namespace

TEST_CASE("AudioRingReader reports NoRing for a missing file", "[audio_ring]")
{
    AudioRingReader reader("/tmp/oxrsys_ring_test_does_not_exist");
    std::vector<float> out;
    uint32_t rate = 0;
    uint16_t channels = 0;
    CHECK(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::NoRing);
    CHECK(out.empty());
}

TEST_CASE("AudioRingReader streams frames in order across the wrap point", "[audio_ring]")
{
    const std::string path = TempRingPath("wrap");
    TestRingWriter writer(path, 48000, 2, 1024);
    AudioRingReader reader(path);
    std::vector<float> out;
    uint32_t rate = 0;
    uint16_t channels = 0;

    // First poll syncs to the writer (nothing written yet).
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);
    CHECK(rate == 48000);
    CHECK(channels == 2);
    CHECK(out.empty());

    uint64_t expected = 0;
    for (int block = 0; block < 10; ++block) // 10 * 300 frames wraps a 1024 ring
    {
        writer.Write(300);
        REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);
        REQUIRE(out.size() == 600);
        for (size_t f = 0; f < 300; ++f)
        {
            CHECK(out[f * 2] == static_cast<float>(expected + f));
            CHECK(out[f * 2 + 1] == static_cast<float>(expected + f) + 0.5f);
        }
        expected += 300;
    }
    CHECK(reader.OverrunCount() == 0);
    CHECK(reader.SkipCount() == 0);
    std::remove(path.c_str());
}

TEST_CASE("AudioRingReader bounds backlog by skipping to recent audio", "[audio_ring]")
{
    const std::string path = TempRingPath("backlog");
    TestRingWriter writer(path, 48000, 2, 32768);
    AudioRingReader reader(path);
    std::vector<float> out;
    uint32_t rate = 0;
    uint16_t channels = 0;
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);

    writer.Write(48000 / 2); // 500 ms queued, far beyond the backlog limit
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);
    const size_t leadFrames = 48000 * AudioRingReader::kResyncLeadMs / 1000;
    CHECK(out.size() == leadFrames * 2);
    CHECK(out[0] == static_cast<float>(24000 - leadFrames));
    CHECK(reader.SkipCount() == 1);
    std::remove(path.c_str());
}

TEST_CASE("AudioRingReader goes inactive when the writer stops or stalls", "[audio_ring]")
{
    const std::string path = TempRingPath("stale");
    TestRingWriter writer(path, 48000, 2, 1024);
    AudioRingReader reader(path);
    std::vector<float> out;
    uint32_t rate = 0;
    uint16_t channels = 0;
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);

    // Heartbeat older than the stale limit.
    CHECK(reader.Poll(kNow + OX_AUDIO_RING_STALE_NS + 1, out, rate, channels) ==
          AudioRingReader::State::Inactive);

    writer.header_->active.store(0);
    CHECK(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Inactive);

    // Writer restarts: new generation, new rate, counter reset.
    writer.header_->sampleRateHz = 44100;
    writer.header_->writeFrame.store(0);
    writer.header_->generation.store(2);
    writer.header_->active.store(1);
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);
    CHECK(rate == 44100);
    writer.Write(100);
    REQUIRE(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::Live);
    CHECK(out.size() == 200);
    CHECK(out[0] == 0.0f);
    std::remove(path.c_str());
}

TEST_CASE("AudioRingReader rejects a ring with a bad header", "[audio_ring]")
{
    const std::string path = TempRingPath("bad");
    {
        TestRingWriter writer(path, 48000, 2, 1024);
        writer.header_->capacityFrames = 1000; // not a power of two
    }
    AudioRingReader reader(path);
    std::vector<float> out;
    uint32_t rate = 0;
    uint16_t channels = 0;
    CHECK(reader.Poll(kNow, out, rate, channels) == AudioRingReader::State::NoRing);
    std::remove(path.c_str());
}

TEST_CASE("LinearResampler passes equal rates through and converts 44.1k to 48k",
          "[audio_ring]")
{
    LinearResampler resampler;
    std::vector<float> in = {0.1f, 0.2f, 0.3f, 0.4f};
    std::vector<float> out;
    resampler.Process(in.data(), 2, 48000, 48000, 2, out);
    CHECK(out == in);

    // A 441 Hz sine at 44.1 kHz fed in uneven blocks must come out as a clean
    // 441 Hz sine at 48 kHz with the right length and no seams.
    resampler.Reset();
    out.clear();
    const size_t inFrames = 44100;
    std::vector<float> sine(inFrames);
    for (size_t i = 0; i < inFrames; ++i)
    {
        sine[i] = static_cast<float>(std::sin(2.0 * M_PI * 441.0 * i / 44100.0));
    }
    size_t offset = 0;
    const size_t blocks[] = {512, 441, 1000, 37};
    size_t blockIndex = 0;
    while (offset < inFrames)
    {
        const size_t n = std::min(blocks[blockIndex++ % 4], inFrames - offset);
        resampler.Process(sine.data() + offset, n, 44100, 48000, 1, out);
        offset += n;
    }
    CHECK(out.size() >= 47998);
    CHECK(out.size() <= 48001);
    double maxError = 0.0;
    for (size_t i = 0; i < out.size(); ++i)
    {
        const double expected = std::sin(2.0 * M_PI * 441.0 * i / 48000.0);
        maxError = std::max(maxError, std::fabs(out[i] - expected));
    }
    CHECK(maxError < 0.01);
}
