#pragma once

// AudioCapture — captures the game's audio on macOS and delivers interleaved
// stereo float32 PCM to a callback.
//
// The OXRSys runtime is dlopen'd in-process by the CrossOver/Wine host, and the
// game's audio (WASAPI/XAudio2/DirectSound → winecoreaudio) is produced by that
// same process. So a Core Audio *process tap* on our own PID (macOS 14.4+)
// captures exactly the game audio and nothing else — no virtual device, no
// system-wide tap, no TCC prompt for other processes.
//
// This header is plain C++ so it can be included from StreamingServer.cpp; the
// implementation (AudioCapture.mm) is Objective-C++ using Core Audio.

#include <cstdint>
#include <functional>
#include <memory>

namespace oxrsys
{

class AudioCapture
{
public:
    // frames = number of sample frames; data = interleaved stereo float32
    // (frames * channels floats). sampleRateHz/channels describe the buffer.
    using SampleCallback = std::function<void(const float* data,
                                              uint32_t frames,
                                              uint32_t sampleRateHz,
                                              uint16_t channels)>;

    AudioCapture();
    ~AudioCapture();

    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;

    // Start tapping this process's audio output. Returns false if process taps
    // are unavailable (pre-14.4) or setup failed; logs the reason.
    bool Start(SampleCallback callback);
    void Stop();

    bool IsRunning() const;

    // Public so the Core Audio render callback (a C function) can name the type.
    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

} // namespace oxrsys
