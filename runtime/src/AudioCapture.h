#pragma once

// AudioCapture — delivers the game's audio on macOS as interleaved float32 PCM
// to a callback. Two sources:
//
//  * Tap ring (preferred): the OXRSys launcher app captures the game process
//    (or all Mac audio) with a Core Audio process tap and publishes it through a
//    shared-memory ring (OXAudioRing.h), read here by AudioRingReader. The tap
//    must live in a separate app: CrossOver cannot hold the "System Audio
//    Recording" permission, so a tap created in this process yields silence.
//  * Loopback input device (fallback): BlackHole or similar, with the bottle's
//    output routed to it.
//
// In `auto` mode both run and the ring wins whenever its writer is live.
//
// This header is plain C++ so it can be included from StreamingServer.cpp; the
// implementation (AudioCapture.mm) is Objective-C++ using Core Audio.

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace oxrsys
{

class AudioCapture
{
public:
    // frames = number of sample frames; data = interleaved stereo float32
    // (frames * channels floats). sampleRateHz/channels describe the buffer.
    // source is a string literal naming the delivering source ("tap" or
    // "loopback"), safe to store.
    using SampleCallback = std::function<void(const float* data,
                                              uint32_t frames,
                                              uint32_t sampleRateHz,
                                              uint16_t channels,
                                              const char* source)>;

    enum class Source
    {
        Auto,     // tap ring when live, otherwise loopback device
        Tap,      // tap ring only
        Loopback, // loopback input device only (legacy)
    };

    // Parses the `headset_audio_source` config value; unknown values -> Auto.
    static Source ParseSource(const std::string& value);

    // Sample rate delivered to the callback for the tap ring (the stream rate).
    static constexpr uint32_t kStreamSampleRateHz = 48000;

    AudioCapture();
    ~AudioCapture();

    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;

    // Start capturing. Returns false if no source could be started; logs why.
    // The callback runs on a Core Audio IO thread (loopback) or on the ring
    // reader thread (tap); around a source switch both may briefly call it, so
    // it must be thread-safe.
    bool Start(SampleCallback callback, Source source = Source::Auto);
    void Stop();

    bool IsRunning() const;

    // "tap", "loopback" or "none": the source currently delivering audio.
    const char* ActiveSourceName() const;

    // Public so the Core Audio render callback (a C function) can name the type.
    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

} // namespace oxrsys
