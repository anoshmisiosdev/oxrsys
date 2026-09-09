#pragma once

// AudioPlayer — plays the streamed game audio on the headset via AAudio.
//
// The server (macOS) taps the game's output and sends interleaved float32 PCM as
// TcpRecordType::Audio records. This class buffers those samples in a ring and
// feeds them to a low-latency AAudio output stream via its data callback, so the
// network receive thread never blocks on playback.

#include <aaudio/AAudio.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

class AudioPlayer
{
public:
    AudioPlayer() = default;
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;

    // Feed interleaved float samples. Lazily opens the stream on first call using
    // the reported rate/channels. sampleCount is the number of float samples
    // (frames * channels), matching the server's payloadSize / sizeof(float).
    void Write(const float* samples, uint32_t sampleCount,
               uint32_t sampleRateHz, uint16_t channels);

    void Stop();

private:
    bool EnsureStream(uint32_t sampleRateHz, uint16_t channels);
    static aaudio_data_callback_result_t DataCallback(AAudioStream* stream,
                                                      void* userData,
                                                      void* audioData,
                                                      int32_t numFrames);

    AAudioStream* stream_ = nullptr;
    uint32_t sampleRateHz_ = 0;
    uint16_t channels_ = 0;

    std::mutex ringMutex_;
    std::vector<float> ring_;   // fixed-capacity circular buffer of floats
    size_t head_ = 0;           // read index
    size_t count_ = 0;          // valid samples
    std::atomic<bool> started_{false};
};
