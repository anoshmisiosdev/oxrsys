#include "AudioPlayer.h"

#include <android/log.h>
#include <algorithm>
#include <cstring>

#define LOG_TAG "OXRSysAudio"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

AudioPlayer::~AudioPlayer() { Stop(); }

aaudio_data_callback_result_t AudioPlayer::DataCallback(AAudioStream* /*stream*/,
                                                        void* userData,
                                                        void* audioData,
                                                        int32_t numFrames)
{
    auto* self = static_cast<AudioPlayer*>(userData);
    auto* out = static_cast<float*>(audioData);
    const size_t wanted =
        static_cast<size_t>(numFrames) * std::max<uint16_t>(self->channels_, 1);

    std::lock_guard<std::mutex> lock(self->ringMutex_);
    const size_t avail = std::min(wanted, self->count_);
    const size_t cap = self->ring_.size();
    for (size_t i = 0; i < avail; ++i)
    {
        out[i] = self->ring_[(self->head_ + i) % cap];
    }
    // Underrun: fill the rest with silence so the stream keeps flowing.
    for (size_t i = avail; i < wanted; ++i)
    {
        out[i] = 0.0f;
    }
    self->head_ = cap ? (self->head_ + avail) % cap : 0;
    self->count_ -= avail;
    return AAUDIO_CALLBACK_RESULT_CONTINUE;
}

bool AudioPlayer::EnsureStream(uint32_t sampleRateHz, uint16_t channels)
{
    if (stream_ != nullptr)
    {
        return true;
    }
    if (sampleRateHz == 0 || channels == 0)
    {
        return false;
    }

    AAudioStreamBuilder* builder = nullptr;
    if (AAudio_createStreamBuilder(&builder) != AAUDIO_OK || builder == nullptr)
    {
        LOGW("AAudio_createStreamBuilder failed");
        return false;
    }

    AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
    AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_FLOAT);
    AAudioStreamBuilder_setSampleRate(builder, static_cast<int32_t>(sampleRateHz));
    AAudioStreamBuilder_setChannelCount(builder, channels);
    AAudioStreamBuilder_setPerformanceMode(builder, AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
    AAudioStreamBuilder_setSharingMode(builder, AAUDIO_SHARING_MODE_SHARED);
    AAudioStreamBuilder_setUsage(builder, AAUDIO_USAGE_GAME);
    AAudioStreamBuilder_setDataCallback(builder, &AudioPlayer::DataCallback, this);

    aaudio_result_t result = AAudioStreamBuilder_openStream(builder, &stream_);
    AAudioStreamBuilder_delete(builder);
    if (result != AAUDIO_OK || stream_ == nullptr)
    {
        LOGW("AAudio openStream failed: %s", AAudio_convertResultToText(result));
        stream_ = nullptr;
        return false;
    }

    sampleRateHz_ = sampleRateHz;
    channels_ = channels;
    // ~0.5 s of headroom so brief network stalls don't underrun.
    const size_t capacity = static_cast<size_t>(sampleRateHz) * channels / 2;
    {
        std::lock_guard<std::mutex> lock(ringMutex_);
        ring_.assign(capacity, 0.0f);
        head_ = 0;
        count_ = 0;
    }

    result = AAudioStream_requestStart(stream_);
    if (result != AAUDIO_OK)
    {
        LOGW("AAudio requestStart failed: %s", AAudio_convertResultToText(result));
        AAudioStream_close(stream_);
        stream_ = nullptr;
        return false;
    }

    started_.store(true);
    LOGI("Headset audio playback started (%u Hz, %u ch)", sampleRateHz, channels);
    return true;
}

void AudioPlayer::Write(const float* samples, uint32_t sampleCount,
                        uint32_t sampleRateHz, uint16_t channels)
{
    if (samples == nullptr || sampleCount == 0)
    {
        return;
    }
    if (!EnsureStream(sampleRateHz, channels))
    {
        return;
    }
    // A mid-stream format change would need a new stream; log once and ignore.
    if (sampleRateHz != sampleRateHz_ || channels != channels_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(ringMutex_);
    const size_t cap = ring_.size();
    if (cap == 0)
    {
        return;
    }
    for (uint32_t i = 0; i < sampleCount; ++i)
    {
        const size_t writeIndex = (head_ + count_) % cap;
        ring_[writeIndex] = samples[i];
        if (count_ < cap)
        {
            ++count_;
        }
        else
        {
            // Full: overwrite oldest by advancing the read head.
            head_ = (head_ + 1) % cap;
        }
    }
}

void AudioPlayer::Stop()
{
    if (stream_ != nullptr)
    {
        AAudioStream_requestStop(stream_);
        AAudioStream_close(stream_);
        stream_ = nullptr;
    }
    started_.store(false);
    std::lock_guard<std::mutex> lock(ringMutex_);
    ring_.clear();
    head_ = 0;
    count_ = 0;
}
