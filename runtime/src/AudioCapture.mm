#include "AudioCapture.h"

#import <CoreAudio/CoreAudio.h>
#import <CoreAudio/CATapDescription.h>
#import <CoreAudio/AudioHardwareTapping.h>
#import <Foundation/Foundation.h>

#include <spdlog/spdlog.h>
#include <atomic>
#include <unistd.h>

namespace oxrsys
{

struct AudioCapture::Impl
{
    AudioObjectID tapId = kAudioObjectUnknown;
    AudioObjectID aggregateId = kAudioObjectUnknown;
    AudioDeviceIOProcID ioProcId = nullptr;
    SampleCallback callback;
    uint32_t sampleRateHz = 48000;
    uint16_t channels = 2;
    std::atomic<bool> running{false};
};

namespace
{
// Translate our own PID to a Core Audio process object.
AudioObjectID ProcessObjectForPid(pid_t pid)
{
    AudioObjectID object = kAudioObjectUnknown;
    AudioObjectPropertyAddress address = {
        kAudioHardwarePropertyTranslatePIDToProcessObject,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain};
    UInt32 size = sizeof(object);
    OSStatus status = AudioObjectGetPropertyData(
        kAudioObjectSystemObject, &address, sizeof(pid), &pid, &size, &object);
    if (status != noErr)
    {
        return kAudioObjectUnknown;
    }
    return object;
}

double NominalSampleRate(AudioObjectID device)
{
    Float64 rate = 48000.0;
    AudioObjectPropertyAddress address = {
        kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain};
    UInt32 size = sizeof(rate);
    AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, &rate);
    return rate;
}
} // namespace

AudioCapture::AudioCapture() : impl_(std::make_unique<Impl>()) {}

AudioCapture::~AudioCapture() { Stop(); }

bool AudioCapture::IsRunning() const { return impl_ && impl_->running.load(); }

bool AudioCapture::Start(SampleCallback callback)
{
    if (impl_->running.load())
    {
        return true;
    }
    if (@available(macOS 14.4, *))
    {
        // fall through to real implementation
    }
    else
    {
        spdlog::warn("AudioCapture: Core Audio process taps require macOS 14.4+; "
                     "headset audio disabled");
        return false;
    }

    impl_->callback = std::move(callback);

    @autoreleasepool
    {
        pid_t pid = getpid();
        AudioObjectID processObject = ProcessObjectForPid(pid);
        if (processObject == kAudioObjectUnknown)
        {
            spdlog::error("AudioCapture: could not resolve process object for pid {}", pid);
            return false;
        }

        // Stereo mixdown of just this process's audio output.
        CATapDescription* description =
            [[CATapDescription alloc] initStereoMixdownOfProcesses:@[ @(processObject) ]];
        description.name = @"OXRSys Game Audio";
        description.privateTap = YES;                 // not visible system-wide
        description.muteBehavior = CATapUnmuted;      // don't alter local playback

        OSStatus status = AudioHardwareCreateProcessTap(description, &impl_->tapId);
        if (status != noErr || impl_->tapId == kAudioObjectUnknown)
        {
            spdlog::error("AudioCapture: AudioHardwareCreateProcessTap failed ({})", status);
            return false;
        }

        NSString* tapUID = description.UUID.UUIDString;
        NSString* aggUID =
            [NSString stringWithFormat:@"org.oxrsys.audiocap.%d", pid];
        NSDictionary* aggDescription = @{
            @(kAudioAggregateDeviceNameKey): @"OXRSys Audio Capture",
            @(kAudioAggregateDeviceUIDKey): aggUID,
            @(kAudioAggregateDeviceIsPrivateKey): @YES,
            @(kAudioAggregateDeviceIsStackedKey): @NO,
            @(kAudioAggregateDeviceTapAutoStartKey): @YES,
            @(kAudioAggregateDeviceTapListKey): @[ @{
                @(kAudioSubTapUIDKey): tapUID,
                @(kAudioSubTapDriftCompensationKey): @YES,
            } ],
        };

        status = AudioHardwareCreateAggregateDevice(
            (__bridge CFDictionaryRef)aggDescription, &impl_->aggregateId);
        if (status != noErr || impl_->aggregateId == kAudioObjectUnknown)
        {
            spdlog::error("AudioCapture: AudioHardwareCreateAggregateDevice failed ({})", status);
            AudioHardwareDestroyProcessTap(impl_->tapId);
            impl_->tapId = kAudioObjectUnknown;
            return false;
        }

        impl_->sampleRateHz = static_cast<uint32_t>(NominalSampleRate(impl_->aggregateId) + 0.5);
        if (impl_->sampleRateHz == 0)
        {
            impl_->sampleRateHz = 48000;
        }
        impl_->channels = 2;

        Impl* impl = impl_.get();
        status = AudioDeviceCreateIOProcIDWithBlock(
            &impl_->ioProcId, impl_->aggregateId, nullptr,
            ^(const AudioTimeStamp*, const AudioBufferList* inInputData,
              const AudioTimeStamp*, AudioBufferList*, const AudioTimeStamp*) {
                if (!impl->running.load() || inInputData == nullptr ||
                    inInputData->mNumberBuffers == 0)
                {
                    return;
                }
                const AudioBuffer& buffer = inInputData->mBuffers[0];
                if (buffer.mData == nullptr || buffer.mNumberChannels == 0)
                {
                    return;
                }
                uint16_t channels = static_cast<uint16_t>(buffer.mNumberChannels);
                uint32_t frames = buffer.mDataByteSize /
                                  (sizeof(float) * channels);
                if (frames == 0)
                {
                    return;
                }
                impl->callback(static_cast<const float*>(buffer.mData), frames,
                               impl->sampleRateHz, channels);
            });
        if (status != noErr || impl_->ioProcId == nullptr)
        {
            spdlog::error("AudioCapture: AudioDeviceCreateIOProcIDWithBlock failed ({})", status);
            Stop();
            return false;
        }

        impl_->running.store(true);
        status = AudioDeviceStart(impl_->aggregateId, impl_->ioProcId);
        if (status != noErr)
        {
            spdlog::error("AudioCapture: AudioDeviceStart failed ({})", status);
            impl_->running.store(false);
            Stop();
            return false;
        }
    }

    spdlog::info("AudioCapture: capturing game audio ({} Hz, {} ch) via process tap",
                 impl_->sampleRateHz, impl_->channels);
    return true;
}

void AudioCapture::Stop()
{
    if (!impl_)
    {
        return;
    }
    bool was = impl_->running.exchange(false);
    if (impl_->aggregateId != kAudioObjectUnknown && impl_->ioProcId != nullptr)
    {
        AudioDeviceStop(impl_->aggregateId, impl_->ioProcId);
        AudioDeviceDestroyIOProcID(impl_->aggregateId, impl_->ioProcId);
        impl_->ioProcId = nullptr;
    }
    if (impl_->aggregateId != kAudioObjectUnknown)
    {
        AudioHardwareDestroyAggregateDevice(impl_->aggregateId);
        impl_->aggregateId = kAudioObjectUnknown;
    }
    if (impl_->tapId != kAudioObjectUnknown)
    {
        AudioHardwareDestroyProcessTap(impl_->tapId);
        impl_->tapId = kAudioObjectUnknown;
    }
    impl_->callback = nullptr;
    if (was)
    {
        spdlog::info("AudioCapture: stopped");
    }
}

} // namespace oxrsys
