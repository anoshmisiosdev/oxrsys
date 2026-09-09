#include "AudioCapture.h"

#import <CoreAudio/CoreAudio.h>
#import <AudioUnit/AudioUnit.h>
#import <Foundation/Foundation.h>

#include <spdlog/spdlog.h>
#include <atomic>
#include <string>
#include <vector>

// AudioCapture — capture the game's audio on macOS and deliver interleaved
// stereo float32 PCM to a callback.
//
// Core Audio process/system taps do NOT work under CrossOver: capturing audio
// outside our own process needs the "System Audio Recording" TCC permission, and
// CrossOver's Info.plist has no NSAudioCaptureUsageDescription, so a tap only
// ever yields silence. Instead we capture from a loopback INPUT device
// (BlackHole): the user routes the game's output to it, and we read it back as an
// input device — which uses CrossOver's Microphone permission (it HAS that).
//
// Route the bottle's audio output to BlackHole (or a Multi-Output that includes
// it), and this reads the same samples the game produced.

namespace oxrsys
{

namespace
{
// Input devices we recognize as loopbacks, in preference order. The game's audio
// output is routed to the matching *output* device (e.g. "PICO Virtual Speaker")
// and we read it back from the paired *input* device here.
// Real loopbacks (output loops straight back to the same device's input) first.
// PICO's Speaker/Mic are NOT a loopback pair (Speaker feeds PICO Connect, Mic
// carries the headset mic), so they can't carry game audio — excluded.
const char* const kLoopbackInputHints[] = {
    "BlackHole",
    "Loopback",     // Rogue Amoeba Loopback
    "Soundflower",
    "VB-",          // VB-Audio Cable
    "Aggregate",    // a user aggregate that includes a loopback
};

std::string DeviceName(AudioObjectID device)
{
    CFStringRef name = nullptr;
    AudioObjectPropertyAddress address = {kAudioObjectPropertyName,
                                          kAudioObjectPropertyScopeGlobal,
                                          kAudioObjectPropertyElementMain};
    UInt32 size = sizeof(name);
    if (AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, &name) != noErr ||
        name == nullptr)
    {
        return {};
    }
    char buffer[256] = {0};
    CFStringGetCString(name, buffer, sizeof(buffer), kCFStringEncodingUTF8);
    CFRelease(name);
    return std::string(buffer);
}

// Find an input-capable device whose name contains the hint (e.g. "BlackHole").
AudioObjectID FindLoopbackInputDevice(std::string& outName)
{
    AudioObjectPropertyAddress listAddr = {kAudioHardwarePropertyDevices,
                                           kAudioObjectPropertyScopeGlobal,
                                           kAudioObjectPropertyElementMain};
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &listAddr, 0, nullptr,
                                       &dataSize) != noErr)
    {
        return kAudioObjectUnknown;
    }
    const UInt32 count = dataSize / sizeof(AudioObjectID);
    std::vector<AudioObjectID> devices(count);
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &listAddr, 0, nullptr,
                                   &dataSize, devices.data()) != noErr)
    {
        return kAudioObjectUnknown;
    }

    auto hasInputChannels = [](AudioObjectID device) -> bool {
        AudioObjectPropertyAddress inAddr = {kAudioDevicePropertyStreamConfiguration,
                                             kAudioObjectPropertyScopeInput,
                                             kAudioObjectPropertyElementMain};
        UInt32 cfgSize = 0;
        if (AudioObjectGetPropertyDataSize(device, &inAddr, 0, nullptr, &cfgSize) != noErr ||
            cfgSize == 0)
        {
            return false;
        }
        std::vector<uint8_t> cfgStorage(cfgSize);
        auto* bufferList = reinterpret_cast<AudioBufferList*>(cfgStorage.data());
        if (AudioObjectGetPropertyData(device, &inAddr, 0, nullptr, &cfgSize, bufferList) != noErr)
        {
            return false;
        }
        for (UInt32 i = 0; i < bufferList->mNumberBuffers; ++i)
        {
            if (bufferList->mBuffers[i].mNumberChannels > 0)
            {
                return true;
            }
        }
        return false;
    };

    // Prefer hints in order (PICO first, then BlackHole, ...).
    for (const char* hint : kLoopbackInputHints)
    {
        for (AudioObjectID device : devices)
        {
            if (!hasInputChannels(device))
            {
                continue;
            }
            std::string name = DeviceName(device);
            if (name.find(hint) != std::string::npos)
            {
                outName = name;
                return device;
            }
        }
    }
    return kAudioObjectUnknown;
}
} // namespace

struct AudioCapture::Impl
{
    AudioUnit unit = nullptr;
    SampleCallback callback;
    uint32_t sampleRateHz = 48000;
    uint16_t channels = 2;
    std::vector<uint8_t> renderBuffer; // AudioBufferList + interleaved float storage
    std::atomic<bool> running{false};
};

namespace
{
OSStatus InputCallback(void* inRefCon, AudioUnitRenderActionFlags* ioActionFlags,
                       const AudioTimeStamp* inTimeStamp, UInt32 inBusNumber,
                       UInt32 inNumberFrames, AudioBufferList* /*ioData*/)
{
    auto* impl = static_cast<AudioCapture::Impl*>(inRefCon);
    if (!impl->running.load() || impl->unit == nullptr)
    {
        return noErr;
    }

    const UInt32 bytesNeeded = inNumberFrames * impl->channels * sizeof(float);
    const UInt32 storageNeeded = sizeof(AudioBufferList) + bytesNeeded;
    if (impl->renderBuffer.size() < storageNeeded)
    {
        impl->renderBuffer.resize(storageNeeded);
    }
    auto* list = reinterpret_cast<AudioBufferList*>(impl->renderBuffer.data());
    list->mNumberBuffers = 1;
    list->mBuffers[0].mNumberChannels = impl->channels;
    list->mBuffers[0].mDataByteSize = bytesNeeded;
    list->mBuffers[0].mData = impl->renderBuffer.data() + sizeof(AudioBufferList);

    OSStatus status = AudioUnitRender(impl->unit, ioActionFlags, inTimeStamp, inBusNumber,
                                      inNumberFrames, list);
    if (status != noErr)
    {
        return status;
    }
    impl->callback(static_cast<const float*>(list->mBuffers[0].mData), inNumberFrames,
                   impl->sampleRateHz, impl->channels);
    return noErr;
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
    impl_->callback = std::move(callback);

    @autoreleasepool
    {
        std::string deviceName;
        AudioObjectID device = FindLoopbackInputDevice(deviceName);
        if (device == kAudioObjectUnknown)
        {
            spdlog::warn("AudioCapture: no loopback input device found (install BlackHole and "
                         "route the bottle's audio output to it); headset audio disabled");
            return false;
        }

        AudioComponentDescription desc = {};
        desc.componentType = kAudioUnitType_Output;
        desc.componentSubType = kAudioUnitSubType_HALOutput;
        desc.componentManufacturer = kAudioUnitManufacturer_Apple;
        AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
        if (comp == nullptr || AudioComponentInstanceNew(comp, &impl_->unit) != noErr)
        {
            spdlog::error("AudioCapture: could not create HAL AudioUnit");
            return false;
        }

        // Enable input (bus 1), disable output (bus 0).
        UInt32 enable = 1, disable = 0;
        AudioUnitSetProperty(impl_->unit, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Input, 1, &enable, sizeof(enable));
        AudioUnitSetProperty(impl_->unit, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Output, 0, &disable, sizeof(disable));

        // Bind to the loopback device.
        if (AudioUnitSetProperty(impl_->unit, kAudioOutputUnitProperty_CurrentDevice,
                                 kAudioUnitScope_Global, 0, &device, sizeof(device)) != noErr)
        {
            spdlog::error("AudioCapture: could not bind AudioUnit to '{}'", deviceName);
            Stop();
            return false;
        }

        // Ask the device for its input sample rate; keep interleaved stereo float32.
        Float64 rate = 48000.0;
        UInt32 rateSize = sizeof(rate);
        AudioObjectPropertyAddress rateAddr = {kAudioDevicePropertyNominalSampleRate,
                                               kAudioObjectPropertyScopeGlobal,
                                               kAudioObjectPropertyElementMain};
        AudioObjectGetPropertyData(device, &rateAddr, 0, nullptr, &rateSize, &rate);
        impl_->sampleRateHz = static_cast<uint32_t>(rate + 0.5);
        impl_->channels = 2;

        AudioStreamBasicDescription format = {};
        format.mSampleRate = rate;
        format.mFormatID = kAudioFormatLinearPCM;
        format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        format.mChannelsPerFrame = impl_->channels;
        format.mBitsPerChannel = 32;
        format.mFramesPerPacket = 1;
        format.mBytesPerFrame = sizeof(float) * impl_->channels;
        format.mBytesPerPacket = format.mBytesPerFrame;
        // Output scope of the input bus = the format we receive.
        if (AudioUnitSetProperty(impl_->unit, kAudioUnitProperty_StreamFormat,
                                 kAudioUnitScope_Output, 1, &format, sizeof(format)) != noErr)
        {
            spdlog::error("AudioCapture: could not set capture format");
            Stop();
            return false;
        }

        AURenderCallbackStruct cb = {};
        cb.inputProc = &InputCallback;
        cb.inputProcRefCon = impl_.get();
        AudioUnitSetProperty(impl_->unit, kAudioOutputUnitProperty_SetInputCallback,
                             kAudioUnitScope_Global, 0, &cb, sizeof(cb));

        if (AudioUnitInitialize(impl_->unit) != noErr)
        {
            spdlog::error("AudioCapture: AudioUnitInitialize failed (Microphone permission for "
                          "CrossOver may be required)");
            Stop();
            return false;
        }

        impl_->running.store(true);
        if (AudioOutputUnitStart(impl_->unit) != noErr)
        {
            spdlog::error("AudioCapture: AudioOutputUnitStart failed");
            impl_->running.store(false);
            Stop();
            return false;
        }

        spdlog::info("AudioCapture: capturing game audio from '{}' ({} Hz, {} ch)",
                     deviceName, impl_->sampleRateHz, impl_->channels);
    }
    return true;
}

void AudioCapture::Stop()
{
    if (!impl_)
    {
        return;
    }
    bool was = impl_->running.exchange(false);
    if (impl_->unit != nullptr)
    {
        AudioOutputUnitStop(impl_->unit);
        AudioUnitUninitialize(impl_->unit);
        AudioComponentInstanceDispose(impl_->unit);
        impl_->unit = nullptr;
    }
    impl_->callback = nullptr;
    if (was)
    {
        spdlog::info("AudioCapture: stopped");
    }
}

} // namespace oxrsys
