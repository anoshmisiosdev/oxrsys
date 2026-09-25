#include "AudioCapture.h"

#import <CoreAudio/CoreAudio.h>
#import <AudioUnit/AudioUnit.h>
#import <Foundation/Foundation.h>

#include <spdlog/spdlog.h>
#include "AudioRingReader.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <time.h>
#include <string>
#include <vector>

// AudioCapture — capture the game's audio on macOS and deliver interleaved
// stereo float32 PCM to a callback.
//
// Core Audio process/system taps do NOT work from inside CrossOver: capturing
// audio needs the "System Audio Recording" TCC permission, and CrossOver's
// Info.plist has no NSAudioCaptureUsageDescription, so a tap created here only
// ever yields silence. Two sources work instead:
//
//  * Tap ring: the OXRSys launcher (a separate app that holds the permission)
//    taps the game's Wine process and writes PCM into a shared-memory ring that
//    AudioRingReader maps read-only. No virtual device or output routing needed.
//  * Loopback: a loopback INPUT device (BlackHole) that the bottle's output is
//    routed to, read back with CrossOver's Microphone permission (it has that).

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
    // Loopback device path.
    AudioUnit unit = nullptr;
    uint32_t sampleRateHz = 48000;
    uint16_t channels = 2;
    std::vector<uint8_t> renderBuffer; // AudioBufferList + interleaved float storage
    std::atomic<bool> loopbackRunning{false};

    // Tap ring path.
    std::thread ringThread;
    std::atomic<bool> ringThreadRunning{false};
    std::atomic<bool> ringLive{false};

    SampleCallback callback;
    std::atomic<bool> running{false};
};

namespace
{
OSStatus InputCallback(void* inRefCon, AudioUnitRenderActionFlags* ioActionFlags,
                       const AudioTimeStamp* inTimeStamp, UInt32 inBusNumber,
                       UInt32 inNumberFrames, AudioBufferList* /*ioData*/)
{
    auto* impl = static_cast<AudioCapture::Impl*>(inRefCon);
    if (!impl->loopbackRunning.load() || impl->unit == nullptr)
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
    // The tap ring wins while its writer is live (auto mode).
    if (impl->ringLive.load(std::memory_order_relaxed))
    {
        return noErr;
    }
    impl->callback(static_cast<const float*>(list->mBuffers[0].mData), inNumberFrames,
                   impl->sampleRateHz, impl->channels, "loopback");
    return noErr;
}

uint64_t RealtimeNowNs()
{
    timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

// Polls the tap ring and forwards new PCM, resampled to the stream rate, in
// bounded chunks. Never blocks on the writer: a missing or stale ring just
// means nothing is delivered (and the loopback path takes over in auto mode).
void RingThreadMain(AudioCapture::Impl* impl)
{
    AudioRingReader reader(AudioRingReader::DefaultPath());
    LinearResampler resampler;
    std::vector<float> block;
    std::vector<float> converted;
    constexpr uint32_t kChunkFrames = 480; // 10 ms at 48 kHz
    AudioRingReader::State lastState = AudioRingReader::State::NoRing;
    uint64_t lastOverruns = 0;
    uint64_t lastSkips = 0;
    auto lastDiagLog = std::chrono::steady_clock::now();

    while (impl->ringThreadRunning.load())
    {
        uint32_t rate = 0;
        uint16_t channels = 0;
        const AudioRingReader::State state = reader.Poll(RealtimeNowNs(), block, rate, channels);
        const bool live = state == AudioRingReader::State::Live;
        if (state != lastState)
        {
            if (live)
            {
                spdlog::info("AudioCapture: launcher tap ring live ({} Hz, {} ch, scope={}); "
                             "streaming tapped audio",
                             rate, channels,
                             reader.Scope() == 1 ? "game" : reader.Scope() == 2 ? "system" : "?");
            }
            else if (lastState == AudioRingReader::State::Live)
            {
                spdlog::info("AudioCapture: launcher tap ring stopped");
                resampler.Reset();
            }
            lastState = state;
        }
        impl->ringLive.store(live, std::memory_order_relaxed);

        if (live && !block.empty() && channels > 0)
        {
            converted.clear();
            resampler.Process(block.data(), block.size() / channels, rate,
                              AudioCapture::kStreamSampleRateHz, channels, converted);
            const size_t frames = converted.size() / channels;
            for (size_t offset = 0; offset < frames; offset += kChunkFrames)
            {
                const uint32_t n =
                    static_cast<uint32_t>(std::min<size_t>(kChunkFrames, frames - offset));
                impl->callback(converted.data() + offset * channels, n,
                               AudioCapture::kStreamSampleRateHz, channels, "tap");
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - lastDiagLog > std::chrono::seconds(10))
        {
            if (reader.OverrunCount() != lastOverruns || reader.SkipCount() != lastSkips)
            {
                spdlog::info("AudioCapture: tap ring overruns={} backlog skips={}",
                             reader.OverrunCount(), reader.SkipCount());
                lastOverruns = reader.OverrunCount();
                lastSkips = reader.SkipCount();
            }
            lastDiagLog = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(live ? 5 : 100));
    }
    impl->ringLive.store(false);
}

bool StartLoopback(AudioCapture::Impl* impl, bool quietIfMissing)
{
    @autoreleasepool
    {
        std::string deviceName;
        AudioObjectID device = FindLoopbackInputDevice(deviceName);
        if (device == kAudioObjectUnknown)
        {
            if (quietIfMissing)
            {
                spdlog::info("AudioCapture: no loopback input device; waiting for the "
                             "launcher tap ring only");
            }
            else
            {
                spdlog::warn("AudioCapture: no loopback input device found (install BlackHole "
                             "and route the bottle's audio output to it)");
            }
            return false;
        }

        AudioComponentDescription desc = {};
        desc.componentType = kAudioUnitType_Output;
        desc.componentSubType = kAudioUnitSubType_HALOutput;
        desc.componentManufacturer = kAudioUnitManufacturer_Apple;
        AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
        if (comp == nullptr || AudioComponentInstanceNew(comp, &impl->unit) != noErr)
        {
            spdlog::error("AudioCapture: could not create HAL AudioUnit");
            impl->unit = nullptr;
            return false;
        }

        auto fail = [impl](const char* message) {
            spdlog::error("AudioCapture: {}", message);
            impl->loopbackRunning.store(false);
            AudioUnitUninitialize(impl->unit);
            AudioComponentInstanceDispose(impl->unit);
            impl->unit = nullptr;
            return false;
        };

        // Enable input (bus 1), disable output (bus 0).
        UInt32 enable = 1, disable = 0;
        AudioUnitSetProperty(impl->unit, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Input, 1, &enable, sizeof(enable));
        AudioUnitSetProperty(impl->unit, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Output, 0, &disable, sizeof(disable));

        if (AudioUnitSetProperty(impl->unit, kAudioOutputUnitProperty_CurrentDevice,
                                 kAudioUnitScope_Global, 0, &device, sizeof(device)) != noErr)
        {
            return fail("could not bind AudioUnit to the loopback device");
        }

        // Ask the device for its input sample rate; keep interleaved stereo float32.
        Float64 rate = 48000.0;
        UInt32 rateSize = sizeof(rate);
        AudioObjectPropertyAddress rateAddr = {kAudioDevicePropertyNominalSampleRate,
                                               kAudioObjectPropertyScopeGlobal,
                                               kAudioObjectPropertyElementMain};
        AudioObjectGetPropertyData(device, &rateAddr, 0, nullptr, &rateSize, &rate);
        impl->sampleRateHz = static_cast<uint32_t>(rate + 0.5);
        impl->channels = 2;

        AudioStreamBasicDescription format = {};
        format.mSampleRate = rate;
        format.mFormatID = kAudioFormatLinearPCM;
        format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        format.mChannelsPerFrame = impl->channels;
        format.mBitsPerChannel = 32;
        format.mFramesPerPacket = 1;
        format.mBytesPerFrame = sizeof(float) * impl->channels;
        format.mBytesPerPacket = format.mBytesPerFrame;
        // Output scope of the input bus = the format we receive.
        if (AudioUnitSetProperty(impl->unit, kAudioUnitProperty_StreamFormat,
                                 kAudioUnitScope_Output, 1, &format, sizeof(format)) != noErr)
        {
            return fail("could not set capture format");
        }

        AURenderCallbackStruct cb = {};
        cb.inputProc = &InputCallback;
        cb.inputProcRefCon = impl;
        AudioUnitSetProperty(impl->unit, kAudioOutputUnitProperty_SetInputCallback,
                             kAudioUnitScope_Global, 0, &cb, sizeof(cb));

        if (AudioUnitInitialize(impl->unit) != noErr)
        {
            return fail("AudioUnitInitialize failed (Microphone permission for CrossOver may "
                        "be required)");
        }

        impl->loopbackRunning.store(true);
        if (AudioOutputUnitStart(impl->unit) != noErr)
        {
            return fail("AudioOutputUnitStart failed");
        }

        spdlog::info("AudioCapture: capturing loopback audio from '{}' ({} Hz, {} ch)",
                     deviceName, impl->sampleRateHz, impl->channels);
    }
    return true;
}
} // namespace

AudioCapture::Source AudioCapture::ParseSource(const std::string& value)
{
    if (value == "tap")
    {
        return Source::Tap;
    }
    if (value == "loopback")
    {
        return Source::Loopback;
    }
    return Source::Auto;
}

AudioCapture::AudioCapture() : impl_(std::make_unique<Impl>()) {}
AudioCapture::~AudioCapture() { Stop(); }
bool AudioCapture::IsRunning() const { return impl_ && impl_->running.load(); }

const char* AudioCapture::ActiveSourceName() const
{
    if (!impl_ || !impl_->running.load())
    {
        return "none";
    }
    if (impl_->ringLive.load())
    {
        return "tap";
    }
    return impl_->loopbackRunning.load() ? "loopback" : "none";
}

bool AudioCapture::Start(SampleCallback callback, Source source)
{
    if (impl_->running.load())
    {
        return true;
    }
    impl_->callback = std::move(callback);

    bool anyStarted = false;
    if (source != Source::Tap)
    {
        anyStarted = StartLoopback(impl_.get(), source == Source::Auto);
    }
    if (source != Source::Loopback)
    {
        impl_->ringThreadRunning.store(true);
        impl_->ringThread = std::thread(RingThreadMain, impl_.get());
        anyStarted = true;
        spdlog::info("AudioCapture: watching launcher tap ring at {}",
                     AudioRingReader::DefaultPath());
    }
    impl_->running.store(anyStarted);
    if (!anyStarted)
    {
        impl_->callback = nullptr;
    }
    return anyStarted;
}

void AudioCapture::Stop()
{
    if (!impl_)
    {
        return;
    }
    bool was = impl_->running.exchange(false);
    impl_->ringThreadRunning.store(false);
    if (impl_->ringThread.joinable())
    {
        impl_->ringThread.join();
    }
    impl_->loopbackRunning.store(false);
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
