// SPDX-License-Identifier: MPL-2.0
//
// WiredHeadset for builds without a wired-headset driver: never open.

#include "WiredHeadset.h"

struct WiredHeadset::Impl
{
};

WiredHeadset& WiredHeadset::Shared()
{
    static WiredHeadset shared;
    return shared;
}

WiredHeadset::WiredHeadset() = default;
WiredHeadset::~WiredHeadset() = default;

bool WiredHeadset::IsSupported()
{
    return false;
}

bool WiredHeadset::EnsureOpen(const ConfigValues&)
{
    return false;
}

bool WiredHeadset::IsOpen() const
{
    return false;
}

void WiredHeadset::Close()
{
}

uint32_t WiredHeadset::GetEyeWidth() const
{
    return 0;
}

uint32_t WiredHeadset::GetEyeHeight() const
{
    return 0;
}

uint32_t WiredHeadset::GetRefreshRateHz() const
{
    return 0;
}

std::string WiredHeadset::GetName() const
{
    return {};
}

bool WiredHeadset::AttachGraphics(const GraphicsContext&)
{
    return false;
}

void WiredHeadset::DetachGraphics()
{
}

bool WiredHeadset::IsPresenting() const
{
    return false;
}

void WiredHeadset::SendFrame(FrameSource)
{
}

TrackingReceiver* WiredHeadset::GetTrackingReceiver()
{
    return nullptr;
}

uint64_t WiredHeadset::GetPresentedFrameCount() const
{
    return 0;
}
