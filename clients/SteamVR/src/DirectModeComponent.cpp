// SPDX-License-Identifier: MPL-2.0

#include "DirectModeComponent.h"

#include "DriverLog.h"

namespace oxrsys
{

DirectModeComponent::DirectModeComponent() = default;

DirectModeComponent::~DirectModeComponent() = default;

void DirectModeComponent::CreateSwapTextureSet(uint32_t unPid, const SwapTextureSetDesc_t* pSwapTextureSetDesc, SwapTextureSet_t* pOutSwapTextureSet)
{
    if (pOutSwapTextureSet == nullptr)
    {
        return;
    }

    *pOutSwapTextureSet = {};

    std::lock_guard<std::mutex> lock(mutex_);
    ++swapTextureSetsRequested_;

    if (pSwapTextureSetDesc != nullptr)
    {
        OXRSYS_LOG("[oxrsys] CreateSwapTextureSet pid=%u %ux%u format=%u samples=%u (not yet allocated)",
                   unPid,
                   pSwapTextureSetDesc->nWidth,
                   pSwapTextureSetDesc->nHeight,
                   pSwapTextureSetDesc->nFormat,
                   pSwapTextureSetDesc->nSampleCount);
    }
}

void DirectModeComponent::DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle)
{
    (void)sharedTextureHandle;
}

void DirectModeComponent::DestroyAllSwapTextureSets(uint32_t unPid)
{
    OXRSYS_LOG("[oxrsys] DestroyAllSwapTextureSets pid=%u", unPid);
}

void DirectModeComponent::GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t (*pIndices)[2])
{
    (void)sharedTextureHandles;
    if (pIndices == nullptr)
    {
        return;
    }

    (*pIndices)[0] = ((*pIndices)[0] + 1) % 3;
    (*pIndices)[1] = ((*pIndices)[1] + 1) % 3;
}

void DirectModeComponent::SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2])
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!loggedFirstSubmit_)
    {
        loggedFirstSubmit_ = true;
        OXRSYS_LOG("[oxrsys] first SubmitLayer: left=0x%llx right=0x%llx",
                   static_cast<unsigned long long>(perEye[0].hTexture),
                   static_cast<unsigned long long>(perEye[1].hTexture));
    }
}

void DirectModeComponent::Present(vr::SharedTextureHandle_t syncTexture)
{
    (void)syncTexture;

    std::lock_guard<std::mutex> lock(mutex_);
    ++framesPresented_;
    if (framesPresented_ == 1 || (framesPresented_ % 600) == 0)
    {
        OXRSYS_LOG("[oxrsys] Present frame %llu", static_cast<unsigned long long>(framesPresented_));
    }
}

void DirectModeComponent::PostPresent(const Throttling_t* pThrottling)
{
    (void)pThrottling;
}

void DirectModeComponent::GetFrameTiming(vr::DriverDirectMode_FrameTiming* pFrameTiming)
{
    if (pFrameTiming == nullptr)
    {
        return;
    }

    // Reprojection flags overlap the throttle mask, so they have to be cleared
    // or SteamVR reads them back as a throttling request.
    pFrameTiming->m_nReprojectionFlags = 0;
}

} // namespace oxrsys
