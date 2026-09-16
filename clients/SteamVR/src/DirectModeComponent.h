// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <openvr_driver.h>

#include <cstdint>
#include <mutex>

namespace oxrsys
{

// Direct-mode presentation.
//
// With this component present the SteamVR compositor stops owning a swapchain
// of its own: it asks the driver to allocate the shared textures applications
// render into, then hands those textures back once per frame through
// SubmitLayer/Present. That is what lets frames reach OXRSys without going
// through a desktop DXGI present, which is the step that stalls when the
// compositor runs under Wine.
//
// None of the methods return a class in memory, so this interface can be
// derived from the upstream declaration directly.
class DirectModeComponent final : public vr::IVRDriverDirectModeComponent
{
public:
    DirectModeComponent();
    ~DirectModeComponent();

    void CreateSwapTextureSet(uint32_t unPid, const SwapTextureSetDesc_t* pSwapTextureSetDesc, SwapTextureSet_t* pOutSwapTextureSet) override;
    void DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle) override;
    void DestroyAllSwapTextureSets(uint32_t unPid) override;
    void GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t (*pIndices)[2]) override;
    void SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2]) override;
    void Present(vr::SharedTextureHandle_t syncTexture) override;
    void PostPresent(const Throttling_t* pThrottling) override;
    void GetFrameTiming(vr::DriverDirectMode_FrameTiming* pFrameTiming) override;

private:
    std::mutex mutex_;
    uint64_t framesPresented_ = 0;
    uint32_t swapTextureSetsRequested_ = 0;
    bool loggedFirstSubmit_ = false;
};

} // namespace oxrsys
