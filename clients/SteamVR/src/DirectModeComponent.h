// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <openvr_driver.h>

#include <d3d11.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

namespace oxrsys
{

// Direct-mode presentation.
//
// With this component present the SteamVR compositor stops owning a swapchain
// of its own: it asks the driver to allocate the shared textures it and the
// running application render into, then hands those textures back once per
// frame through SubmitLayer/Present. That is what lets frames reach OXRSys
// without going through a desktop DXGI present, which is the step that stalls
// when the compositor runs under Wine.
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
    // One set of three textures the compositor rotates through, plus the
    // process it was allocated for so DestroyAllSwapTextureSets can find it.
    struct SwapTextureSet
    {
        uint32_t pid = 0;
        uint32_t nextIndex = 0;
        std::array<ID3D11Texture2D*, 3> textures = {};
        std::array<vr::SharedTextureHandle_t, 3> handles = {};
    };

    // Creates the D3D11 device the shared textures are allocated on, once.
    // Returns false and logs if the device cannot be created; callers then hand
    // SteamVR an empty set, which fails start-up cleanly instead of crashing.
    bool EnsureDeviceLocked();

    void DestroySetLocked(size_t index);

    std::mutex mutex_;
    ID3D11Device* device_ = nullptr;
    ID3D11DeviceContext* context_ = nullptr;
    bool deviceInitFailed_ = false;
    std::vector<SwapTextureSet> textureSets_;
    uint64_t framesPresented_ = 0;
    bool loggedFirstSubmit_ = false;
};

} // namespace oxrsys
