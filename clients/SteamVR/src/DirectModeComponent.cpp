// SPDX-License-Identifier: MPL-2.0

#include "DirectModeComponent.h"

#include "DriverLog.h"

#include <dxgi.h>

#include <cstring>

namespace oxrsys
{

namespace
{

// Legacy shared handles rather than NT handles: SteamVR accepts both (the
// difference is reported through VRSwapTextureFlag_Shared_NTHandle), and a
// legacy handle is valid in any process on the same adapter, which avoids
// having to duplicate it into the application process by pid.
constexpr UINT kSharedMiscFlags = D3D11_RESOURCE_MISC_SHARED;

vr::SharedTextureHandle_t SharedHandleOf(ID3D11Texture2D* texture)
{
    IDXGIResource* resource = nullptr;
    if (FAILED(texture->QueryInterface(__uuidof(IDXGIResource), reinterpret_cast<void**>(&resource))) || resource == nullptr)
    {
        return 0;
    }

    HANDLE handle = nullptr;
    const HRESULT hr = resource->GetSharedHandle(&handle);
    resource->Release();

    if (FAILED(hr) || handle == nullptr)
    {
        return 0;
    }

    return reinterpret_cast<vr::SharedTextureHandle_t>(handle);
}

} // namespace

DirectModeComponent::DirectModeComponent() = default;

DirectModeComponent::~DirectModeComponent()
{
    if (oxrStartThread_.joinable())
    {
        oxrStartThread_.join();
    }
    oxrClient_.Stop();

    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = textureSets_.size(); i > 0; --i)
    {
        DestroySetLocked(i - 1);
    }

    for (auto& entry : openedTextures_)
    {
        if (entry.second != nullptr)
        {
            entry.second->Release();
        }
    }
    openedTextures_.clear();

    if (context_ != nullptr)
    {
        context_->Release();
        context_ = nullptr;
    }

    if (device_ != nullptr)
    {
        device_->Release();
        device_ = nullptr;
    }
}

bool DirectModeComponent::EnsureDeviceLocked()
{
    if (device_ != nullptr)
    {
        return true;
    }

    if (deviceInitFailed_)
    {
        return false;
    }

    static const D3D_FEATURE_LEVEL kFeatureLevels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};

    D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;
    const HRESULT hr = D3D11CreateDevice(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        0,
        kFeatureLevels,
        static_cast<UINT>(sizeof(kFeatureLevels) / sizeof(kFeatureLevels[0])),
        D3D11_SDK_VERSION,
        &device_,
        &featureLevel,
        &context_);

    if (FAILED(hr) || device_ == nullptr)
    {
        deviceInitFailed_ = true;
        OXRSYS_LOG("[oxrsys] D3D11CreateDevice failed: 0x%08lx", static_cast<unsigned long>(hr));
        return false;
    }

    OXRSYS_LOG("[oxrsys] D3D11 device created, feature level 0x%04x", static_cast<unsigned>(featureLevel));
    return true;
}

void DirectModeComponent::DestroySetLocked(size_t index)
{
    SwapTextureSet& set = textureSets_[index];
    for (ID3D11Texture2D* texture : set.textures)
    {
        if (texture != nullptr)
        {
            texture->Release();
        }
    }

    textureSets_.erase(textureSets_.begin() + static_cast<ptrdiff_t>(index));
}

void DirectModeComponent::CreateSwapTextureSet(uint32_t unPid, const SwapTextureSetDesc_t* pSwapTextureSetDesc, SwapTextureSet_t* pOutSwapTextureSet)
{
    if (pOutSwapTextureSet == nullptr)
    {
        return;
    }

    *pOutSwapTextureSet = {};

    if (pSwapTextureSetDesc == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (!EnsureDeviceLocked())
    {
        return;
    }

    D3D11_TEXTURE2D_DESC desc = {};
    desc.Width = pSwapTextureSetDesc->nWidth;
    desc.Height = pSwapTextureSetDesc->nHeight;
    desc.MipLevels = 1;
    desc.ArraySize = 1;
    desc.Format = static_cast<DXGI_FORMAT>(pSwapTextureSetDesc->nFormat);
    desc.SampleDesc.Count = pSwapTextureSetDesc->nSampleCount == 0 ? 1 : pSwapTextureSetDesc->nSampleCount;
    desc.SampleDesc.Quality = 0;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    desc.CPUAccessFlags = 0;
    desc.MiscFlags = kSharedMiscFlags;

    SwapTextureSet set;
    set.pid = unPid;

    for (size_t i = 0; i < set.textures.size(); ++i)
    {
        const HRESULT hr = device_->CreateTexture2D(&desc, nullptr, &set.textures[i]);
        if (FAILED(hr) || set.textures[i] == nullptr)
        {
            OXRSYS_LOG("[oxrsys] CreateSwapTextureSet: CreateTexture2D %ux%u format=%u failed: 0x%08lx",
                       desc.Width,
                       desc.Height,
                       static_cast<unsigned>(desc.Format),
                       static_cast<unsigned long>(hr));
            for (ID3D11Texture2D* texture : set.textures)
            {
                if (texture != nullptr)
                {
                    texture->Release();
                }
            }
            return;
        }

        set.handles[i] = SharedHandleOf(set.textures[i]);
        if (set.handles[i] == 0)
        {
            OXRSYS_LOG("[oxrsys] CreateSwapTextureSet: texture %zu is not shareable", i);
            for (ID3D11Texture2D* texture : set.textures)
            {
                if (texture != nullptr)
                {
                    texture->Release();
                }
            }
            return;
        }
    }

    for (size_t i = 0; i < set.handles.size(); ++i)
    {
        pOutSwapTextureSet->rSharedTextureHandles[i] = set.handles[i];
    }
    pOutSwapTextureSet->unTextureFlags = 0;

    OXRSYS_LOG("[oxrsys] CreateSwapTextureSet pid=%u %ux%u format=%u samples=%u -> 0x%llx 0x%llx 0x%llx",
               unPid,
               pSwapTextureSetDesc->nWidth,
               pSwapTextureSetDesc->nHeight,
               pSwapTextureSetDesc->nFormat,
               pSwapTextureSetDesc->nSampleCount,
               static_cast<unsigned long long>(set.handles[0]),
               static_cast<unsigned long long>(set.handles[1]),
               static_cast<unsigned long long>(set.handles[2]));

    textureSets_.push_back(set);
}

void DirectModeComponent::DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < textureSets_.size(); ++i)
    {
        for (vr::SharedTextureHandle_t handle : textureSets_[i].handles)
        {
            if (handle != 0 && handle == sharedTextureHandle)
            {
                DestroySetLocked(i);
                return;
            }
        }
    }
}

void DirectModeComponent::DestroyAllSwapTextureSets(uint32_t unPid)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = textureSets_.size(); i > 0; --i)
    {
        if (textureSets_[i - 1].pid == unPid)
        {
            DestroySetLocked(i - 1);
        }
    }
}

void DirectModeComponent::GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t (*pIndices)[2])
{
    if (pIndices == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t eye = 0; eye < 2; ++eye)
    {
        uint32_t next = ((*pIndices)[eye] + 1) % 3;

        if (sharedTextureHandles != nullptr && sharedTextureHandles[eye] != 0)
        {
            for (SwapTextureSet& set : textureSets_)
            {
                bool owned = false;
                for (vr::SharedTextureHandle_t handle : set.handles)
                {
                    owned = owned || handle == sharedTextureHandles[eye];
                }

                if (owned)
                {
                    set.nextIndex = (set.nextIndex + 1) % static_cast<uint32_t>(set.handles.size());
                    next = set.nextIndex;
                    break;
                }
            }
        }

        (*pIndices)[eye] = next;
    }
}

ID3D11Texture2D* DirectModeComponent::TextureForHandleLocked(vr::SharedTextureHandle_t handle) const
{
    if (handle == 0)
    {
        return nullptr;
    }

    for (const SwapTextureSet& set : textureSets_)
    {
        for (size_t i = 0; i < set.handles.size(); ++i)
        {
            if (set.handles[i] == handle)
            {
                return set.textures[i];
            }
        }
    }

    return nullptr;
}

ID3D11Texture2D* DirectModeComponent::OpenSharedTextureLocked(vr::SharedTextureHandle_t handle)
{
    if (handle == 0 || device_ == nullptr)
    {
        return nullptr;
    }

    for (const auto& entry : openedTextures_)
    {
        if (entry.first == handle)
        {
            return entry.second;
        }
    }

    ID3D11Texture2D* texture = nullptr;
    const HRESULT hr = device_->OpenSharedResource(
        reinterpret_cast<HANDLE>(handle), __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&texture));

    if (FAILED(hr) || texture == nullptr)
    {
        OXRSYS_LOG("[oxrsys] OpenSharedResource(0x%llx) failed: 0x%08lx",
                   static_cast<unsigned long long>(handle),
                   static_cast<unsigned long>(hr));
        openedTextures_.emplace_back(handle, nullptr);
        return nullptr;
    }

    openedTextures_.emplace_back(handle, texture);
    return texture;
}

void DirectModeComponent::StartOxrClientAsync()
{
    if (oxrStartRequested_ || device_ == nullptr)
    {
        return;
    }

    oxrStartRequested_ = true;
    ID3D11Device* device = device_;
    ID3D11DeviceContext* context = context_;
    oxrStartThread_ = std::thread([this, device, context] { oxrClient_.Start(device, context); });
}

void DirectModeComponent::SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2])
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (layersThisFrame_ == 0)
    {
        submittedEyes_[0] = perEye[0].hTexture;
        submittedEyes_[1] = perEye[1].hTexture;
    }
    ++layersThisFrame_;

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
    ID3D11Texture2D* leftEye = nullptr;
    ID3D11Texture2D* rightEye = nullptr;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!loggedSyncTextureState_ && framesPresented_ == 0)
        {
            OXRSYS_LOG("[oxrsys] Present sync texture handle: 0x%llx",
                       static_cast<unsigned long long>(syncTexture));
        }

        // The sync texture is deliberately not taken.
        //
        // Holding its keyed mutex across the handover is the documented
        // contract, but doing so stopped the compositor dead after five
        // presents: before this driver took the mutex it sustained thousands,
        // after it took the mutex it stopped every time. Releasing on key 0
        // evidently does not hand it back in a state the compositor can
        // re-acquire under DXMT. Frames matter more than the handshake, so the
        // driver leaves the mutex alone; the cost is that a frame can be read
        // while the compositor is still writing it.
        (void)syncTexture;

        ++framesPresented_;
        if (framesPresented_ <= 5 || (framesPresented_ % 600) == 0)
        {
            OXRSYS_LOG("[oxrsys] Present frame %llu, %u layer(s), scene left=0x%llx right=0x%llx",
                       static_cast<unsigned long long>(framesPresented_),
                       layersThisFrame_,
                       static_cast<unsigned long long>(submittedEyes_[0]),
                       static_cast<unsigned long long>(submittedEyes_[1]));
        }
        layersThisFrame_ = 0;

        StartOxrClientAsync();

        leftEye = TextureForHandleLocked(submittedEyes_[0]);
        rightEye = TextureForHandleLocked(submittedEyes_[1]);

        if (leftEye == nullptr && submittedEyes_[0] != 0 && framesPresented_ <= 5)
        {
            OXRSYS_LOG("[oxrsys] submitted handle 0x%llx is not one this driver allocated",
                       static_cast<unsigned long long>(submittedEyes_[0]));
        }

    }

    // Hand the textures over and return. The runtime's frame loop runs on its
    // own thread, so SteamVR's compositor is never made to wait on it.
    oxrClient_.SetPendingEyes(leftEye, rightEye);

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
