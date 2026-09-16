// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <d3d11.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace oxrsys
{

// A source rectangle in normalised texture coordinates, in the order SteamVR
// reports them: uMin, vMin, uMax, vMax. vMin greater than vMax is meaningful
// rather than an error -- it is how an application says its render is already
// the other way up.
struct UvRect
{
    float uMin = 0.0f;
    float vMin = 0.0f;
    float uMax = 1.0f;
    float vMax = 1.0f;
};

// Draws one texture into another through a full-screen triangle.
//
// A straight CopySubresourceRegion cannot do this job: it is a memory copy and
// has no notion of orientation, so it cannot correct the vertical flip between
// what SteamVR renders and what the runtime expects, and it cannot honour the
// source rectangle SteamVR supplies. Sampling through a shader can do both, and
// costs one triangle per eye per frame.
class TextureBlitter
{
public:
    ~TextureBlitter();

    bool Initialize(ID3D11Device* device, ID3D11DeviceContext* context);

    bool IsReady() const
    {
        return ready_;
    }

    // Draws `source` into `destination`, sampling the given rectangle. The
    // formats are passed explicitly because the runtime's swapchain images
    // arrive as typeless textures, which cannot be viewed without one.
    bool Blit(ID3D11Texture2D* source,
              DXGI_FORMAT sourceFormat,
              const UvRect& sourceRect,
              ID3D11Texture2D* destination,
              DXGI_FORMAT destinationFormat,
              uint32_t destinationWidth,
              uint32_t destinationHeight);

    void Reset();

private:
    ID3D11ShaderResourceView* ShaderResourceViewFor(ID3D11Texture2D* texture, DXGI_FORMAT format);
    ID3D11RenderTargetView* RenderTargetViewFor(ID3D11Texture2D* texture, DXGI_FORMAT format);

    ID3D11Device* device_ = nullptr;
    ID3D11DeviceContext* context_ = nullptr;

    ID3D11VertexShader* vertexShader_ = nullptr;
    ID3D11PixelShader* pixelShader_ = nullptr;
    ID3D11SamplerState* sampler_ = nullptr;
    ID3D11Buffer* constantBuffer_ = nullptr;
    ID3D11BlendState* blendState_ = nullptr;
    ID3D11RasterizerState* rasterizerState_ = nullptr;
    ID3D11DepthStencilState* depthStencilState_ = nullptr;

    std::vector<std::pair<ID3D11Texture2D*, ID3D11ShaderResourceView*>> shaderResourceViews_;
    std::vector<std::pair<ID3D11Texture2D*, ID3D11RenderTargetView*>> renderTargetViews_;

    bool ready_ = false;
};

} // namespace oxrsys
