// SPDX-License-Identifier: MPL-2.0

#include "TextureBlitter.h"

#include "DriverLog.h"

#include <d3dcompiler.h>

#include <cstring>

namespace oxrsys
{

namespace
{

// A full-screen triangle rather than a quad: three vertices, no vertex buffer,
// no input layout, and no seam down the diagonal. The source rectangle arrives
// in a constant buffer so the same shader serves an upright blit, a flipped
// one, and a partial one.
constexpr const char* kShaderSource = R"(
cbuffer SourceRect : register(b0)
{
    float4 rect; // uMin, vMin, uMax, vMax
};

struct Varyings
{
    float4 position : SV_POSITION;
    float2 texcoord : TEXCOORD0;
};

Varyings VertexMain(uint id : SV_VertexID)
{
    float2 corner = float2((id << 1) & 2, id & 2);

    Varyings output;
    output.position = float4(corner * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    output.texcoord = lerp(rect.xy, rect.zw, corner);
    return output;
}

Texture2D sourceTexture : register(t0);
SamplerState sourceSampler : register(s0);

float4 PixelMain(Varyings input) : SV_TARGET
{
    return sourceTexture.Sample(sourceSampler, input.texcoord);
}
)";

ID3DBlob* CompileStage(const char* entryPoint, const char* target)
{
    ID3DBlob* code = nullptr;
    ID3DBlob* errors = nullptr;

    const HRESULT hr = D3DCompile(kShaderSource,
                                  std::strlen(kShaderSource),
                                  nullptr,
                                  nullptr,
                                  nullptr,
                                  entryPoint,
                                  target,
                                  0,
                                  0,
                                  &code,
                                  &errors);

    if (FAILED(hr))
    {
        OXRSYS_LOG("[oxrsys] blitter: compiling %s failed: 0x%08lx %s",
                   entryPoint,
                   static_cast<unsigned long>(hr),
                   errors != nullptr ? static_cast<const char*>(errors->GetBufferPointer()) : "");
    }

    if (errors != nullptr)
    {
        errors->Release();
    }

    return FAILED(hr) ? nullptr : code;
}

} // namespace

TextureBlitter::~TextureBlitter()
{
    Reset();
}

void TextureBlitter::Reset()
{
    for (auto& entry : shaderResourceViews_)
    {
        if (entry.second != nullptr)
        {
            entry.second->Release();
        }
    }
    shaderResourceViews_.clear();

    for (auto& entry : renderTargetViews_)
    {
        if (entry.second != nullptr)
        {
            entry.second->Release();
        }
    }
    renderTargetViews_.clear();

    const auto release = [](auto*& resource) {
        if (resource != nullptr)
        {
            resource->Release();
            resource = nullptr;
        }
    };

    release(depthStencilState_);
    release(rasterizerState_);
    release(blendState_);
    release(constantBuffer_);
    release(sampler_);
    release(pixelShader_);
    release(vertexShader_);

    ready_ = false;
}

bool TextureBlitter::Initialize(ID3D11Device* device, ID3D11DeviceContext* context)
{
    if (ready_)
    {
        return true;
    }

    if (device == nullptr || context == nullptr)
    {
        return false;
    }

    device_ = device;
    context_ = context;

    ID3DBlob* vertexCode = CompileStage("VertexMain", "vs_5_0");
    ID3DBlob* pixelCode = CompileStage("PixelMain", "ps_5_0");
    if (vertexCode == nullptr || pixelCode == nullptr)
    {
        if (vertexCode != nullptr)
        {
            vertexCode->Release();
        }
        if (pixelCode != nullptr)
        {
            pixelCode->Release();
        }
        return false;
    }

    HRESULT hr = device_->CreateVertexShader(
        vertexCode->GetBufferPointer(), vertexCode->GetBufferSize(), nullptr, &vertexShader_);
    if (SUCCEEDED(hr))
    {
        hr = device_->CreatePixelShader(
            pixelCode->GetBufferPointer(), pixelCode->GetBufferSize(), nullptr, &pixelShader_);
    }

    vertexCode->Release();
    pixelCode->Release();

    if (FAILED(hr))
    {
        OXRSYS_LOG("[oxrsys] blitter: creating shaders failed: 0x%08lx", static_cast<unsigned long>(hr));
        Reset();
        return false;
    }

    D3D11_SAMPLER_DESC samplerDesc = {};
    samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
    samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
    samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
    samplerDesc.MaxLOD = D3D11_FLOAT32_MAX;
    hr = device_->CreateSamplerState(&samplerDesc, &sampler_);

    if (SUCCEEDED(hr))
    {
        D3D11_BUFFER_DESC bufferDesc = {};
        bufferDesc.ByteWidth = 16;
        bufferDesc.Usage = D3D11_USAGE_DEFAULT;
        bufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        hr = device_->CreateBuffer(&bufferDesc, nullptr, &constantBuffer_);
    }

    if (SUCCEEDED(hr))
    {
        D3D11_BLEND_DESC blendDesc = {};
        blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        hr = device_->CreateBlendState(&blendDesc, &blendState_);
    }

    if (SUCCEEDED(hr))
    {
        D3D11_RASTERIZER_DESC rasterizerDesc = {};
        rasterizerDesc.FillMode = D3D11_FILL_SOLID;
        rasterizerDesc.CullMode = D3D11_CULL_NONE;
        rasterizerDesc.DepthClipEnable = TRUE;
        hr = device_->CreateRasterizerState(&rasterizerDesc, &rasterizerState_);
    }

    if (SUCCEEDED(hr))
    {
        D3D11_DEPTH_STENCIL_DESC depthDesc = {};
        depthDesc.DepthEnable = FALSE;
        depthDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
        depthDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
        hr = device_->CreateDepthStencilState(&depthDesc, &depthStencilState_);
    }

    if (FAILED(hr))
    {
        OXRSYS_LOG("[oxrsys] blitter: creating pipeline state failed: 0x%08lx", static_cast<unsigned long>(hr));
        Reset();
        return false;
    }

    ready_ = true;
    OXRSYS_LOG("[oxrsys] blitter ready");
    return true;
}

ID3D11ShaderResourceView* TextureBlitter::ShaderResourceViewFor(ID3D11Texture2D* texture, DXGI_FORMAT format)
{
    for (const auto& entry : shaderResourceViews_)
    {
        if (entry.first == texture)
        {
            return entry.second;
        }
    }

    D3D11_SHADER_RESOURCE_VIEW_DESC desc = {};
    desc.Format = format;
    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    desc.Texture2D.MipLevels = 1;

    ID3D11ShaderResourceView* view = nullptr;
    const HRESULT hr = device_->CreateShaderResourceView(texture, &desc, &view);
    if (FAILED(hr))
    {
        OXRSYS_LOG("[oxrsys] blitter: CreateShaderResourceView failed: 0x%08lx", static_cast<unsigned long>(hr));
        view = nullptr;
    }

    shaderResourceViews_.emplace_back(texture, view);
    return view;
}

ID3D11RenderTargetView* TextureBlitter::RenderTargetViewFor(ID3D11Texture2D* texture, DXGI_FORMAT format)
{
    for (const auto& entry : renderTargetViews_)
    {
        if (entry.first == texture)
        {
            return entry.second;
        }
    }

    D3D11_RENDER_TARGET_VIEW_DESC desc = {};
    desc.Format = format;
    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;

    ID3D11RenderTargetView* view = nullptr;
    const HRESULT hr = device_->CreateRenderTargetView(texture, &desc, &view);
    if (FAILED(hr))
    {
        OXRSYS_LOG("[oxrsys] blitter: CreateRenderTargetView failed: 0x%08lx", static_cast<unsigned long>(hr));
        view = nullptr;
    }

    renderTargetViews_.emplace_back(texture, view);
    return view;
}

bool TextureBlitter::Blit(ID3D11Texture2D* source,
                          DXGI_FORMAT sourceFormat,
                          const UvRect& sourceRect,
                          ID3D11Texture2D* destination,
                          DXGI_FORMAT destinationFormat,
                          uint32_t destinationWidth,
                          uint32_t destinationHeight)
{
    if (!ready_ || source == nullptr || destination == nullptr)
    {
        return false;
    }

    ID3D11ShaderResourceView* sourceView = ShaderResourceViewFor(source, sourceFormat);
    ID3D11RenderTargetView* targetView = RenderTargetViewFor(destination, destinationFormat);
    if (sourceView == nullptr || targetView == nullptr)
    {
        return false;
    }

    const float rect[4] = {sourceRect.uMin, sourceRect.vMin, sourceRect.uMax, sourceRect.vMax};
    context_->UpdateSubresource(constantBuffer_, 0, nullptr, rect, 0, 0);

    D3D11_VIEWPORT viewport = {};
    viewport.Width = static_cast<float>(destinationWidth);
    viewport.Height = static_cast<float>(destinationHeight);
    viewport.MaxDepth = 1.0f;

    // Nothing else shares this context, so the previous state does not need
    // saving and restoring around the draw.
    context_->OMSetRenderTargets(1, &targetView, nullptr);
    context_->RSSetViewports(1, &viewport);
    context_->RSSetState(rasterizerState_);
    context_->OMSetDepthStencilState(depthStencilState_, 0);

    const float blendFactor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    context_->OMSetBlendState(blendState_, blendFactor, 0xffffffff);

    context_->IASetInputLayout(nullptr);
    context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    context_->VSSetShader(vertexShader_, nullptr, 0);
    context_->VSSetConstantBuffers(0, 1, &constantBuffer_);
    context_->PSSetShader(pixelShader_, nullptr, 0);
    context_->PSSetShaderResources(0, 1, &sourceView);
    context_->PSSetSamplers(0, 1, &sampler_);

    context_->Draw(3, 0);

    // Unbind the source so the same texture can be written next frame without
    // tripping a read/write hazard.
    ID3D11ShaderResourceView* nullView = nullptr;
    context_->PSSetShaderResources(0, 1, &nullView);
    ID3D11RenderTargetView* nullTarget = nullptr;
    context_->OMSetRenderTargets(1, &nullTarget, nullptr);

    return true;
}

} // namespace oxrsys
