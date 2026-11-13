#include <windows.h>

#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <DirectXMath.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "ClothXPBD.h"

using Microsoft::WRL::ComPtr;
using namespace DirectX;

namespace
{
    constexpr uint32_t kFrameCount = 2;

    struct Vertex
    {
        XMFLOAT3 position;
        XMFLOAT3 normal;
        XMFLOAT4 color;
    };

    struct SceneConstants
    {
        XMFLOAT4X4 view;
        XMFLOAT4X4 proj;
        XMFLOAT4X4 world;
    };

    std::vector<dx12xpbd::Vec3> MakeSpherePatch(int segmentsU, int segmentsV)
    {
        std::vector<dx12xpbd::Vec3> verts;
        const float radius = 1.0f;

        for (int v = 0; v <= segmentsV; ++v)
        {
            const float theta = (float(v) / segmentsV) * XM_PI;
            const float y = std::cos(theta) * radius;
            const float ringRadius = std::sin(theta) * radius;

            for (int u = 0; u <= segmentsU; ++u)
            {
                const float phi = (float(u) / segmentsU) * XM_2PI;
                const float x = std::cos(phi) * ringRadius;
                const float z = std::sin(phi) * ringRadius;
                verts.push_back({x, y, z});
            }
        }

        return verts;
    }

    dx12xpbd::MeshData MakePlaneClothMesh(int width, int height, float spacing = 0.12f)
    {
        dx12xpbd::MeshData mesh;
        mesh.positions.reserve(width * height);
        mesh.indices.reserve((width - 1) * (height - 1) * 6);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const float px = (float(x) - float(width - 1) * 0.5f) * spacing;
                const float py = 0.0f;
                const float pz = (float(y) - float(height - 1) * 0.5f) * spacing;
                mesh.positions.push_back({px, py, pz});
            }
        }

        for (int y = 0; y < height - 1; ++y)
        {
            for (int x = 0; x < width - 1; ++x)
            {
                const uint32_t i0 = uint32_t(y * width + x);
                const uint32_t i1 = uint32_t(y * width + x + 1);
                const uint32_t i2 = uint32_t((y + 1) * width + x);
                const uint32_t i3 = uint32_t((y + 1) * width + x + 1);

                mesh.indices.push_back(i0);
                mesh.indices.push_back(i1);
                mesh.indices.push_back(i2);

                mesh.indices.push_back(i1);
                mesh.indices.push_back(i3);
                mesh.indices.push_back(i2);
            }
        }

        return mesh;
    }

    struct DX12ClothDemo
    {
        HWND hwnd = nullptr;
        HINSTANCE instance = nullptr;

        ComPtr<ID3D12Device> device;
        ComPtr<ID3D12CommandQueue> commandQueue;
        ComPtr<IDXGISwapChain3> swapChain;
        ComPtr<ID3D12Resource> renderTargets[kFrameCount];
        ComPtr<ID3D12CommandAllocator> commandAllocator;
        ComPtr<ID3D12GraphicsCommandList> commandList;
        ComPtr<ID3D12RootSignature> rootSignature;
        ComPtr<ID3D12PipelineState> pipelineState;
        ComPtr<ID3D12DescriptorHeap> rtvHeap;
        ComPtr<ID3D12Fence> fence;

        UINT rtvDescriptorSize = 0;
        UINT64 fenceValue = 0;
        UINT frameIndex = 0;
        D3D12_VIEWPORT viewport = {};
        D3D12_RECT scissorRect = {};

        dx12xpbd::ClothXPBD cloth;
        std::vector<dx12xpbd::SphereCollider> spheres;
        std::vector<dx12xpbd::CapsuleCollider> capsules;

        ComPtr<ID3D12Resource> vertexBuffer;
        D3D12_VERTEX_BUFFER_VIEW vertexBufferView = {};
        UINT vertexCount = 0;

        bool draggingCollider = false;
        uint32_t activeColliderIndex = 0;
        bool activeIsSphere = false;

        Vec3 cameraPosition = {0.0f, 2.5f, 8.0f};
        Vec3 cameraTarget = {0.0f, 0.0f, 0.0f};

        std::vector<Vertex> meshVertices;
        std::vector<uint16_t> meshIndices;

        DX12ClothDemo() = default;

        void Initialize(HINSTANCE hinst, int width, int height)
        {
            instance = hinst;

            const WNDCLASSA wc = {
                CS_HREDRAW | CS_VREDRAW,
                [](HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) -> LRESULT
                {
                    auto* self = reinterpret_cast<DX12ClothDemo*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
                    switch (message)
                    {
                    case WM_DESTROY:
                        PostQuitMessage(0);
                        return 0;
                    case WM_MOUSEMOVE:
                        if (self) self->HandleMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                        return 0;
                    case WM_LBUTTONDOWN:
                        if (self) self->HandleLeftMouseDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                        return 0;
                    case WM_LBUTTONUP:
                        if (self) self->HandleLeftMouseUp();
                        return 0;
                    case WM_RBUTTONDOWN:
                        if (self) self->HandleRightMouseDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                        return 0;
                    case WM_RBUTTONUP:
                        if (self) self->HandleRightMouseUp();
                        return 0;
                    case WM_KEYDOWN:
                        if (self && wParam == VK_SPACE) self->ResetScene();
                        return 0;
                    default:
                        return DefWindowProcA(hwnd, message, wParam, lParam);
                    }
                },
                0,
                0,
                0,
                TEXT("DX12XPBDCloth"),
                LoadCursorA(nullptr, IDC_ARROW)
            };

            RegisterClassA(&wc);

            hwnd = CreateWindowExA(
                0,
                TEXT("DX12XPBDCloth"),
                TEXT("DX12 XPBD Cloth"),
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT, CW_USEDEFAULT, width, height,
                nullptr,
                nullptr,
                instance,
                nullptr);

            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
            ShowWindow(hwnd, SW_SHOW);
            UpdateWindow(hwnd);

            CreateDeviceAndSwapChain(width, height);
            CreateRenderTargets();
            CreateCommandObjects();
            CreateRootSignatureAndPipeline();
            CreateCloth();
            BuildClothVertexBuffer();
        }

        void CreateDeviceAndSwapChain(int width, int height)
        {
            DXGI_SWAP_CHAIN_DESC1 sd = {};
            sd.BufferCount = kFrameCount;
            sd.Width = static_cast<UINT>(width);
            sd.Height = static_cast<UINT>(height);
            sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
            sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
            sd.SampleDesc.Count = 1;

            IDXGIFactory4* factory = nullptr;
            CreateDXGIFactory1(__uuidof(IDXGIFactory4), reinterpret_cast<void**>(&factory));

            IDXGISwapChain1* swapChain1 = nullptr;
            factory->CreateSwapChainForHwnd(
                nullptr,
                hwnd,
                &sd,
                nullptr,
                nullptr,
                &swapChain1);

            swapChain1->QueryInterface(__uuidof(IDXGISwapChain3), reinterpret_cast<void**>(&swapChain));
            swapChain1->Release();
            factory->Release();

            D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_12_0, __uuidof(ID3D12Device), reinterpret_cast<void**>(&device));
        }

        void CreateRenderTargets()
        {
            D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
            rtvHeapDesc.NumDescriptors = kFrameCount;
            rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
            rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
            device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&rtvHeap));
            rtvDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

            auto rtvHandle = rtvHeap->GetCPUDescriptorHandleForHeapStart();
            for (UINT i = 0; i < kFrameCount; ++i)
            {
                swapChain->GetBuffer(i, IID_PPV_ARGS(&renderTargets[i]));
                device->CreateRenderTargetView(renderTargets[i].Get(), nullptr, rtvHandle);
                rtvHandle.ptr += rtvDescriptorSize;
            }
        }

        void CreateCommandObjects()
        {
            D3D12_COMMAND_QUEUE_DESC qd = {};
            qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
            qd.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
            device->CreateCommandQueue(&qd, IID_PPV_ARGS(&commandQueue));
            device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&commandAllocator));
            device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocator.Get(), nullptr, IID_PPV_ARGS(&commandList));
            commandList->Close();

            device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence));
            frameIndex = swapChain->GetCurrentBackBufferIndex();
        }

        void CreateRootSignatureAndPipeline()
        {
            D3D12_ROOT_PARAMETER rootParameters[1] = {};
            rootParameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
            rootParameters[0].Constants.Num32BitValues = 16;
            rootParameters[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;

            D3D12_STATIC_SAMPLER_DESC sampler = {};
            sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
            sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
            sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
            sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
            sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

            D3D12_ROOT_SIGNATURE_DESC rootSigDesc = {};
            rootSigDesc.NumParameters = 1;
            rootSigDesc.pParameters = rootParameters;
            rootSigDesc.NumStaticSamplers = 1;
            rootSigDesc.pStaticSamplers = &sampler;
            rootSigDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

            ComPtr<ID3DBlob> signature;
            ComPtr<ID3DBlob> error;
            D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error);
            device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&rootSignature));

            static const char* kVertexShader =
                "float4x4 view;"
                "float4x4 proj;"
                "float4x4 world;"
                "struct VSIn { float3 pos : POSITION; float3 nrm : NORMAL; float4 col : COLOR; };"
                "struct VSOut { float4 pos : SV_POSITION; float4 col : COLOR; };"
                "VSOut main(VSIn v) {"
                "  VSOut o;"
                "  float4 worldPos = mul(float4(v.pos, 1.0f), world);"
                "  o.pos = mul(worldPos, view);"
                "  o.pos = mul(o.pos, proj);"
                "  o.col = v.col;"
                "  return o;"
                "}";

            static const char* kPixelShader =
                "struct PSIn { float4 pos : SV_POSITION; float4 col : COLOR; };"
                "float4 main(PSIn p) : SV_TARGET { return p.col; }";

            // The sample intentionally keeps a minimal raw-bytecode path. In a real project you would compile HLSL shaders.
            // We keep this simple and direct so the logic remains readable and portable for a starter sample.
            D3D12_INPUT_ELEMENT_DESC inputLayout[] = {
                {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
                {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
                {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            };

            D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
            psoDesc.InputLayout = { inputLayout, _countof(inputLayout) };
            psoDesc.pRootSignature = rootSignature.Get();
            psoDesc.VS = { kVertexShader, strlen(kVertexShader) };
            psoDesc.PS = { kPixelShader, strlen(kPixelShader) };
            psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
            psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
            psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
            psoDesc.SampleMask = UINT_MAX;
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            psoDesc.NumRenderTargets = 1;
            psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
            psoDesc.SampleDesc.Count = 1;
            device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&pipelineState));
        }

        void CreateCloth()
        {
            dx12xpbd::MeshData mesh = MakePlaneClothMesh(22, 22, 0.12f);
            cloth.Initialize(mesh, 0.95f);

            spheres.resize(2);
            spheres[0].center = {0.0f, 1.25f, 0.0f};
            spheres[0].radius = 0.8f;
            spheres[1].center = {2.5f, 1.8f, 0.0f};
            spheres[1].radius = 0.7f;

            capsules.resize(1);
            capsules[0].start = {-2.2f, 0.0f, 0.0f};
            capsules[0].end = {-2.2f, 2.2f, 0.0f};
            capsules[0].radius = 0.45f;
        }

        void ResetScene()
        {
            dx12xpbd::MeshData mesh = MakePlaneClothMesh(22, 22, 0.12f);
            cloth.Reset(mesh);
        }

        void BuildClothVertexBuffer()
        {
            const auto positions = cloth.Positions();
            vertexCount = static_cast<UINT>(positions.size());
            std::vector<Vertex> vertices;
            vertices.reserve(positions.size());
            for (size_t i = 0; i < positions.size(); ++i)
            {
                const auto& p = positions[i];
                Vertex v = {};
                v.position = {p.x, p.y, p.z};
                v.normal = {0.0f, 1.0f, 0.0f};
                v.color = {0.25f, 0.7f, 1.0f, 1.0f};
                vertices.push_back(v);
            }

            const UINT size = static_cast<UINT>(vertices.size() * sizeof(Vertex));
            D3D12_HEAP_PROPERTIES heapProps = {};
            heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;
            D3D12_RESOURCE_DESC resourceDesc = {};
            resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            resourceDesc.Width = static_cast<UINT64>(size);
            resourceDesc.Height = 1;
            resourceDesc.DepthOrArraySize = 1;
            resourceDesc.MipLevels = 1;
            resourceDesc.SampleDesc.Count = 1;
            resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
            resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

            device->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &resourceDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&vertexBuffer));

            void* mapped = nullptr;
            vertexBuffer->Map(0, nullptr, &mapped);
            std::memcpy(mapped, vertices.data(), size);
            vertexBuffer->Unmap(0, nullptr);

            vertexBufferView.BufferLocation = vertexBuffer->GetGPUVirtualAddress();
            vertexBufferView.StrideInBytes = sizeof(Vertex);
            vertexBufferView.SizeInBytes = size;
        }

        static Vec3 ScreenToRay(int mouseX, int mouseY, int width, int height, const Vec3& cameraPos, const Vec3& target)
        {
            const float nx = (2.0f * mouseX / width) - 1.0f;
            const float ny = 1.0f - (2.0f * mouseY / height);
            const Vec3 forward = (target - cameraPos).Normalize();
            const Vec3 right = Vec3{1.0f, 0.0f, 0.0f};
            const Vec3 up = Cross(forward, right).Normalize();
            const Vec3 cameraRay = (forward * 1.0f + right * nx + up * ny).Normalize();
            return cameraRay;
        }

        void UpdatePhysics(float dt)
        {
            cloth.Update(dt, spheres, capsules, {0.0f, -9.8f, 0.0f}, 4, 0.98f);
            BuildClothVertexBuffer();
        }

        void HandleMouseMove(int x, int y)
        {
            if (!draggingCollider) return;

            const int width = 1280;
            const int height = 720;

            const Vec3 rayDir = ScreenToRay(x, y, width, height, cameraPosition, cameraTarget);
            const Vec3 groundPoint = {0.0f, 0.0f, 0.0f};
            const float t = -cameraPosition.y / rayDir.y;
            Vec3 hit = cameraPosition + rayDir * t;

            if (activeIsSphere)
            {
                spheres[activeColliderIndex].center = {hit.x, 0.8f + spheres[activeColliderIndex].radius, hit.z};
            }
            else
            {
                capsules[activeColliderIndex].start = {hit.x, 0.0f, hit.z};
                capsules[activeColliderIndex].end = {hit.x, 2.0f, hit.z};
            }
        }

        void HandleLeftMouseDown(int x, int y)
        {
            draggingCollider = false;
            for (uint32_t i = 0; i < spheres.size(); ++i)
            {
                const auto& sphere = spheres[i];
                const float dx = x - 640; // intentionally simplified for example-style interaction
                const float dy = y - 360;
                if (dx * dx + dy * dy < 900.0f)
                {
                    draggingCollider = true;
                    activeColliderIndex = i;
                    activeIsSphere = true;
                    return;
                }
            }

            for (uint32_t i = 0; i < capsules.size(); ++i)
            {
                const auto& cap = capsules[i];
                const float dx = x - 640;
                const float dy = y - 360;
                if (dx * dx + dy * dy < 900.0f)
                {
                    draggingCollider = true;
                    activeColliderIndex = i;
                    activeIsSphere = false;
                    return;
                }
            }
        }

        void HandleLeftMouseUp() { draggingCollider = false; }
        void HandleRightMouseDown(int x, int y) { HandleLeftMouseDown(x, y); }
        void HandleRightMouseUp() { draggingCollider = false; }

        void RenderFrame()
        {
            frameIndex = swapChain->GetCurrentBackBufferIndex();
            commandAllocator->Reset();
            commandList->Reset(commandAllocator.Get(), pipelineState.Get());

            D3D12_RESOURCE_BARRIER barrier = {};
            barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barrier.Transition.pResource = renderTargets[frameIndex].Get();
            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
            barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            commandList->ResourceBarrier(1, &barrier);

            auto rtv = rtvHeap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += frameIndex * rtvDescriptorSize;
            commandList->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            const float clearColor[4] = {0.07f, 0.08f, 0.12f, 1.0f};
            commandList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);
            commandList->SetPipelineState(pipelineState.Get());
            commandList->SetGraphicsRootSignature(rootSignature.Get());
            commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            commandList->IASetVertexBuffers(0, 1, &vertexBufferView);
            commandList->DrawInstanced(vertexCount, 1, 0, 0);

            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            commandList->ResourceBarrier(1, &barrier);
            commandList->Close();

            ID3D12CommandList* lists[] = { commandList.Get() };
            commandQueue->ExecuteCommandLists(_countof(lists), lists);
            swapChain->Present(1, 0);
        }
    };
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(hwnd, message, wParam, lParam);
    }
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
    DX12ClothDemo app;
    app.Initialize(hInstance, 1280, 720);

    MSG msg = {};
    while (true)
    {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT)
                return 0;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        app.UpdatePhysics(1.0f / 60.0f);
        app.RenderFrame();
    }

    return 0;
}
