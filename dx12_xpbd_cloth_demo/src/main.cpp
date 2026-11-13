#include <windows.h>

#include <d3d12.h>
#include <d3dcompiler.h>
#include <d3dx12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <DirectXMath.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

#include "ClothXPBD.h"

using Microsoft::WRL::ComPtr;
using namespace DirectX;

namespace
{
    constexpr UINT kFrameCount = 2;

    struct Vertex
    {
        XMFLOAT3 position;
        XMFLOAT4 color;
    };

    struct SceneConstants
    {
        XMFLOAT4X4 world;
        XMFLOAT4X4 view;
        XMFLOAT4X4 proj;
    };

    dx12xpbd::MeshData MakePlaneClothMesh(int width, int height, float spacing = 0.12f)
    {
        dx12xpbd::MeshData mesh;
        mesh.positions.reserve(static_cast<size_t>(width * height));
        mesh.indices.reserve(static_cast<size_t>((width - 1) * (height - 1) * 6));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const float px = (static_cast<float>(x) - static_cast<float>(width - 1) * 0.5f) * spacing;
                const float py = 0.0f;
                const float pz = (static_cast<float>(y) - static_cast<float>(height - 1) * 0.5f) * spacing;
                mesh.positions.push_back({px, py, pz});
            }
        }

        for (int y = 0; y < height - 1; ++y)
        {
            for (int x = 0; x < width - 1; ++x)
            {
                const uint32_t i0 = static_cast<uint32_t>(y * width + x);
                const uint32_t i1 = static_cast<uint32_t>(y * width + x + 1);
                const uint32_t i2 = static_cast<uint32_t>((y + 1) * width + x);
                const uint32_t i3 = static_cast<uint32_t>((y + 1) * width + x + 1);

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

    class ClothDemoApp
    {
    public:
        void Initialize(HINSTANCE instance, int width, int height)
        {
            instance_ = instance;
            width_ = width;
            height_ = height;

            WNDCLASSEXW wc = {};
            wc.cbSize = sizeof(wc);
            wc.style = CS_HREDRAW | CS_VREDRAW;
            wc.lpfnWndProc = WndProc;
            wc.hInstance = instance_;
            wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
            wc.lpszClassName = L"DX12XPBDClothDemo";
            RegisterClassExW(&wc);

            hwnd_ = CreateWindowExW(
                0,
                L"DX12XPBDClothDemo",
                L"DX12 XPBD Cloth",
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                width_,
                height_,
                nullptr,
                nullptr,
                instance_,
                nullptr);

            SetWindowLongPtrW(hwnd_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

            ShowWindow(hwnd_, SW_SHOW);
            UpdateWindow(hwnd_);

            CreateDevice();
            CreateCommandQueue();
            CreateSwapChain();
            CreateRenderTargets();
            CreateRootSignature();
            CreatePipelineState();
            CreateConstantBuffer();
            CreateCloth();
            CreateClothBuffers();
        }

        void Run()
        {
            MSG msg = {};
            while (true)
            {
                while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
                {
                    if (msg.message == WM_QUIT)
                    {
                        return;
                    }
                    TranslateMessage(&msg);
                    DispatchMessageW(&msg);
                }

                UpdatePhysics(1.0f / 60.0f);
                Render();
            }
        }

    private:
        HINSTANCE instance_ = nullptr;
        HWND hwnd_ = nullptr;
        int width_ = 1280;
        int height_ = 720;

        ComPtr<ID3D12Device> device_;
        ComPtr<ID3D12CommandQueue> commandQueue_;
        ComPtr<IDXGISwapChain3> swapChain_;
        ComPtr<ID3D12CommandAllocator> commandAllocator_[kFrameCount];
        ComPtr<ID3D12GraphicsCommandList> commandList_;
        ComPtr<ID3D12RootSignature> rootSignature_;
        ComPtr<ID3D12PipelineState> pipelineState_;
        ComPtr<ID3D12DescriptorHeap> rtvHeap_;
        ComPtr<ID3D12Fence> fence_;
        UINT rtvDescriptorSize_ = 0;
        UINT64 fenceValue_ = 0;
        UINT frameIndex_ = 0;

        ComPtr<ID3D12Resource> renderTargets_[kFrameCount];
        ComPtr<ID3D12Resource> clothVertexBuffer_;
        ComPtr<ID3D12Resource> clothIndexBuffer_;
        ComPtr<ID3D12Resource> constantBuffer_;
        UINT8_t* cbMapped_ = nullptr;

        D3D12_VERTEX_BUFFER_VIEW vertexBufferView_ = {};
        D3D12_INDEX_BUFFER_VIEW indexBufferView_ = {};

        dx12xpbd::ClothXPBD cloth_;
        std::vector<dx12xpbd::SphereCollider> spheres_;
        std::vector<dx12xpbd::CapsuleCollider> capsules_;

        bool dragging_ = false;
        int selectedColliderIndex_ = -1;
        bool selectedIsSphere_ = false;

        XMFLOAT3 cameraPosition_ = {0.0f, 2.8f, 8.0f};
        XMFLOAT3 cameraTarget_ = {0.0f, 0.0f, 0.0f};

        static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
        {
            auto* self = reinterpret_cast<ClothDemoApp*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
            switch (message)
            {
            case WM_DESTROY:
                PostQuitMessage(0);
                return 0;
            case WM_LBUTTONDOWN:
                if (self) self->OnMouseLeftDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                return 0;
            case WM_LBUTTONUP:
                if (self) self->OnMouseLeftUp();
                return 0;
            case WM_MOUSEMOVE:
                if (self) self->OnMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                return 0;
            case WM_KEYDOWN:
                if (self && wParam == VK_SPACE) self->ResetCloth();
                return 0;
            default:
                return DefWindowProcW(hwnd, message, wParam, lParam);
            }
        }

        void CreateDevice()
        {
            UINT dxgiFactoryFlags = 0;
#if defined(_DEBUG)
            ComPtr<ID3D12Debug> debugController;
            if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController))))
            {
                debugController->EnableDebugLayer();
                dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
            }
#endif

            ComPtr<IDXGIFactory4> factory;
            CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&factory));

            D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_12_0;
            HRESULT hr = D3D12CreateDevice(nullptr, featureLevel, IID_PPV_ARGS(&device_));
            if (FAILED(hr))
            {
                throw std::runtime_error("D3D12CreateDevice failed");
            }
        }

        void CreateCommandQueue()
        {
            D3D12_COMMAND_QUEUE_DESC qd = {};
            qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
            qd.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
            device_->CreateCommandQueue(&qd, IID_PPV_ARGS(&commandQueue_));

            for (UINT i = 0; i < kFrameCount; ++i)
            {
                device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&commandAllocator_[i]));
            }

            device_->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocator_[0].Get(), nullptr, IID_PPV_ARGS(&commandList_));
            commandList_->Close();
            device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_));
        }

        void CreateSwapChain()
        {
            ComPtr<IDXGIFactory4> factory;
            CreateDXGIFactory1(IID_PPV_ARGS(&factory));

            DXGI_SWAP_CHAIN_DESC1 sd = {};
            sd.BufferCount = kFrameCount;
            sd.Width = static_cast<UINT>(width_);
            sd.Height = static_cast<UINT>(height_);
            sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
            sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
            sd.SampleDesc.Count = 1;

            ComPtr<IDXGISwapChain1> swapChain1;
            factory->CreateSwapChainForHwnd(
                commandQueue_.Get(),
                hwnd_,
                &sd,
                nullptr,
                nullptr,
                &swapChain1);

            swapChain1.As(&swapChain_);
        }

        void CreateRenderTargets()
        {
            D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
            rtvHeapDesc.NumDescriptors = kFrameCount;
            rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
            device_->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&rtvHeap_));
            rtvDescriptorSize_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap_->GetCPUDescriptorHandleForHeapStart());
            for (UINT i = 0; i < kFrameCount; ++i)
            {
                swapChain_->GetBuffer(i, IID_PPV_ARGS(&renderTargets_[i]));
                device_->CreateRenderTargetView(renderTargets_[i].Get(), nullptr, rtvHandle);
                rtvHandle.Offset(rtvDescriptorSize_);
            }
        }

        void CreateRootSignature()
        {
            CD3DX12_ROOT_PARAMETER rootParameters[1] = {};
            rootParameters[0].InitAsConstantBufferView(0, 0, D3D12_SHADER_VISIBILITY_VERTEX);

            CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc;
            rootSigDesc.Init(_countof(rootParameters), rootParameters, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

            ComPtr<ID3DBlob> serializedSignature;
            ComPtr<ID3DBlob> serializedError;
            HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1, &serializedSignature, &serializedError);
            if (FAILED(hr))
            {
                throw std::runtime_error("D3D12SerializeRootSignature failed");
            }

            device_->CreateRootSignature(0, serializedSignature->GetBufferPointer(), serializedSignature->GetBufferSize(), IID_PPV_ARGS(&rootSignature_));
        }

        void CreatePipelineState()
        {
            static const char* shaderCode =
                "struct VSInput { float3 pos : POSITION; float4 col : COLOR; };\n"
                "struct PSInput { float4 pos : SV_POSITION; float4 col : COLOR; };\n"
                "cbuffer SceneConstants : register(b0) { float4x4 world; float4x4 view; float4x4 proj; };\n"
                "PSInput VSMain(VSInput input) {\n"
                "  PSInput output;\n"
                "  float4 worldPos = mul(float4(input.pos, 1.0f), world);\n"
                "  float4 viewPos = mul(worldPos, view);\n"
                "  output.pos = mul(viewPos, proj);\n"
                "  output.col = input.col;\n"
                "  return output;\n"
                "}\n"
                "float4 PSMain(PSInput input) : SV_TARGET { return input.col; }\n";

            ComPtr<ID3DBlob> vsBlob;
            ComPtr<ID3DBlob> psBlob;
            ComPtr<ID3DBlob> errorBlob;

            HRESULT hrVS = D3DCompile(
                shaderCode,
                strlen(shaderCode),
                nullptr,
                nullptr,
                nullptr,
                "VSMain",
                "vs_5_0",
                0,
                0,
                &vsBlob,
                &errorBlob);
            HRESULT hrPS = D3DCompile(
                shaderCode,
                strlen(shaderCode),
                nullptr,
                nullptr,
                nullptr,
                "PSMain",
                "ps_5_0",
                0,
                0,
                &psBlob,
                &errorBlob);

            if (FAILED(hrVS) || FAILED(hrPS))
            {
                if (errorBlob)
                {
                    OutputDebugStringA(reinterpret_cast<const char*>(errorBlob->GetBufferPointer()));
                }
                throw std::runtime_error("Shader compile failed");
            }

            D3D12_INPUT_ELEMENT_DESC inputLayout[] = {
                {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
                {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            };

            D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
            psoDesc.InputLayout = {inputLayout, _countof(inputLayout)};
            psoDesc.pRootSignature = rootSignature_.Get();
            psoDesc.VS = CD3DX12_SHADER_BYTECODE(vsBlob.Get());
            psoDesc.PS = CD3DX12_SHADER_BYTECODE(psBlob.Get());
            psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
            psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
            psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
            psoDesc.SampleMask = UINT_MAX;
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            psoDesc.NumRenderTargets = 1;
            psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
            psoDesc.SampleDesc.Count = 1;

            device_->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&pipelineState_));
        }

        void CreateConstantBuffer()
        {
            const UINT bufferSize = (sizeof(SceneConstants) + 255) & ~255;
            CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
            CD3DX12_RESOURCE_DESC resourceDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize);
            device_->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &resourceDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&constantBuffer_));

            constantBuffer_->Map(0, nullptr, reinterpret_cast<void**>(&cbMapped_));
        }

        void CreateCloth()
        {
            dx12xpbd::MeshData mesh = MakePlaneClothMesh(24, 24, 0.12f);
            cloth_.Initialize(mesh, 0.98f);

            spheres_.resize(2);
            spheres_[0].center = {0.0f, 1.0f, 0.0f};
            spheres_[0].radius = 0.75f;
            spheres_[1].center = {1.8f, 1.3f, 0.0f};
            spheres_[1].radius = 0.65f;

            capsules_.resize(1);
            capsules_[0].start = {-2.2f, 0.0f, 0.0f};
            capsules_[0].end = {-2.2f, 2.0f, 0.0f};
            capsules_[0].radius = 0.42f;
        }

        void ResetCloth()
        {
            dx12xpbd::MeshData mesh = MakePlaneClothMesh(24, 24, 0.12f);
            cloth_.Reset(mesh);
        }

        void BuildRenderBuffers()
        {
            const auto positions = cloth_.Positions();
            const auto triangles = cloth_.Triangles();

            std::vector<Vertex> vertices;
            vertices.reserve(positions.size());
            for (const auto& p : positions)
            {
                vertices.push_back({{p.x, p.y, p.z}, {0.28f, 0.72f, 1.0f, 1.0f}});
            }

            std::vector<uint32_t> indices;
            indices.reserve(triangles.size() * 3);
            for (const auto& tri : triangles)
            {
                indices.push_back(tri.i0);
                indices.push_back(tri.i1);
                indices.push_back(tri.i2);
            }

            const UINT vertexByteSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));
            const UINT indexByteSize = static_cast<UINT>(indices.size() * sizeof(uint32_t));

            if (!clothVertexBuffer_)
            {
                CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
                CD3DX12_RESOURCE_DESC resourceDesc = CD3DX12_RESOURCE_DESC::Buffer(vertexByteSize);
                device_->CreateCommittedResource(&heapProps, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&clothVertexBuffer_));
            }

            if (!clothIndexBuffer_)
            {
                CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
                CD3DX12_RESOURCE_DESC resourceDesc = CD3DX12_RESOURCE_DESC::Buffer(indexByteSize);
                device_->CreateCommittedResource(&heapProps, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&clothIndexBuffer_));
            }

            void* mappedVertices = nullptr;
            clothVertexBuffer_->Map(0, nullptr, &mappedVertices);
            std::memcpy(mappedVertices, vertices.data(), vertexByteSize);
            clothVertexBuffer_->Unmap(0, nullptr);

            void* mappedIndices = nullptr;
            clothIndexBuffer_->Map(0, nullptr, &mappedIndices);
            std::memcpy(mappedIndices, indices.data(), indexByteSize);
            clothIndexBuffer_->Unmap(0, nullptr);

            vertexBufferView_.BufferLocation = clothVertexBuffer_->GetGPUVirtualAddress();
            vertexBufferView_.StrideInBytes = sizeof(Vertex);
            vertexBufferView_.SizeInBytes = vertexByteSize;

            indexBufferView_.BufferLocation = clothIndexBuffer_->GetGPUVirtualAddress();
            indexBufferView_.Format = DXGI_FORMAT_R32_UINT;
            indexBufferView_.SizeInBytes = indexByteSize;
        }

        void CreateClothBuffers()
        {
            BuildRenderBuffers();
        }

        void UpdatePhysics(float dt)
        {
            cloth_.Update(dt, spheres_, capsules_, {0.0f, -9.8f, 0.0f}, 4, 0.986f);
            BuildRenderBuffers();
        }

        void OnMouseLeftDown(int x, int y)
        {
            dragging_ = TrySelectCollider(x, y);
        }

        void OnMouseLeftUp()
        {
            dragging_ = false;
            selectedColliderIndex_ = -1;
            selectedIsSphere_ = false;
        }

        void OnMouseMove(int x, int y)
        {
            if (!dragging_)
            {
                return;
            }

            const auto hit = ScreenToGround(x, y);
            if (selectedIsSphere_)
            {
                if (selectedColliderIndex_ >= 0 && selectedColliderIndex_ < static_cast<int>(spheres_.size()))
                {
                    spheres_[selectedColliderIndex_].center = {hit.x, 0.8f + spheres_[selectedColliderIndex_].radius, hit.z};
                }
            }
            else if (selectedColliderIndex_ >= 0 && selectedColliderIndex_ < static_cast<int>(capsules_.size()))
            {
                capsules_[selectedColliderIndex_].start = {hit.x, 0.0f, hit.z};
                capsules_[selectedColliderIndex_].end = {hit.x, 2.0f, hit.z};
            }
        }

        bool TrySelectCollider(int x, int y)
        {
            for (int i = 0; i < static_cast<int>(spheres_.size()); ++i)
            {
                const auto projected = ProjectPoint(spheres_[i].center);
                const float dx = static_cast<float>(x) - projected.x;
                const float dy = static_cast<float>(y) - projected.y;
                if (dx * dx + dy * dy < 400.0f)
                {
                    selectedColliderIndex_ = i;
                    selectedIsSphere_ = true;
                    return true;
                }
            }

            for (int i = 0; i < static_cast<int>(capsules_.size()); ++i)
            {
                const auto projected = ProjectPoint((capsules_[i].start + capsules_[i].end) * 0.5f);
                const float dx = static_cast<float>(x) - projected.x;
                const float dy = static_cast<float>(y) - projected.y;
                if (dx * dx + dy * dy < 420.0f)
                {
                    selectedColliderIndex_ = i;
                    selectedIsSphere_ = false;
                    return true;
                }
            }

            return false;
        }

        XMFLOAT2 ProjectPoint(const dx12xpbd::Vec3& worldPos) const
        {
            const XMVECTOR world = XMVectorSet(worldPos.x, worldPos.y, worldPos.z, 1.0f);
            const XMMATRIX view = XMMatrixLookAtLH(XMLoadFloat3(&cameraPosition_), XMLoadFloat3(&cameraTarget_), XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
            const XMMATRIX proj = XMMatrixPerspectiveFovLH(XM_PIDIV4, width_ / static_cast<float>(height_), 0.1f, 100.0f);
            const XMVECTOR clip = XMVector3Transform(world, view * proj);
            const XMVECTOR ndc = clip / XMVectorGetW(clip);
            return {
                (XMVectorGetX(ndc) * 0.5f + 0.5f) * static_cast<float>(width_),
                (1.0f - (XMVectorGetY(ndc) * 0.5f + 0.5f)) * static_cast<float>(height_)
            };
        }

        dx12xpbd::Vec3 ScreenToGround(int x, int y) const
        {
            const float ndcX = (2.0f * x / width_) - 1.0f;
            const float ndcY = 1.0f - (2.0f * y / height_);
            const float aspect = width_ / static_cast<float>(height_);
            const float fov = XM_PIDIV4;
            const float tanHalfFov = std::tan(fov * 0.5f);

            XMFLOAT3 camPos(cameraPosition_.x, cameraPosition_.y, cameraPosition_.z);
            XMFLOAT3 camTarget(cameraTarget_.x, cameraTarget_.y, cameraTarget_.z);
            XMVECTOR forward = XMVector3Normalize(XMLoadFloat3(&camTarget) - XMLoadFloat3(&camPos));
            XMVECTOR right = XMVector3Normalize(XMVector3Cross(forward, XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f)));
            XMVECTOR up = XMVector3Cross(right, forward);

            const XMVECTOR rayDir = XMVector3Normalize(
                forward +
                right * (ndcX * aspect * tanHalfFov) +
                up * (ndcY * tanHalfFov));

            XMFLOAT3 dir;
            XMStoreFloat3(&dir, rayDir);

            const float t = -cameraPosition_.y / dir.y;
            return {
                cameraPosition_.x + dir.x * t,
                0.0f,
                cameraPosition_.z + dir.z * t
            };
        }

        void Render()
        {
            frameIndex_ = swapChain_->GetCurrentBackBufferIndex();
            commandAllocator_[frameIndex_]->Reset();
            commandList_->Reset(commandAllocator_[frameIndex_].Get(), nullptr);

            D3D12_RESOURCE_BARRIER barrier = {};
            barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barrier.Transition.pResource = renderTargets_[frameIndex_].Get();
            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
            barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            commandList_->ResourceBarrier(1, &barrier);

            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap_->GetCPUDescriptorHandleForHeapStart(), frameIndex_, rtvDescriptorSize_);
            commandList_->OMSetRenderTargets(1, &rtvHandle, FALSE, nullptr);

            const float clearColor[4] = {0.05f, 0.06f, 0.10f, 1.0f};
            commandList_->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);

            XMFLOAT4X4 world, view, proj;
            XMStoreFloat4x4(&world, XMMatrixIdentity());
            XMStoreFloat4x4(&view, XMMatrixLookAtLH(XMLoadFloat3(&cameraPosition_), XMLoadFloat3(&cameraTarget_), XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f)));
            XMStoreFloat4x4(&proj, XMMatrixPerspectiveFovLH(XM_PIDIV4, width_ / static_cast<float>(height_), 0.1f, 100.0f));

            SceneConstants scene = {world, view, proj};
            memcpy(cbMapped_, &scene, sizeof(scene));

            commandList_->SetPipelineState(pipelineState_.Get());
            commandList_->SetGraphicsRootSignature(rootSignature_.Get());
            commandList_->SetGraphicsRootConstantBufferView(0, constantBuffer_->GetGPUVirtualAddress());
            commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            commandList_->IASetVertexBuffers(0, 1, &vertexBufferView_);
            commandList_->IASetIndexBuffer(&indexBufferView_);
            commandList_->DrawIndexedInstanced(static_cast<UINT>(cloth_.Triangles().size() * 3), 1, 0, 0, 0);

            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            commandList_->ResourceBarrier(1, &barrier);
            commandList_->Close();

            ID3D12CommandList* pList = commandList_.Get();
            commandQueue_->ExecuteCommandLists(1, &pList);
            swapChain_->Present(1, 0);

            UINT64 signalValue = ++fenceValue_;
            commandQueue_->Signal(fence_.Get(), signalValue);
            if (fence_->GetCompletedValue() < signalValue)
            {
                fence_->SetEventOnCompletion(signalValue, nullptr);
            }
        }
    };
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int)
{
    try
    {
        ClothDemoApp app;
        app.Initialize(hInstance, 1280, 720);
        app.Run();
    }
    catch (const std::exception& e)
    {
        OutputDebugStringA(e.what());
        return 1;
    }

    return 0;
}
