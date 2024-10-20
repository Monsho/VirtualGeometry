#include "sample_application.h"

#include "sl12/resource_mesh.h"
#include "sl12/string_util.h"
#include "sl12/descriptor_set.h"
#include "sl12/resource_texture.h"

#define NOMINMAX
#include <windowsx.h>

#include "sl12/root_signature.h"

#define USE_IN_CPP
#include <execution>

#include "../shaders/cbuffer.hlsli"

namespace
{
	static const char* kResourceDir = "resources";
	static const char* kShaderDir = "VirtualGeometry/shaders";
	static const sl12::u32 kIndirectStride = 4 + sizeof(D3D12_DRAW_INDEXED_ARGUMENTS);

	static std::vector<sl12::RenderGraphTargetDesc> gGBufferDescs;
	static sl12::RenderGraphTargetDesc gAccumDesc;
	void SetGBufferDesc(sl12::u32 width, sl12::u32 height)
	{
		gGBufferDescs.clear();
		
		sl12::RenderGraphTargetDesc desc{};
		desc.name = "GBufferA";
		desc.width = width;
		desc.height = height;
		desc.format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
		desc.srvDescs.push_back(sl12::RenderGraphSRVDesc(0, 0, 0, 0));
		desc.rtvDescs.push_back(sl12::RenderGraphRTVDesc(0, 0, 0));
		gGBufferDescs.push_back(desc);

		desc.name = "GBufferB";
		desc.format = DXGI_FORMAT_R8G8B8A8_UNORM;
		gGBufferDescs.push_back(desc);

		desc.name = "GBufferC";
		desc.format = DXGI_FORMAT_R10G10B10A2_UNORM;
		gGBufferDescs.push_back(desc);

		desc.name = "Depth";
		desc.format = DXGI_FORMAT_D32_FLOAT;
		desc.clearDepth = 1.0f;
		desc.rtvDescs.clear();
		desc.dsvDescs.push_back(sl12::RenderGraphDSVDesc(0, 0, 0));
		desc.usage = sl12::ResourceUsage::ShaderResource | sl12::ResourceUsage::DepthStencil;
		gGBufferDescs.push_back(desc);

		gAccumDesc.name = "Accum";
		gAccumDesc.width = width;
		gAccumDesc.height = height;
		gAccumDesc.format = DXGI_FORMAT_R11G11B10_FLOAT;
		gAccumDesc.usage = sl12::ResourceUsage::ShaderResource | sl12::ResourceUsage::UnorderedAccess;
		gAccumDesc.srvDescs.push_back(sl12::RenderGraphSRVDesc(0, 0, 0, 0));
		gAccumDesc.uavDescs.push_back(sl12::RenderGraphUAVDesc(0, 0, 0));
	}
}

SampleApplication::SampleApplication(HINSTANCE hInstance, int nCmdShow, int screenWidth, int screenHeight, sl12::ColorSpaceType csType, const std::string& homeDir)
	: Application(hInstance, nCmdShow, screenWidth, screenHeight, csType)
	, displayWidth_(screenWidth), displayHeight_(screenHeight)
{
	std::filesystem::path p(homeDir);
	p = std::filesystem::absolute(p);
	homeDir_ = p.string();
}

SampleApplication::~SampleApplication()
{}

bool SampleApplication::Initialize()
{
	// initialize mesh manager.
	const size_t kVertexBufferSize = 512 * 1024 * 1024;		// 512MB
	const size_t kIndexBufferSize = 64 * 1024 * 1024;		// 64MB
	meshMan_ = sl12::MakeUnique<sl12::MeshManager>(&device_, &device_, kVertexBufferSize, kIndexBufferSize);
	
	// initialize resource loader.
	resLoader_ = sl12::MakeUnique<sl12::ResourceLoader>(nullptr);
	if (!resLoader_->Initialize(&device_, &meshMan_, sl12::JoinPath(homeDir_, kResourceDir)))
	{
		sl12::ConsolePrint("Error: failed to init resource loader.");
		return false;
	}

	// initialize shader manager.
	std::vector<std::string> shaderIncludeDirs;
	shaderIncludeDirs.push_back(sl12::JoinPath(homeDir_, "../SampleLib12/SampleLib12/shaders/include"));
	shaderMan_ = sl12::MakeUnique<sl12::ShaderManager>(nullptr);
	if (!shaderMan_->Initialize(&device_, &shaderIncludeDirs))
	{
		sl12::ConsolePrint("Error: failed to init shader manager.");
		return false;
	}

	// compile shaders.
	const std::string shaderBaseDir = sl12::JoinPath(homeDir_, kShaderDir);
	std::vector<sl12::ShaderDefine> shaderDefines;
	shaderDefines.push_back(sl12::ShaderDefine("ENABLE_DYNAMIC_RESOURCE", "0"));
	hMeshVV_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "mesh.vv.hlsl"),
		"main", sl12::ShaderType::Vertex, 6, 6, nullptr, &shaderDefines);
	hMeshP_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "mesh.p.hlsl"),
		"main", sl12::ShaderType::Pixel, 6, 6, nullptr, &shaderDefines);
	hLightingC_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "lighting.c.hlsl"),
		"main", sl12::ShaderType::Compute, 6, 6, nullptr, &shaderDefines);
	hFullscreenVV_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "fullscreen.vv.hlsl"),
		"main", sl12::ShaderType::Vertex, 6, 6, nullptr, &shaderDefines);
	hTonemapP_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "tonemap.p.hlsl"),
		"main", sl12::ShaderType::Pixel, 6, 6, nullptr, &shaderDefines);
	hClearCountC_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "traverse_tree.lib.hlsl"),
		"ClearCountCS", sl12::ShaderType::Compute, 6, 8, nullptr, &shaderDefines);
	hTraverseLib_ = shaderMan_->CompileFromFile(
		sl12::JoinPath(shaderBaseDir, "traverse_tree.lib.hlsl"),
		"main", sl12::ShaderType::Library, 6, 8, nullptr, &shaderDefines);

	// load glb and mesh_grouping.
	std::string glbPath = sl12::JoinPath(sl12::JoinPath(homeDir_, kResourceDir), "bunny.glb");
	myModel_ = LoadGLTFModel(glbPath.c_str());
	ProcessModel(myModel_);

	// create vertex and index buffer.
	myPositionBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	myNormalBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	baseIndexBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	meshletIndexBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	{
		sl12::BufferDesc desc;
		desc.heap = sl12::BufferHeap::Dynamic;
		desc.size = sizeof(Microsoft::glTF::Vector3) * myModel_->meshes[0].positions.size();
		desc.stride = sizeof(Microsoft::glTF::Vector3);
		desc.usage = sl12::ResourceUsage::VertexBuffer;
		desc.initialState = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;
		myPositionBuffer_->Initialize(&device_, desc);
		myNormalBuffer_->Initialize(&device_, desc);

		void* p = myPositionBuffer_->Map();
		memcpy(p, myModel_->meshes[0].positions.data(), desc.size);
		myPositionBuffer_->Unmap();
		void* n = myNormalBuffer_->Map();
		memcpy(n, myModel_->meshes[0].normals.data(), desc.size);
		myNormalBuffer_->Unmap();
	}
	{
		sl12::BufferDesc desc;
		desc.heap = sl12::BufferHeap::Dynamic;
		desc.size = sizeof(sl12::u32) * myModel_->meshes[0].indices.size();
		desc.stride = sizeof(sl12::u32);
		desc.usage = sl12::ResourceUsage::IndexBuffer;
		desc.initialState = D3D12_RESOURCE_STATE_INDEX_BUFFER;
		baseIndexBuffer_->Initialize(&device_, desc);

		desc.size = sizeof(sl12::u32) * myModel_->meshes[0].meshletIndices.size();
		meshletIndexBuffer_->Initialize(&device_, desc);

		sl12::u32* i = (sl12::u32*)baseIndexBuffer_->Map();
		size_t count = 0;
		for (auto&& meshlet : myModel_->meshes[0].meshletsLOD0)
		{
			memcpy(i, meshlet.indices.data(), sizeof(sl12::u32) * meshlet.indices.size());
			i += meshlet.indices.size();
			count += meshlet.indices.size();
		}
		assert(count == myModel_->meshes[0].indices.size());
		baseIndexBuffer_->Unmap();

		i = (sl12::u32*)meshletIndexBuffer_->Map();
		memcpy(i, myModel_->meshes[0].meshletIndices.data(), sizeof(sl12::u32) * myModel_->meshes[0].meshletIndices.size());
		meshletIndexBuffer_->Unmap();
	}

	// init command list.
	mainCmdList_ = sl12::MakeUnique<CommandLists>(nullptr);
	if (!mainCmdList_->Initialize(&device_, &device_.GetGraphicsQueue()))
	{
		sl12::ConsolePrint("Error: failed to init main command list.");
		return false;
	}

	// init cbv manager.
	cbvMan_ = sl12::MakeUnique<sl12::CbvManager>(nullptr, &device_);

	// init render graph.
	renderGraph_ = sl12::MakeUnique<sl12::RenderGraph>(nullptr);

	// get GBuffer target descs.
	SetGBufferDesc(displayWidth_, displayHeight_);
	
	// create sampler.
	{
		linearSampler_ = sl12::MakeUnique<sl12::Sampler>(&device_);

		D3D12_SAMPLER_DESC desc{};
		desc.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
		desc.AddressU = desc.AddressV = desc.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
		desc.MaxLOD = FLT_MAX;
		desc.MinLOD = 0.0f;
		desc.MipLODBias = 0.0f;
		linearSampler_->Initialize(&device_, desc);
	}
	
	// create depth stencil buffer.
	{
		depthTex_ = sl12::MakeUnique<sl12::Texture>(&device_);
		sl12::TextureDesc desc{};
		desc.width = displayWidth_;
		desc.height = displayHeight_;
		desc.depth = 1;
		desc.format = DXGI_FORMAT_D32_FLOAT;
		desc.initialState = D3D12_RESOURCE_STATE_DEPTH_WRITE;
		desc.usage = sl12::ResourceUsage::DepthStencil;

		depthTex_->Initialize(&device_, desc);

		depthDSV_ = sl12::MakeUnique<sl12::DepthStencilView>(&device_);
		depthDSV_->Initialize(&device_, &depthTex_);
	}

	// init utility command list.
	auto utilCmdList = sl12::MakeUnique<sl12::CommandList>(&device_);
	utilCmdList->Initialize(&device_, &device_.GetGraphicsQueue());
	utilCmdList->Reset();

	// init GUI.
	gui_ = sl12::MakeUnique<sl12::Gui>(nullptr);
	if (!gui_->Initialize(&device_, device_.GetSwapchain().GetTexture(0)->GetResourceDesc().Format))
	{
		sl12::ConsolePrint("Error: failed to init GUI.");
		return false;
	}
	if (!gui_->CreateFontImage(&device_, &utilCmdList))
	{
		sl12::ConsolePrint("Error: failed to create GUI font.");
		return false;
	}

	// create dummy texture.
	if (!device_.CreateDummyTextures(&utilCmdList))
	{
		return false;
	}

	// buffers used with work graph.
	meshletBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	countBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	drawArgBuffer_ = sl12::MakeUnique<sl12::Buffer>(&device_);
	meshletBV_ = sl12::MakeUnique<sl12::BufferView>(&device_);
	countUAV_ = sl12::MakeUnique<sl12::UnorderedAccessView>(&device_);
	drawArgUAV_ = sl12::MakeUnique<sl12::UnorderedAccessView>(&device_);
	{
		sl12::BufferDesc desc{};
		desc.heap = sl12::BufferHeap::Default;
		desc.usage = sl12::ResourceUsage::ShaderResource;
		desc.stride = sizeof(MeshletData);
		desc.size = desc.stride * myModel_->meshes[0].meshletMap.size();
		desc.initialState = D3D12_RESOURCE_STATE_COMMON;
		if (!meshletBuffer_->Initialize(&device_, desc))
		{
			return false;
		}

		UniqueHandle<sl12::Buffer> copySrc = sl12::MakeUnique<sl12::Buffer>(&device_);
		desc.heap = sl12::BufferHeap::Dynamic;
		if (!copySrc->Initialize(&device_, desc))
		{
			return false;
		}

		MeshletData* data = static_cast<MeshletData*>(copySrc->Map());
		auto& mesh = myModel_->meshes[0];
		for (sl12::u32 i = 0; i < mesh.meshletMap.size(); i++, data++)
		{
			auto it = mesh.meshletMap.find(i);
			if (it != mesh.meshletMap.end())
			{
				data->groupSphereCenter = DirectX::XMFLOAT3(it->second.groupBounds.sphereCenter.x, it->second.groupBounds.sphereCenter.y, it->second.groupBounds.sphereCenter.z);
				data->parentSphereCenter = DirectX::XMFLOAT3(it->second.parentBounds.sphereCenter.x, it->second.parentBounds.sphereCenter.y, it->second.parentBounds.sphereCenter.z);
				data->groupError = it->second.groupError;
				data->parentError = it->second.parentError;
				data->indexCount = (UINT)it->second.indices.size();
				data->indexOffset = (UINT)it->second.indexBufferOffset;
				data->childCount = (UINT)it->second.traverseChildren.size();
				for (sl12::u32 c = 0; c < data->childCount; c++)
				{
					data->children[c] = it->second.traverseChildren[c];
				}
			}
			else
			{
				sl12::ConsolePrint("don't find meshlet (id : %d)\n", i);
			}
		}
		copySrc->Unmap();

		utilCmdList->GetLatestCommandList()->CopyResource(meshletBuffer_->GetResourceDep(), copySrc->GetResourceDep());
		utilCmdList->TransitionBarrier(&meshletBuffer_, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);

		if (!meshletBV_->Initialize(&device_, &meshletBuffer_, 0, 0, sizeof(MeshletData)))
		{
			return false;
		}
	}
	{
		sl12::BufferDesc desc{};
		desc.heap = sl12::BufferHeap::Default;
		desc.usage = sl12::ResourceUsage::UnorderedAccess;
		desc.stride = 0;
		desc.size = sizeof(sl12::u32);
		desc.initialState = D3D12_RESOURCE_STATE_COMMON;
		if (!countBuffer_->Initialize(&device_, desc))
		{
			return false;
		}
		if (!countUAV_->Initialize(&device_, &countBuffer_, 0, 0, 0, 0))
		{
			return false;
		}

		desc.size = kIndirectStride * myModel_->meshes[0].meshletsLOD0.size();
		if (!drawArgBuffer_->Initialize(&device_, desc))
		{
			return false;
		}
		if (!drawArgUAV_->Initialize(&device_, &drawArgBuffer_, 0, 0, 0, 0))
		{
			return false;
		}
	}
	
	// execute utility commands.
	utilCmdList->Close();
	utilCmdList->Execute();
	device_.WaitDrawDone();

	// wait compile and load.
	while (shaderMan_->IsCompiling() || resLoader_->IsLoading())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// setup texture streamer.
	texStreamer_ = sl12::MakeUnique<sl12::TextureStreamer>(&device_);
	if (!texStreamer_->Initialize(&device_))
	{
		return false;
	}
	
	// init root signature and pipeline state.
	rsMesh_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	rsVsPs_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	rsMeshDR_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	rsTonemapDR_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	psoMesh_ = sl12::MakeUnique<sl12::GraphicsPipelineState>(&device_);
	psoTonemap_ = sl12::MakeUnique<sl12::GraphicsPipelineState>(&device_);
	rsMesh_->Initialize(&device_, hMeshVV_.GetShader(), hMeshP_.GetShader(), nullptr, nullptr, nullptr, 1);
	rsVsPs_->Initialize(&device_, hMeshVV_.GetShader(), hMeshP_.GetShader(), nullptr, nullptr, nullptr);
	{
		sl12::GraphicsPipelineStateDesc desc{};
		desc.pRootSignature = &rsMesh_;
		desc.pVS = hMeshVV_.GetShader();
		desc.pPS = hMeshP_.GetShader();

		desc.blend.sampleMask = UINT_MAX;
		desc.blend.rtDesc[0].isBlendEnable = false;
		desc.blend.rtDesc[0].writeMask = 0xf;

		desc.rasterizer.cullMode = D3D12_CULL_MODE_BACK;
		desc.rasterizer.fillMode = D3D12_FILL_MODE_SOLID;
		desc.rasterizer.isDepthClipEnable = true;
		desc.rasterizer.isFrontCCW = true;

		desc.depthStencil.isDepthEnable = true;
		desc.depthStencil.isDepthWriteEnable = true;
		desc.depthStencil.depthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;

		D3D12_INPUT_ELEMENT_DESC input_elems[] = {
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
			{"NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 1, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
		};
		desc.inputLayout.numElements = ARRAYSIZE(input_elems);
		desc.inputLayout.pElements = input_elems;

		desc.primTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		desc.numRTVs = 0;
		desc.rtvFormats[desc.numRTVs++] = gGBufferDescs[0].format;
		desc.rtvFormats[desc.numRTVs++] = gGBufferDescs[1].format;
		desc.rtvFormats[desc.numRTVs++] = gGBufferDescs[2].format;
		desc.dsvFormat = gGBufferDescs[3].format;
		desc.multisampleCount = 1;

		if (!psoMesh_->Initialize(&device_, desc))
		{
			sl12::ConsolePrint("Error: failed to init mesh pso.");
			return false;
		}
	}
	{
		sl12::GraphicsPipelineStateDesc desc{};
		desc.pRootSignature = &rsVsPs_;
		desc.pVS = hFullscreenVV_.GetShader();
		desc.pPS = hTonemapP_.GetShader();

		desc.blend.sampleMask = UINT_MAX;
		desc.blend.rtDesc[0].isBlendEnable = false;
		desc.blend.rtDesc[0].writeMask = 0xf;

		desc.rasterizer.cullMode = D3D12_CULL_MODE_NONE;
		desc.rasterizer.fillMode = D3D12_FILL_MODE_SOLID;
		desc.rasterizer.isDepthClipEnable = true;
		desc.rasterizer.isFrontCCW = true;

		desc.depthStencil.isDepthEnable = false;
		desc.depthStencil.isDepthWriteEnable = false;

		desc.primTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		desc.numRTVs = 0;
		desc.rtvFormats[desc.numRTVs++] = device_.GetSwapchain().GetTexture(0)->GetResourceDesc().Format;
		desc.dsvFormat = DXGI_FORMAT_UNKNOWN;
		desc.multisampleCount = 1;

		if (!psoTonemap_->Initialize(&device_, desc))
		{
			sl12::ConsolePrint("Error: failed to init tonemap pso.");
			return false;
		}
	}

	rsCs_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	rsLightingDR_ = sl12::MakeUnique<sl12::RootSignature>(&device_);
	psoLighting_ = sl12::MakeUnique<sl12::ComputePipelineState>(&device_);
	psoClearCount_ = sl12::MakeUnique<sl12::ComputePipelineState>(&device_);
	rsCs_->Initialize(&device_, hLightingC_.GetShader());
	{
		sl12::ComputePipelineStateDesc desc{};
		desc.pCS = hLightingC_.GetShader();
		desc.pRootSignature = &rsCs_;

		if (!psoLighting_->Initialize(&device_, desc))
		{
			sl12::ConsolePrint("Error: failed to init lighting pso.");
			return false;
		}
	}
	{
		sl12::ComputePipelineStateDesc desc{};
		desc.pCS = hClearCountC_.GetShader();
		desc.pRootSignature = &rsCs_;

		if (!psoClearCount_->Initialize(&device_, desc))
		{
			sl12::ConsolePrint("Error: failed to init clear count pso.");
			return false;
		}
	}

	// work graph context.
	traverseWGState_ = sl12::MakeUnique<sl12::WorkGraphState>(&device_);
	traverseWGContext_ = sl12::MakeUnique<sl12::WorkGraphContext>(&device_);
	{
		static LPCWSTR kProgramName = L"TraverseWG";
		static LPCWSTR kEntryPoint = L"RootNode";

		D3D12_NODE_ID entryPoint{};
		entryPoint.Name = kEntryPoint;
		entryPoint.ArrayIndex = 0;

		sl12::WorkGraphStateDesc desc;
		desc.AddDxilLibrary(hTraverseLib_.GetShader()->GetData(), hTraverseLib_.GetShader()->GetSize(), nullptr, 0);
		desc.AddWorkGraph(kProgramName, D3D12_WORK_GRAPH_FLAG_INCLUDE_ALL_AVAILABLE_NODES, 1, &entryPoint);
		desc.AddGlobalRootSignature(*&rsCs_);

		if (!traverseWGState_->Initialize(&device_, desc))
		{
			sl12::ConsolePrint("Error: failed to init work graph state.");
			return false;
		}
		
		if (!traverseWGContext_->Initialize(&device_, &traverseWGState_, kProgramName))
		{
			sl12::ConsolePrint("Error: failed to init work graph context.");
			return false;
		}
	}

	meshletIndirect_ = sl12::MakeUnique<sl12::IndirectExecuter>(&device_);
	if (!meshletIndirect_->InitializeWithConstants(&device_, sl12::IndirectType::DrawIndexed, kIndirectStride, &rsMesh_))
	{
		return false;
	}

	return true;
}

void SampleApplication::Finalize()
{
	// wait render.
	device_.WaitDrawDone();
	device_.Present(1);

	// destroy render objects.
	meshletIndirect_.Reset();
	traverseWGContext_.Reset();
	traverseWGState_.Reset();
	gui_.Reset();
	psoLighting_.Reset();
	rsCs_.Reset();
	psoTonemap_.Reset();
	psoMesh_.Reset();
	rsLightingDR_.Reset();
	rsTonemapDR_.Reset();
	rsMeshDR_.Reset();
	rsVsPs_.Reset();
	texStreamer_.Reset();
	depthTex_.Reset();
	depthDSV_.Reset();
	renderGraph_.Reset();
	cbvMan_.Reset();
	mainCmdList_.Reset();
	shaderMan_.Reset();
	resLoader_.Reset();
}

void SampleApplication::ControlCamera()
{
	if (GetKeyState('W') < 0)
	{
		cameraAngleX_ = std::min(cameraAngleX_ + 1.0f, 85.0f);
	}
	else if (GetKeyState('S') < 0)
	{
		cameraAngleX_ = std::max(cameraAngleX_ - 1.0f, -85.0f);
	}
	if (GetKeyState('A') < 0)
	{
		cameraAngleY_ += 1.0f;
	}
	else if (GetKeyState('D') < 0)
	{
		cameraAngleY_ -= 1.0f;
	}
	if (GetKeyState('K') < 0)
	{
		cameraLength_ = std::min(cameraLength_ + 0.1f, 100.0f);
	}
	else if (GetKeyState('I') < 0)
	{
		cameraLength_ = std::max(cameraLength_ - 0.1f, 1.0f);
	}
}

void SampleApplication::EnumerateVisibleMeshlets(const MyMesh* InMyMesh, const DirectX::XMMATRIX& InMtxWorldToView, const DirectX::XMMATRIX& InMtxLocalToWorld, float InFovYRad, float InScreenHeight, bool isParallelTraverse, std::set<uint32>& OutMeshletIDs)
{
	OutMeshletIDs.clear();
	DirectX::XMMATRIX mtxLocalToView = InMtxLocalToWorld * InMtxWorldToView;
	DirectX::XMVECTOR scale, rot, trans;
	DirectX::XMMatrixDecompose(&scale, &rot, &trans, InMtxLocalToWorld);
	//DirectX::XMMatrixDecompose(&scale, &rot, &trans, mtxLocalToView);
	DirectX::XMFLOAT3 fs3;
	DirectX::XMStoreFloat3(&fs3, scale);
	float maxScale = std::max(fs3.x, std::max(fs3.y, fs3.z));

	auto ErrorProjection = [&](const DirectX::XMFLOAT3& center, float error)
	{
		float yScale = 1.0f / tanf(InFovYRad * 0.5f);
		float d2 = (center.x * center.x) + (center.y * center.y) + (center.z * center.z);
		return InScreenHeight * 0.5f * yScale * error / (d2 - error * error);
		//return InScreenHeight * 0.5f * yScale * radius / sqrtf(d2 - radius * radius);
	};
	auto IsMeshletVisible = [&](const MyMeshlet* InMyMeshlet)
	{
		float ownError = InMyMeshlet->groupError * maxScale;
		DirectX::XMFLOAT3 ownCenter(InMyMeshlet->groupBounds.sphereCenter.x, InMyMeshlet->groupBounds.sphereCenter.y, InMyMeshlet->groupBounds.sphereCenter.z);
		DirectX::XMVECTOR tempVec = DirectX::XMLoadFloat3(&ownCenter);
		tempVec = DirectX::XMVector3Transform(tempVec, mtxLocalToView);
		DirectX::XMStoreFloat3(&ownCenter, tempVec);
		ownError = ErrorProjection(ownCenter, std::max(ownError, 1e-8f));

		float parentError = InMyMeshlet->parentError * maxScale;
		DirectX::XMFLOAT3 parentCenter(InMyMeshlet->parentBounds.sphereCenter.x, InMyMeshlet->parentBounds.sphereCenter.y, InMyMeshlet->parentBounds.sphereCenter.z);
		tempVec = DirectX::XMLoadFloat3(&parentCenter);
		tempVec = DirectX::XMVector3Transform(tempVec, mtxLocalToView);
		DirectX::XMStoreFloat3(&parentCenter, tempVec);
		parentError = ErrorProjection(parentCenter, std::max(parentError, 1e-8f));

		bool isVisible = ownError <= errorThreshold_ && errorThreshold_ < parentError;
		if (isVisible && bPrintError_)
		{
			sl12::ConsolePrint("LOD %d: own(%f) parent(%f)\n", InMyMeshlet->lod, ownError, parentError);
		}
		return isVisible;
	};

	sl12::CpuTimer start = sl12::CpuTimer::CurrentTime();
	// traverse meshlet tree.
	std::vector<uint32> queue;
	queue.push_back(InMyMesh->rootMeshletID);
	if (!isParallelTraverse)
	{
		for (size_t i = 0; i < queue.size(); i++)
		{
			uint32 meshletID = queue[i];
			auto&& myMeshlet = InMyMesh->meshletMap.find(meshletID)->second;
			bool isVisible = IsMeshletVisible(&myMeshlet);
			if (isVisible)
			{
				OutMeshletIDs.insert(meshletID);
				continue;
			}
			for (auto child : myMeshlet.traverseChildren)
			{
				queue.push_back(child);
			}
		}
	}
	else
	{
		struct SelectionWork
		{
			std::vector<sl12::u32> inIDs;
			std::vector<sl12::u32> outIDs;
		};
		static const sl12::u32 kCountPerThreads = 64;
		sl12::u32 numThreads = (sl12::u32)((InMyMesh->meshletMap.size() + kCountPerThreads - 1) / kCountPerThreads);
		std::vector<SelectionWork> workData;
		workData.resize(numThreads);
		for (sl12::u32 t = 0; t < numThreads; t++)
		{
			for (sl12::u32 i = 0; i < kCountPerThreads; i++)
			{
				if (t * kCountPerThreads + i >= InMyMesh->meshletMap.size())
					break;
				workData[t].inIDs.push_back(t * kCountPerThreads + i);
			}
			workData[t].outIDs.clear();
		}
		std::for_each(std::execution::par, workData.begin(), workData.end(), [&](SelectionWork& work)
		{
			for (sl12::u32 id : work.inIDs)
			{
				auto&& myMeshlet = InMyMesh->meshletMap.find(id)->second;
				bool isVisible = IsMeshletVisible(&myMeshlet);
				if (isVisible)
				{
					work.outIDs.push_back(id);
				}
			}
		});
		for (auto&& work : workData)
		{
			OutMeshletIDs.insert(work.outIDs.begin(), work.outIDs.end());
		}
	}
	traverseTime_ = sl12::CpuTimer::CurrentTime() - start;

	bPrintError_ = false;
}

bool SampleApplication::Execute()
{
	const int kSwapchainBufferOffset = 1;
	auto frameIndex = (device_.GetSwapchain().GetFrameIndex() + sl12::Swapchain::kMaxBuffer - 1) % sl12::Swapchain::kMaxBuffer;
	auto prevFrameIndex = (device_.GetSwapchain().GetFrameIndex() + sl12::Swapchain::kMaxBuffer - 2) % sl12::Swapchain::kMaxBuffer;
	auto pCmdList = &mainCmdList_->Reset();

	bool bStream = false;
	static sl12::u32 texTargetWidth = 256;

	ControlCamera();
	gui_->BeginNewFrame(pCmdList, displayWidth_, displayHeight_, inputData_);
	inputData_.Reset();
	{
		ImGui::Checkbox("GroupColor", &bRenderGroupColor_);
		ImGui::Checkbox("RenderLOD", &bRenderLOD_);
		if (bRenderLOD_)
		{
			static const char* kTraverseTypes[] = {
				"CPU Linear",
				"CPU Parallel",
				"GPU Work Graph"
			};
			ImGui::Combo("Traverse Type", &traverseType_, kTraverseTypes, 3);
			ImGui::Checkbox("Print Error", &bPrintError_);
			ImGui::SliderFloat("Error Threshold", &errorThreshold_, 0.001f, 1.0f);
			ImGui::Text("Traverse Time : %.3f (ms)", traverseTime_.ToMicroSecond() / 1000.0f);
		}
	}
	ImGui::Render();

	device_.WaitPresent();
	device_.SyncKillObjects();

	device_.LoadRenderCommands(pCmdList);
	meshMan_->BeginNewFrame(pCmdList);
	cbvMan_->BeginNewFrame();
	renderGraph_->BeginNewFrame();

	// texture streaming request.
	if (bStream)
	{
		for (auto&& work : workMaterials_)
		{
			for (auto&& handle : work.texHandles)
			{
				texStreamer_->RequestStreaming(handle, texTargetWidth);
			}
		}
	}
	
	// create targets.
	std::vector<sl12::RenderGraphTargetID> gbufferTargetIDs;
	sl12::RenderGraphTargetID accumTargetID;
	for (auto&& desc : gGBufferDescs)
	{
		gbufferTargetIDs.push_back(renderGraph_->AddTarget(desc));
	}
	accumTargetID = renderGraph_->AddTarget(gAccumDesc);

	// create render passes.
	{
		std::vector<sl12::RenderPass> passes;
		std::vector<sl12::RenderGraphTargetID> histories;
		std::vector<sl12::RenderGraphTargetID> return_histories;
		
		sl12::RenderPass gbufferPass{};
		gbufferPass.output.push_back(gbufferTargetIDs[0]);
		gbufferPass.output.push_back(gbufferTargetIDs[1]);
		gbufferPass.output.push_back(gbufferTargetIDs[2]);
		gbufferPass.output.push_back(gbufferTargetIDs[3]);
		gbufferPass.outputStates.push_back(D3D12_RESOURCE_STATE_RENDER_TARGET);
		gbufferPass.outputStates.push_back(D3D12_RESOURCE_STATE_RENDER_TARGET);
		gbufferPass.outputStates.push_back(D3D12_RESOURCE_STATE_RENDER_TARGET);
		gbufferPass.outputStates.push_back(D3D12_RESOURCE_STATE_DEPTH_WRITE);
		passes.push_back(gbufferPass);

		sl12::RenderPass lightingPass{};
		lightingPass.input.push_back(gbufferTargetIDs[0]);
		lightingPass.input.push_back(gbufferTargetIDs[1]);
		lightingPass.input.push_back(gbufferTargetIDs[2]);
		lightingPass.input.push_back(gbufferTargetIDs[3]);
		lightingPass.inputStates.push_back(D3D12_RESOURCE_STATE_GENERIC_READ);
		lightingPass.inputStates.push_back(D3D12_RESOURCE_STATE_GENERIC_READ);
		lightingPass.inputStates.push_back(D3D12_RESOURCE_STATE_GENERIC_READ);
		lightingPass.inputStates.push_back(D3D12_RESOURCE_STATE_GENERIC_READ);
		lightingPass.output.push_back(accumTargetID);
		lightingPass.outputStates.push_back(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		passes.push_back(lightingPass);

		sl12::RenderPass tonemapPass{};
		tonemapPass.input.push_back(accumTargetID);
		tonemapPass.inputStates.push_back(D3D12_RESOURCE_STATE_GENERIC_READ);
		passes.push_back(tonemapPass);

		renderGraph_->CreateRenderPasses(&device_, passes, histories, return_histories);
	}

	// create scene constant buffer.
	sl12::CbvHandle hSceneCB, hTraverseCB;
	SceneCB cbScene;
	TraverseCB cbTraverse;
	static const float kFovYRad = DirectX::XMConvertToRadians(60.0f);
	{
		DirectX::XMFLOAT3 camPos(0.0f, 0.0f, cameraLength_);
		DirectX::XMFLOAT3 tgtPos(0.0f, 0.7f, 0.0f);
		DirectX::XMFLOAT3 upVec(0.0f, 1.0f, 0.0f);
		auto cp = DirectX::XMLoadFloat3(&camPos);
		auto tp = DirectX::XMLoadFloat3(&tgtPos);
		auto up = DirectX::XMLoadFloat3(&upVec);
		auto cpRot = DirectX::XMMatrixRotationX(DirectX::XMConvertToRadians(cameraAngleX_)) * DirectX::XMMatrixRotationY(DirectX::XMConvertToRadians(cameraAngleY_));
		cp = DirectX::XMVectorAdd(DirectX::XMVector3Transform(cp, cpRot), tp);
		auto mtxWorldToView = DirectX::XMMatrixLookAtRH(cp, tp, up);
		auto mtxViewToClip = sl12::MatrixPerspectiveInfiniteFovRH(kFovYRad, (float)displayWidth_ / (float)displayHeight_, 0.1f);
		auto mtxWorldToClip = mtxWorldToView * mtxViewToClip;
		auto mtxClipToWorld = DirectX::XMMatrixInverse(nullptr, mtxWorldToClip);
		auto mtxViewToWorld = DirectX::XMMatrixInverse(nullptr, mtxWorldToView);

		DirectX::XMStoreFloat4x4(&cbScene.mtxWorldToProj, mtxWorldToClip);
		DirectX::XMStoreFloat4x4(&cbScene.mtxWorldToView, mtxWorldToView);
		DirectX::XMStoreFloat4x4(&cbScene.mtxProjToWorld, mtxClipToWorld);
		DirectX::XMStoreFloat4x4(&cbScene.mtxViewToWorld, mtxViewToWorld);
		cbScene.screenSize.x = (float)displayWidth_;
		cbScene.screenSize.y = (float)displayHeight_;

		hSceneCB = cbvMan_->GetTemporal(&cbScene, sizeof(cbScene));
	}

	std::set<uint32> visibleIDs;
	bool isTraverseGPU = traverseType_ == 2;
	if (bRenderLOD_)
	{
		DirectX::XMMATRIX mtxWorldToView = DirectX::XMLoadFloat4x4(&cbScene.mtxWorldToView);
		DirectX::XMMATRIX mtxLocalToWorld = DirectX::XMMatrixIdentity();
		if (!isTraverseGPU)
		{
			EnumerateVisibleMeshlets(&myModel_->meshes[0], mtxWorldToView, mtxLocalToWorld, kFovYRad, (float)displayHeight_, traverseType_ == 1, visibleIDs);
		}
		else
		{
			DirectX::XMVECTOR scale, rot, trans;
			DirectX::XMMatrixDecompose(&scale, &rot, &trans, mtxLocalToWorld);
			DirectX::XMFLOAT3 fs3;
			DirectX::XMStoreFloat3(&fs3, scale);
			float maxScale = std::max(fs3.x, std::max(fs3.y, fs3.z));
			cbTraverse.maxScale = maxScale;

			float yScale = 1.0f / tanf(kFovYRad * 0.5f);
			cbTraverse.screenYScale = (float)displayHeight_ * 0.5f * yScale;

			DirectX::XMMATRIX mtxLocalToView = mtxLocalToWorld * mtxWorldToView;
			DirectX::XMStoreFloat4x4(&cbTraverse.mtxLocalToView, mtxLocalToView);

			cbTraverse.errorThreshold = errorThreshold_;

			hTraverseCB = cbvMan_->GetTemporal(&cbTraverse, sizeof(cbTraverse));
		}
	}

	// clear swapchain.
	auto&& swapchain = device_.GetSwapchain();
	pCmdList->TransitionBarrier(swapchain.GetCurrentTexture(kSwapchainBufferOffset), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
	{
		float color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		pCmdList->GetLatestCommandList()->ClearRenderTargetView(swapchain.GetCurrentRenderTargetView(kSwapchainBufferOffset)->GetDescInfo().cpuHandle, color, 0, nullptr);
	}

	// execute work graph.
	if (isTraverseGPU)
	{
		pCmdList->AddTransitionBarrier(&countBuffer_, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		pCmdList->AddTransitionBarrier(&drawArgBuffer_, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		pCmdList->FlushBarriers();

		// clear count buffer.
		{
			// set pipeline.
			pCmdList->GetLatestCommandList()->SetPipelineState(psoClearCount_->GetPSO());

			// set descriptors.
			sl12::DescriptorSet descSet;
			descSet.Reset();
			descSet.SetCsSrv(0, meshletBV_->GetDescInfo().cpuHandle);
			descSet.SetCsUav(0, countUAV_->GetDescInfo().cpuHandle);

			pCmdList->SetComputeRootSignatureAndDescriptorSet(&rsCs_, &descSet);

			// dispatch.
			pCmdList->GetLatestCommandList()->Dispatch(1, 1, 1);
		}
		// execute work graph.
		{
			// set program.
			traverseWGContext_->SetProgram(pCmdList, D3D12_SET_WORK_GRAPH_FLAG_INITIALIZE);

			// set descriptors.
			sl12::DescriptorSet descSet;
			descSet.Reset();
			descSet.SetCsCbv(0, hTraverseCB.GetCBV()->GetDescInfo().cpuHandle);
			descSet.SetCsSrv(0, meshletBV_->GetDescInfo().cpuHandle);
			descSet.SetCsUav(0, countUAV_->GetDescInfo().cpuHandle);
			descSet.SetCsUav(1, drawArgUAV_->GetDescInfo().cpuHandle);

			pCmdList->SetComputeRootSignatureAndDescriptorSet(&rsCs_, &descSet);

			// dispatch graph.
			RootNodeRecord records[] = {
				{myModel_->meshes[0].rootMeshletID},
			};
			traverseWGContext_->DispatchGraphCPU(pCmdList, 0, 1, sizeof(RootNodeRecord), records);
		}

		pCmdList->AddTransitionBarrier(&countBuffer_, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
		pCmdList->AddTransitionBarrier(&drawArgBuffer_, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
		pCmdList->FlushBarriers();
	}
	
	// gbuffer pass.
	renderGraph_->NextPass(pCmdList);
	{
		// output barrier.
		renderGraph_->BarrierOutputsAll(pCmdList);

		// clear depth.
		D3D12_CPU_DESCRIPTOR_HANDLE dsv = renderGraph_->GetTarget(gbufferTargetIDs[3])->dsvs[0]->GetDescInfo().cpuHandle;
		pCmdList->GetLatestCommandList()->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

		// set render targets.
		D3D12_CPU_DESCRIPTOR_HANDLE rtvs[] = {
			renderGraph_->GetTarget(gbufferTargetIDs[0])->rtvs[0]->GetDescInfo().cpuHandle,
			renderGraph_->GetTarget(gbufferTargetIDs[1])->rtvs[0]->GetDescInfo().cpuHandle,
			renderGraph_->GetTarget(gbufferTargetIDs[2])->rtvs[0]->GetDescInfo().cpuHandle,
		};
		pCmdList->GetLatestCommandList()->OMSetRenderTargets(ARRAYSIZE(rtvs), rtvs, false, &dsv);

		// set viewport.
		D3D12_VIEWPORT vp;
		vp.TopLeftX = vp.TopLeftY = 0.0f;
		vp.Width = (float)displayWidth_;
		vp.Height = (float)displayHeight_;
		vp.MinDepth = 0.0f;
		vp.MaxDepth = 1.0f;
		pCmdList->GetLatestCommandList()->RSSetViewports(1, &vp);

		// set scissor rect.
		D3D12_RECT rect;
		rect.left = rect.top = 0;
		rect.right = displayWidth_;
		rect.bottom = displayHeight_;
		pCmdList->GetLatestCommandList()->RSSetScissorRects(1, &rect);

		// create constant buffer.
		MeshCB cbMesh;
		{
			DirectX::XMStoreFloat4x4(&cbMesh.mtxLocalToWorld, DirectX::XMMatrixIdentity());
		}
		auto hMeshCB = cbvMan_->GetTemporal(&cbMesh, sizeof(cbMesh));

		{
			// set descriptors.
			sl12::DescriptorSet descSet;
			descSet.Reset();
			descSet.SetVsCbv(0, hSceneCB.GetCBV()->GetDescInfo().cpuHandle);
			descSet.SetVsCbv(1, hMeshCB.GetCBV()->GetDescInfo().cpuHandle);
			descSet.SetPsCbv(0, hSceneCB.GetCBV()->GetDescInfo().cpuHandle);

			// set pipeline.
			pCmdList->GetLatestCommandList()->SetPipelineState(psoMesh_->GetPSO());
			pCmdList->GetLatestCommandList()->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			// set vertex buffers.
			const D3D12_VERTEX_BUFFER_VIEW vbvs[] = {
				{myPositionBuffer_->GetResourceDep()->GetGPUVirtualAddress(), (UINT)myPositionBuffer_->GetBufferDesc().size, (UINT)myPositionBuffer_->GetBufferDesc().stride},
				{myNormalBuffer_->GetResourceDep()->GetGPUVirtualAddress(), (UINT)myNormalBuffer_->GetBufferDesc().size, (UINT)myNormalBuffer_->GetBufferDesc().stride},
			};
			pCmdList->GetLatestCommandList()->IASetVertexBuffers(0, ARRAYSIZE(vbvs), vbvs);

			if (!bRenderLOD_)
			{
				// set index buffer.
				D3D12_INDEX_BUFFER_VIEW ibv;
				ibv.BufferLocation = baseIndexBuffer_->GetResourceDep()->GetGPUVirtualAddress();
				ibv.SizeInBytes = (UINT)baseIndexBuffer_->GetBufferDesc().size;
				ibv.Format = DXGI_FORMAT_R32_UINT;
				pCmdList->GetLatestCommandList()->IASetIndexBuffer(&ibv);

				pCmdList->SetGraphicsRootSignatureAndDescriptorSet(&rsMesh_, &descSet);

				sl12::u32 colorIndex = 0;
				size_t indexOffset = 0;
				for (auto&& meshlet : myModel_->meshes[0].meshletsLOD0)
				{
					pCmdList->GetLatestCommandList()->SetGraphicsRoot32BitConstant(rsMesh_->GetRootConstantIndex(), bRenderGroupColor_ ? colorIndex : 0, 0);

					pCmdList->GetLatestCommandList()->DrawIndexedInstanced((UINT)meshlet.indices.size(), 1, (UINT)indexOffset, 0, 0);

					colorIndex = (colorIndex + 1) % 16;
					indexOffset += meshlet.indices.size();
				}
			}
			else if (!isTraverseGPU)
			{
				// set index buffer.
				D3D12_INDEX_BUFFER_VIEW ibv;
				ibv.BufferLocation = meshletIndexBuffer_->GetResourceDep()->GetGPUVirtualAddress();
				ibv.SizeInBytes = (UINT)meshletIndexBuffer_->GetBufferDesc().size;
				ibv.Format = DXGI_FORMAT_R32_UINT;
				pCmdList->GetLatestCommandList()->IASetIndexBuffer(&ibv);

				for (auto&& mm : myModel_->meshes[0].meshletMap)
				{
					auto&& meshlet = mm.second;
					if (visibleIDs.find(meshlet.id) != visibleIDs.end())
					{
						pCmdList->SetGraphicsRootSignatureAndDescriptorSet(&rsMesh_, &descSet);
						pCmdList->GetLatestCommandList()->SetGraphicsRoot32BitConstant(rsMesh_->GetRootConstantIndex(), bRenderGroupColor_ ? meshlet.id : 0, 0);

						pCmdList->GetLatestCommandList()->DrawIndexedInstanced((UINT)meshlet.indices.size(), 1, (UINT)meshlet.indexBufferOffset, 0, 0);
					}
				}
			}
			else
			{
				// set index buffer.
				D3D12_INDEX_BUFFER_VIEW ibv;
				ibv.BufferLocation = meshletIndexBuffer_->GetResourceDep()->GetGPUVirtualAddress();
				ibv.SizeInBytes = (UINT)meshletIndexBuffer_->GetBufferDesc().size;
				ibv.Format = DXGI_FORMAT_R32_UINT;
				pCmdList->GetLatestCommandList()->IASetIndexBuffer(&ibv);

				pCmdList->SetGraphicsRootSignatureAndDescriptorSet(&rsMesh_, &descSet);
				pCmdList->GetLatestCommandList()->SetGraphicsRoot32BitConstant(rsMesh_->GetRootConstantIndex(), 0, 0);

				pCmdList->GetLatestCommandList()->ExecuteIndirect(
					meshletIndirect_->GetCommandSignature(),		// command signature
					(UINT)myModel_->meshes[0].meshletsLOD0.size(),	// max command count
					drawArgBuffer_->GetResourceDep(),				// argument buffer
					0,												// argument buffer offset
					countBuffer_->GetResourceDep(),					// count buffer
					0);								// count buffer offset
			}
		}
	}
	renderGraph_->EndPass();

	// lighing pass.
	renderGraph_->NextPass(pCmdList);
	{
		// output barrier.
		renderGraph_->BarrierOutputsAll(pCmdList);

		// set pipeline.
		pCmdList->GetLatestCommandList()->SetPipelineState(psoLighting_->GetPSO());

		// set descriptors.
		sl12::DescriptorSet descSet;
		descSet.Reset();
		descSet.SetCsCbv(0, hSceneCB.GetCBV()->GetDescInfo().cpuHandle);
		descSet.SetCsSrv(0, renderGraph_->GetTarget(gbufferTargetIDs[0])->textureSrvs[0]->GetDescInfo().cpuHandle);
		descSet.SetCsSrv(1, renderGraph_->GetTarget(gbufferTargetIDs[1])->textureSrvs[0]->GetDescInfo().cpuHandle);
		descSet.SetCsSrv(2, renderGraph_->GetTarget(gbufferTargetIDs[2])->textureSrvs[0]->GetDescInfo().cpuHandle);
		descSet.SetCsSrv(3, renderGraph_->GetTarget(gbufferTargetIDs[3])->textureSrvs[0]->GetDescInfo().cpuHandle);
		descSet.SetCsUav(0, renderGraph_->GetTarget(accumTargetID)->uavs[0]->GetDescInfo().cpuHandle);

		pCmdList->SetComputeRootSignatureAndDescriptorSet(&rsCs_, &descSet);

		// dispatch.
		UINT x = (displayWidth_ + 7) / 8;
		UINT y = (displayHeight_ + 7) / 8;
		pCmdList->GetLatestCommandList()->Dispatch(x, y, 1);
	}
	renderGraph_->EndPass();

	// tonemap pass.
	renderGraph_->NextPass(pCmdList);
	{
		// output barrier.
		renderGraph_->BarrierOutputsAll(pCmdList);

		// set render targets.
		auto&& rtv = swapchain.GetCurrentRenderTargetView(kSwapchainBufferOffset)->GetDescInfo().cpuHandle;
		pCmdList->GetLatestCommandList()->OMSetRenderTargets(1, &rtv, false, nullptr);

		// set viewport.
		D3D12_VIEWPORT vp;
		vp.TopLeftX = vp.TopLeftY = 0.0f;
		vp.Width = (float)displayWidth_;
		vp.Height = (float)displayHeight_;
		vp.MinDepth = 0.0f;
		vp.MaxDepth = 1.0f;
		pCmdList->GetLatestCommandList()->RSSetViewports(1, &vp);

		// set scissor rect.
		D3D12_RECT rect;
		rect.left = rect.top = 0;
		rect.right = displayWidth_;
		rect.bottom = displayHeight_;
		pCmdList->GetLatestCommandList()->RSSetScissorRects(1, &rect);

		// set pipeline.
		pCmdList->GetLatestCommandList()->SetPipelineState(psoTonemap_->GetPSO());
		pCmdList->GetLatestCommandList()->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		// set descriptors.
		sl12::DescriptorSet descSet;
		descSet.Reset();
		descSet.SetPsSrv(0, renderGraph_->GetTarget(accumTargetID)->textureSrvs[0]->GetDescInfo().cpuHandle);

		pCmdList->SetGraphicsRootSignatureAndDescriptorSet(&rsVsPs_, &descSet);

		// draw fullscreen.
		pCmdList->GetLatestCommandList()->DrawInstanced(3, 1, 0, 0);
	}
	renderGraph_->EndPass();

	// draw GUI.
	gui_->LoadDrawCommands(pCmdList);
	
	// barrier swapchain.
	pCmdList->TransitionBarrier(swapchain.GetCurrentTexture(kSwapchainBufferOffset), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);

	// wait prev frame render.
	mainCmdList_->Close();
	device_.WaitDrawDone();

	// present swapchain.
	device_.Present(1);

	// execute current frame render.
	mainCmdList_->Execute();

	return true;
}

int SampleApplication::Input(UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_LBUTTONDOWN:
		inputData_.mouseButton |= sl12::MouseButton::Left;
		return 0;
	case WM_RBUTTONDOWN:
		inputData_.mouseButton |= sl12::MouseButton::Right;
		return 0;
	case WM_MBUTTONDOWN:
		inputData_.mouseButton |= sl12::MouseButton::Middle;
		return 0;
	case WM_LBUTTONUP:
		inputData_.mouseButton &= ~sl12::MouseButton::Left;
		return 0;
	case WM_RBUTTONUP:
		inputData_.mouseButton &= ~sl12::MouseButton::Right;
		return 0;
	case WM_MBUTTONUP:
		inputData_.mouseButton &= ~sl12::MouseButton::Middle;
		return 0;
	case WM_MOUSEMOVE:
		inputData_.mouseX = GET_X_LPARAM(lParam);
		inputData_.mouseY = GET_Y_LPARAM(lParam);
		return 0;
	case WM_KEYUP:
	case WM_SYSKEYUP:
		inputData_.key = wParam;
		inputData_.scancode = (int)LOBYTE(HIWORD(lParam));;
		inputData_.keyDown = false;
		return 0;
	case WM_KEYDOWN:
	case WM_SYSKEYDOWN:
		inputData_.key = wParam;
		inputData_.scancode = (int)LOBYTE(HIWORD(lParam));;
		inputData_.keyDown = true;
		return 0;
	case WM_CHAR:
		inputData_.chara = (sl12::u16)wParam;
		return 0;
	}

	return 0;
}

//	EOF
