#include <set>
#include "sl12/application.h"
#include "sl12/resource_loader.h"
#include "sl12/shader_manager.h"
#include "sl12/command_list.h"
#include "sl12/gui.h"
#include "sl12/root_signature.h"
#include "sl12/pipeline_state.h"
#include "sl12/unique_handle.h"
#include "sl12/cbv_manager.h"
#include "sl12/render_graph.h"
#include "sl12/texture_streamer.h"
#include "sl12/resource_mesh.h"
#include "sl12/resource_streaming_texture.h"
#include "sl12/work_graph.h"
#include "sl12/indirect_executer.h"
#include "mesh_grouping.h"


class SampleApplication
	: public sl12::Application
{
	template <typename T> using UniqueHandle = sl12::UniqueHandle<T>;

	struct WorkMaterial
	{
		const sl12::ResourceItemMesh::Material*	pResMaterial;
		std::vector<sl12::ResourceHandle>		texHandles;

		bool operator==(const WorkMaterial& rhs) const
		{
			return pResMaterial == rhs.pResMaterial;
		}
		bool operator!=(const WorkMaterial& rhs) const
		{
			return !operator==(rhs);
		}

		sl12::u32 GetCurrentMiplevel() const
		{
			if (!texHandles.empty())
			{
				auto sTex = texHandles[0].GetItem<sl12::ResourceItemStreamingTexture>();
				if (sTex)
				{
					return sTex->GetCurrMipLevel();
				}
			}
			return 0;
		}
	};	// struct WorkMaterial

public:
	SampleApplication(HINSTANCE hInstance, int nCmdShow, int screenWidth, int screenHeight, sl12::ColorSpaceType csType, const std::string& homeDir);
	virtual ~SampleApplication();

	// virtual
	virtual bool Initialize() override;
	virtual bool Execute() override;
	virtual void Finalize() override;
	virtual int Input(UINT message, WPARAM wParam, LPARAM lParam) override;

private:
	static const int kBufferCount = sl12::Swapchain::kMaxBuffer;

	struct CommandLists
	{
		sl12::CommandList	cmdLists[kBufferCount];
		int					index = 0;

		CommandLists()
		{}
		~CommandLists()
		{
			Destroy();
		}

		bool Initialize(sl12::Device* pDev, sl12::CommandQueue* pQueue)
		{
			for (auto&& v : cmdLists)
			{
				if (!v.Initialize(pDev, pQueue, true))
				{
					return false;
				}
			}
			index = 0;
			return true;
		}

		void Destroy()
		{
			for (auto&& v : cmdLists) v.Destroy();
		}

		sl12::CommandList& Reset()
		{
			index = (index + 1) % kBufferCount;
			auto&& ret = cmdLists[index];
			ret.Reset();
			return ret;
		}

		void Close()
		{
			cmdLists[index].Close();
		}

		void Execute()
		{
			cmdLists[index].Execute();
		}

		sl12::CommandQueue* GetParentQueue()
		{
			return cmdLists[index].GetParentQueue();
		}
	};	// struct CommandLists

	// functions.
	void ControlCamera();
	void EnumerateVisibleMeshlets(const MyMesh* InMyMesh, const DirectX::XMMATRIX& InMtxWorldToView, const DirectX::XMMATRIX& InMtxLocalToWorld, float InFovYRad, float InScreenHeight, bool isParallelTraverse, std::set<uint32>& OutMeshletIDs);

private:
	std::string		homeDir_;
	
	UniqueHandle<sl12::ResourceLoader>	resLoader_;
	UniqueHandle<sl12::ShaderManager>	shaderMan_;
	UniqueHandle<sl12::MeshManager>		meshMan_;
	UniqueHandle<CommandLists>			mainCmdList_;
	UniqueHandle<sl12::CbvManager>		cbvMan_;
	UniqueHandle<sl12::RenderGraph>		renderGraph_;
	UniqueHandle<sl12::TextureStreamer>	texStreamer_;

	UniqueHandle<sl12::RootSignature>			rsMesh_;
	UniqueHandle<sl12::RootSignature>			rsVsPs_;
	UniqueHandle<sl12::RootSignature>			rsCs_;
	UniqueHandle<sl12::RootSignature>			rsMeshDR_;
	UniqueHandle<sl12::RootSignature>			rsTonemapDR_;
	UniqueHandle<sl12::RootSignature>			rsLightingDR_;
	UniqueHandle<sl12::GraphicsPipelineState>	psoMesh_;
	UniqueHandle<sl12::GraphicsPipelineState>	psoTonemap_;
	UniqueHandle<sl12::ComputePipelineState>	psoLighting_;
	UniqueHandle<sl12::ComputePipelineState>	psoClearCount_;

	UniqueHandle<sl12::Buffer>				myPositionBuffer_;
	UniqueHandle<sl12::Buffer>				myNormalBuffer_;
	UniqueHandle<sl12::Buffer>				baseIndexBuffer_;
	UniqueHandle<sl12::Buffer>				meshletIndexBuffer_;
	
	UniqueHandle<sl12::Texture>				depthTex_;
	UniqueHandle<sl12::DepthStencilView>	depthDSV_;

	UniqueHandle<sl12::Sampler>				linearSampler_;

	UniqueHandle<sl12::WorkGraphState>		traverseWGState_;
	UniqueHandle<sl12::WorkGraphContext>	traverseWGContext_;
	UniqueHandle<sl12::Buffer>				meshletBuffer_;
	UniqueHandle<sl12::Buffer>				countBuffer_;
	UniqueHandle<sl12::Buffer>				drawArgBuffer_;
	UniqueHandle<sl12::BufferView>			meshletBV_;
	UniqueHandle<sl12::UnorderedAccessView>	countUAV_;
	UniqueHandle<sl12::UnorderedAccessView>	drawArgUAV_;
	UniqueHandle<sl12::IndirectExecuter>	meshletIndirect_;

	UniqueHandle<sl12::Gui>		gui_;
	sl12::InputData				inputData_{};

	sl12::ShaderHandle		hMeshVV_;
	sl12::ShaderHandle		hMeshP_;
	sl12::ShaderHandle		hLightingC_;
	sl12::ShaderHandle		hFullscreenVV_;
	sl12::ShaderHandle		hTonemapP_;
	sl12::ShaderHandle		hClearCountC_;
	sl12::ShaderHandle		hTraverseLib_;

	std::unique_ptr<MyModel>	myModel_;
	
	std::vector<WorkMaterial>	workMaterials_;

	float cameraAngleX_ = 0.0f;
	float cameraAngleY_ = 0.0f;
	float cameraLength_ = 3.0f;
	bool bRenderGroupColor_ = true;
	bool bRenderLOD_ = false;
	int traverseType_ = 0;
	bool bPrintError_ = false;
	float errorThreshold_ = 0.01f;
	sl12::CpuTimer traverseTime_;
	
	int	displayWidth_, displayHeight_;
};	// class SampleApplication

//	EOF
