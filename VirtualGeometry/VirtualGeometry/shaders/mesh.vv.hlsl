#include "cbuffer.hlsli"

struct VSInput
{
	float3	position	: POSITION;
	float3	normal		: NORMAL;
};

struct VSOutput
{
	float4	position	: SV_POSITION;
	float3	normal		: NORMAL;
};

#if !ENABLE_DYNAMIC_RESOURCE

ConstantBuffer<SceneCB>		cbScene			: register(b0);
ConstantBuffer<MeshCB>		cbMesh			: register(b1);

#else

struct ResourceIndex
{
	uint	SceneCB;
	uint	MeshCB;
};

ConstantBuffer<ResourceIndex>	cbResIndex	: register(b0);

#endif

VSOutput main(const VSInput In)
{
	VSOutput Out = (VSOutput)0;

#if ENABLE_DYNAMIC_RESOURCE
	ConstantBuffer<SceneCB> cbScene = ResourceDescriptorHeap[cbResIndex.SceneCB];
	ConstantBuffer<MeshCB> cbMesh = ResourceDescriptorHeap[cbResIndex.MeshCB];
#endif

	float4x4 mtxLocalToProj = mul(cbScene.mtxWorldToProj, cbMesh.mtxLocalToWorld);

	float3 pos = In.position.xzy * float3(1, -1, 1);
	float3 norm = In.normal.xzy * float3(1, -1, 1);
	Out.position = mul(mtxLocalToProj, float4(pos, 1));
	Out.normal = normalize(mul((float3x3)cbMesh.mtxLocalToWorld, norm));

	return Out;
}

//	EOF
