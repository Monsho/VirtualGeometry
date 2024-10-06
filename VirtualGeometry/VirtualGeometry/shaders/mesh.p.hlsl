#include "cbuffer.hlsli"
#include "math.hlsli"

struct PSInput
{
	float4	position	: SV_POSITION;
	float3	normal		: NORMAL;
};

struct PSOutput
{
	float4	color	: SV_TARGET0;
	float4	orm		: SV_TARGET1;
	float4	normal	: SV_TARGET2;
};

#if !ENABLE_DYNAMIC_RESOURCE

ConstantBuffer<SceneCB>					cbScene			: register(b0);
ConstantBuffer<ColorCB>					cbColor			: register(b0, space1);

#else

struct ResourceIndex
{
	uint	SceneCB;
	uint	ColorCB;
};

ConstantBuffer<ResourceIndex>	cbResIndex	: register(b0);

#endif

float4 GetColor(uint index)
{
	const float4 kColors[] = {
		float4(1, 1, 1, 1),
		float4(1, 0, 0, 1),
		float4(0, 1, 0, 1),
		float4(0, 0, 1, 1),
		float4(1, 1, 0, 1),
		float4(1, 0, 1, 1),
		float4(0, 1, 1, 1),
		float4(1, 0.5, 0, 1),
		float4(0.5, 1, 0, 1),
		float4(1, 0, 0.5, 1),
		float4(0.5, 0, 1, 1),
		float4(0, 1, 0.5, 1),
		float4(0, 0.5, 1, 1),
		float4(1, 1, 0.5, 1),
		float4(1, 0.5, 1, 1),
		float4(0.5, 1, 1, 1),
	};
	return kColors[index % 16];
}

PSOutput main(PSInput In)
{
	PSOutput Out = (PSOutput)0;

#if ENABLE_DYNAMIC_RESOURCE
	ConstantBuffer<SceneCB> cbScene = ResourceDescriptorHeap[cbResIndex.SceneCB];
	ConstantBuffer<ColorCB> cbColor = ResourceDescriptorHeap[cbResIndex.ColorCB];
#endif

	float4 baseColor = GetColor(cbColor.colorIndex);
	float3 orm = float3(1.0, 0.5, 0.0);

	float3 normalInWS = In.normal;

	Out.color = baseColor;
	Out.orm.rgb = orm;
	Out.normal.xyz = normalInWS * 0.5 + 0.5;

	return Out;
}
