#include "cbuffer.hlsli"

#if !ENABLE_DYNAMIC_RESOURCE

ConstantBuffer<SceneCB>				cbScene				: register(b0);

Texture2D							texGBufferA			: register(t0);
Texture2D							texGBufferB			: register(t1);
Texture2D							texGBufferC			: register(t2);
Texture2D<float>					texDepth			: register(t3);

RWTexture2D<float4>					rwOutput			: register(u0);

#else

struct ResourceIndex
{
	uint cbScene;
	uint texGBufferA;
	uint texGBufferB;
	uint texGBufferC;
	uint texDepth;
	uint rwOutput;
};

ConstantBuffer<ResourceIndex>	cbResIndex	: register(b0);

#endif

float3 Lighting(uint2 pixelPos, float depth)
{
#if ENABLE_DYNAMIC_RESOURCE
	ConstantBuffer<SceneCB> cbScene = ResourceDescriptorHeap[cbResIndex.cbScene];
	Texture2D texGBufferA = ResourceDescriptorHeap[cbResIndex.texGBufferA];
	Texture2D texGBufferB = ResourceDescriptorHeap[cbResIndex.texGBufferB];
	Texture2D texGBufferC = ResourceDescriptorHeap[cbResIndex.texGBufferC];
#endif
	
	// get gbuffer.
	float4 color = texGBufferA[pixelPos];
	float3 orm = texGBufferB[pixelPos].xyz;
	float3 normal = texGBufferC[pixelPos].xyz * 2.0 - 1.0;

	// get world position.
	float2 screenPos = ((float2)pixelPos + 0.5) / cbScene.screenSize;
	float2 clipSpacePos = screenPos * float2(2, -2) + float2(-1, 1);
	float4 worldPos = mul(cbScene.mtxProjToWorld, float4(clipSpacePos, depth, 1));
	worldPos.xyz /= worldPos.w;

	// apply light.
	float NoL = saturate(normal.y * 0.5 + 0.5);
	float3 diffuse = lerp(color.rgb, 0, orm.b);
	return diffuse * NoL;
}

[numthreads(8, 8, 1)]
void main(
	uint3 gid : SV_GroupID,
	uint3 gtid : SV_GroupThreadID,
	uint3 did : SV_DispatchThreadID)
{
	uint2 pixelPos = did.xy;

#if ENABLE_DYNAMIC_RESOURCE
	ConstantBuffer<SceneCB> cbScene = ResourceDescriptorHeap[cbResIndex.cbScene];
	Texture2D<float> texDepth = ResourceDescriptorHeap[cbResIndex.texDepth];
	RWTexture2D<float4> rwOutput = ResourceDescriptorHeap[cbResIndex.rwOutput];
#endif

	if (all(pixelPos < (uint2)cbScene.screenSize))
	{
		float depth = texDepth[pixelPos];
		[branch]
		if (depth >= 1.0)
		{
			rwOutput[pixelPos] = float4(0, 0, 0, 1);
		}
		else
		{
			rwOutput[pixelPos] = float4(Lighting(pixelPos, depth), 1);
		}
	}
}