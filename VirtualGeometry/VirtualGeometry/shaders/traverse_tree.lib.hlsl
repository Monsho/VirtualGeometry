#include "cbuffer.hlsli"

ConstantBuffer<TraverseCB>	cbTraverse				: register(b0);

StructuredBuffer<MeshletData>	rMeshletData		: register(t0);

RWByteAddressBuffer		rwCountBuffer				: register(u0);
RWByteAddressBuffer		rwDrawArgs					: register(u1);

float ErrorProjection(in float3 center, in float radius)
{
	float d2 = dot(center, center);
	return cbTraverse.screenYScale * radius / (d2 - radius * radius);
	//return cbTraverse.screenYScale * radius / sqrt(d2 - radius * radius);
};

bool IsMeshletVisible(in MeshletData meshlet)
{
	float ownError = meshlet.groupError * cbTraverse.maxScale;
	float3 ownCenter = mul(cbTraverse.mtxLocalToView, float4(meshlet.groupSphereCenter, 1)).xyz;
	ownError = ErrorProjection(ownCenter, max(ownError, 1e-8f));

	float parentError = meshlet.parentError * cbTraverse.maxScale;
	float3 parentCenter = mul(cbTraverse.mtxLocalToView, float4(meshlet.parentSphereCenter, 1)).xyz;
	parentError = ErrorProjection(parentCenter, max(parentError, 1e-8f));

	bool isVisible = ownError <= cbTraverse.errorThreshold && cbTraverse.errorThreshold < parentError;
	return isVisible;
};

[NumThreads(1, 1, 1)]
void ClearCountCS()
{
	MeshletData data = rMeshletData[2650];
	if (data.childCount > 0)
		rwCountBuffer.Store(0, 0);
	else
		rwCountBuffer.Store(33, 0);
}

[Shader("node")]
[NodeLaunch("broadcasting")]
[NodeDispatchGrid(1, 1, 1)]
[NumThreads(1, 1, 1)]
void RootNode(
	uint gid : SV_GroupID,
	uint gtid : SV_GroupThreadID,
	uint dtid : SV_DispatchThreadID,
	DispatchNodeInputRecord<RootNodeRecord> inputRecord,
	[MaxRecords(8)] NodeOutput<RecursiveRecord> RecursiveNode)
{
	RootNodeRecord inRec = inputRecord.Get();
	uint meshletID = inRec.RootMeshletID;
	MeshletData meshlet = rMeshletData[meshletID];
	bool isVisible = IsMeshletVisible(meshlet);

	if (isVisible)
	{
		uint ov;
		rwCountBuffer.InterlockedAdd(0, 1, ov);
		uint baseAddr = ov * 4 * 6;
		uint4 arg0 = uint4(meshletID, meshlet.indexCount, 1, meshlet.indexOffset);
		uint2 arg1 = uint2(0, 0);
		rwDrawArgs.Store4(baseAddr, arg0);
		rwDrawArgs.Store2(baseAddr + 16, arg1);
	}
	else
	{
		ThreadNodeOutputRecords<RecursiveRecord> outRecs = RecursiveNode.GetThreadNodeOutputRecords(meshlet.childCount);
		for (uint i = 0; i < meshlet.childCount; i++)
		{
			outRecs[i].meshletID = meshlet.children[i];
		}
		outRecs.OutputComplete();
	}
}

[Shader("node")]
[NodeLaunch("thread")]
[NodeMaxRecursionDepth(16)]
void RecursiveNode(
	ThreadNodeInputRecord<RecursiveRecord> inputRecord,
	[MaxRecords(8)] NodeOutput<RecursiveRecord> RecursiveNode)
{
	uint meshletID = inputRecord.Get().meshletID;
	MeshletData meshlet = rMeshletData[meshletID];
	bool isVisible = IsMeshletVisible(meshlet);

	if (isVisible)
	{
		uint ov;
		rwCountBuffer.InterlockedAdd(0, 1, ov);
		uint baseAddr = ov * 4 * 6;
		uint4 arg0 = uint4(meshletID, meshlet.indexCount, 1, meshlet.indexOffset);
		uint2 arg1 = uint2(0, 0);
		rwDrawArgs.Store4(baseAddr, arg0);
		rwDrawArgs.Store2(baseAddr + 16, arg1);
	}
	else
	{
		ThreadNodeOutputRecords<RecursiveRecord> outRecs = RecursiveNode.GetThreadNodeOutputRecords(meshlet.childCount);
		for (uint i = 0; i < meshlet.childCount; i++)
		{
			outRecs[i].meshletID = meshlet.children[i];
		}
		outRecs.OutputComplete();
	}
}
