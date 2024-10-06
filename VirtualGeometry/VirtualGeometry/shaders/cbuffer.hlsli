#ifndef CBUFFER_HLSLI
#define  CBUFFER_HLSLI

#ifdef USE_IN_CPP
#	define		float4x4		DirectX::XMFLOAT4X4
#	define		float4			DirectX::XMFLOAT4
#	define		float3			DirectX::XMFLOAT3
#	define		float2			DirectX::XMFLOAT2
#	define		uint			UINT
#endif

struct RootIndexCB
{
    uint        index;
};

struct SceneCB
{
    float4x4	mtxWorldToProj;
    float4x4	mtxWorldToView;
    float4x4    mtxProjToWorld;
    float4x4    mtxViewToWorld;
    float2      screenSize;
};

struct MeshCB
{
    float4x4	mtxLocalToWorld;
};

struct ColorCB
{
    uint        colorIndex;
};

struct TraverseCB
{
    float4x4    mtxLocalToView;
    float       screenYScale;   // ScreenHeight * 0.5 / tan(fovY * 0.5)
    float       maxScale;
    float       errorThreshold;
};

struct MeshletData
{
    uint        children[8];
    float3      groupSphereCenter;
    float       groupError;
    float3      parentSphereCenter;
    float       parentError;
    uint        childCount;
    uint        indexCount;
    uint        indexOffset;
};

struct RootNodeRecord
{
    uint	RootMeshletID;
};

struct RecursiveRecord
{
    uint	meshletID;
};

#endif // CBUFFER_HLSLI
//  EOF
