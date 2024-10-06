#pragma once

#include <algorithm>
#include <memory>
#include <vector>
#include <map>
#include <GLTFSDK/GLBResourceReader.h>
#include <GLTFSDK/Deserialize.h>
#include <meshoptimizer.h>
#include <sl12/types.h>

using int8 = char;
using uint8 = unsigned char;
using int16 = short;
using uint16 = unsigned short;
using int32 = int;
using uint32 = unsigned int;

struct MyBounds
{
    Microsoft::glTF::Vector3 sphereCenter;
    float sphereRadius;
    Microsoft::glTF::Vector3 coneApex;
    Microsoft::glTF::Vector3 coneAxis;
    float coneCutoff;
};

struct MyMeshlet
{
    // shape.
    std::vector<uint32> indices;
    std::vector<uint32> meshletVertices;
    std::vector<uint8> meshletTriangles;
    size_t indexBufferOffset;

    // bounds.
    MyBounds bounds;
    MyBounds groupBounds;
    MyBounds parentBounds;

    // tree.
    uint32 id;
    uint16 lod;
    std::vector<uint32> children;
    std::vector<uint32> traverseChildren;
    std::vector<uint32> parents;

    // error.
    float meshletError = 0.0f;
    float parentError = std::numeric_limits<float>::infinity();
};

struct MyMesh
{
    std::vector<uint32> indices;
    std::vector<uint32> meshletIndices;
    std::vector<Microsoft::glTF::Vector3> positions;
    std::vector<Microsoft::glTF::Vector3> normals;
    std::vector<MyMeshlet> meshletsLOD0;
    std::map<uint32, MyMeshlet> meshletMap;
    uint32 rootMeshletID;
    uint16 maxLOD;
};

struct MyModel
{
    std::vector<MyMesh> meshes;
};

std::unique_ptr<MyModel> LoadGLTFModel(const char* filename);
void ProcessModel(std::unique_ptr<MyModel>& myModel);
