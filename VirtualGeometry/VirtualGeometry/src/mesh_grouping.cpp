#include "mesh_grouping.h"

#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <memory>
#include <metis.h>

#include "sl12/util.h"

using namespace Microsoft::glTF;

namespace
{
    uint32 CurrMeshletID = 0;
    uint32 CurrGroupID = 0;

    // from meshoptimizer.
    void ComputeBoundingSphere(const std::vector<uint32>& InIndices, const std::vector<Vector3>& InPositions, Vector3& OutCenter, float& OutRadius)
    {
        // find extremum points along all 3 axes; for each axis we get a pair of points with min/max coordinates
        // find extremum points along all 3 axes; for each axis we get a pair of points with min/max coordinates
        size_t pmin[3] = {0, 0, 0};
        size_t pmax[3] = {0, 0, 0};

        for (auto index : InIndices)
        {
            auto& p = InPositions[index];

            // x
            pmin[0] = (p.x < InPositions[pmin[0]].x) ? index : pmin[0];
            pmax[0] = (p.x > InPositions[pmax[0]].x) ? index : pmax[0];
            // y
            pmin[1] = (p.y < InPositions[pmin[1]].y) ? index : pmin[1];
            pmax[1] = (p.y > InPositions[pmax[1]].y) ? index : pmax[1];
            // z
            pmin[2] = (p.z < InPositions[pmin[2]].z) ? index : pmin[2];
            pmax[2] = (p.z > InPositions[pmax[2]].z) ? index : pmax[2];
        }

        // find the pair of points with largest distance
        float paxisd2 = 0;
        int paxis = 0;
        for (int axis = 0; axis < 3; ++axis)
        {
            auto& p1 = InPositions[pmin[axis]];
            auto& p2 = InPositions[pmax[axis]];

            Vector3 d(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            float d2 = d.x * d.x + d.y * d.y + d.z * d.z;

            if (d2 > paxisd2)
            {
                paxisd2 = d2;
                paxis = axis;
            }
        }

        // use the longest segment as the initial sphere diameter
        auto& p1 = InPositions[pmin[paxis]];
        auto& p2 = InPositions[pmax[paxis]];

        Vector3 center((p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f, (p1.z + p2.z) * 0.5f);
        float radius = sqrtf(paxisd2) * 0.5f;

        // iteratively adjust the sphere up until all points fit
        for (auto index : InIndices)
        {
            auto& p = InPositions[index];
            float d2 = (p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y) + (p.z - center.z) * (p.z - center.z);

            if (d2 > radius * radius)
            {
                float d = sqrtf(d2);
                assert(d > 0);

                float k = 0.5f + (radius / d) / 2;

                center.x = center.x * k + p.x * (1 - k);
                center.y = center.y * k + p.y * (1 - k);
                center.z = center.z * k + p.z * (1 - k);
                radius = (radius + d) / 2;
            }
        }

        OutCenter = center;
        OutRadius = radius;
    }
}

class StreamReader : public IStreamReader
{
public:
    StreamReader()
    {}

    std::shared_ptr<std::istream> GetInputStream(const std::string& filename) const override
    {
        auto stream = std::make_shared<std::ifstream>(filename, std::ios_base::binary);
        return stream;
    }

private:
};

std::unique_ptr<MyModel> LoadGLTFModel(const char* filename)
{
    auto stream_reader = std::make_unique<StreamReader>();
    auto gltf_stream = stream_reader->GetInputStream(filename);
    std::unique_ptr<GLTFResourceReader> resource_reader;

    auto glb_res_reader = std::make_unique<GLBResourceReader>(std::move(stream_reader), std::move(gltf_stream));
    std::string manifest = glb_res_reader->GetJson();
    resource_reader = std::move(glb_res_reader);

    auto document = Deserialize(manifest);

    std::unique_ptr<MyModel> myModel = std::make_unique<MyModel>();

    size_t numMeshes = document.meshes.Size();
    for (size_t i = 0; i < numMeshes; i++)
    {
        auto&& mesh = document.meshes[i];
        MyMesh myMesh;
        for (auto&& prim : mesh.primitives)
        {
            // indices.
            auto&& index_accessor = document.accessors.Get(prim.indicesAccessorId);
            if (index_accessor.componentType == Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_BYTE)
            {
                auto indexBuffer = resource_reader->ReadBinaryData<uint8>(document, index_accessor);
                myMesh.indices.reserve(indexBuffer.size());
                for (auto&& index : indexBuffer)
                {
                    myMesh.indices.push_back((uint32)index);
                }
            }
            else if (index_accessor.componentType == Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_SHORT)
            {
                auto indexBuffer = resource_reader->ReadBinaryData<uint16>(document, index_accessor);
                myMesh.indices.reserve(indexBuffer.size());
                for (auto&& index : indexBuffer)
                {
                    myMesh.indices.push_back((uint32)index);
                }
            }
            else if (index_accessor.componentType == Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_INT)
            {
                myMesh.indices = resource_reader->ReadBinaryData<uint32>(document, index_accessor);
            }

            // vertex attributes.
            std::string accessorId;
            if (prim.TryGetAttributeAccessorId("POSITION", accessorId))
            {
                auto&& accessor = document.accessors.Get(accessorId);
                auto pos = resource_reader->ReadBinaryData<float>(document, accessor);
                size_t vertex_count = pos.size() / 3;
                myMesh.positions.reserve(vertex_count);
                for (size_t i = 0; i < vertex_count; ++i)
                {
                    Vector3 v(pos[i * 3 + 0], pos[i * 3 + 1], pos[i * 3 + 2]);
                    myMesh.positions.push_back(v);
                }

                if (prim.TryGetAttributeAccessorId("NORMAL", accessorId))
                {
                    auto&& accessor = document.accessors.Get(accessorId);
                    auto norm = resource_reader->ReadBinaryData<float>(document, accessor);
                    myMesh.normals.reserve(vertex_count);
                    for (size_t i = 0; i < vertex_count; ++i)
                    {
                        Vector3 v(norm[i * 3 + 0], norm[i * 3 + 1], norm[i * 3 + 2]);
                        myMesh.normals.push_back(v);
                    }
                }
            }
        }

        // optimize mesh.
        std::vector<uint32> newIndices;
        newIndices.resize(myMesh.indices.size());
        meshopt_optimizeVertexCache(newIndices.data(), myMesh.indices.data(), myMesh.indices.size(), myMesh.positions.size());
        myMesh.indices = newIndices;

        myModel->meshes.push_back(myMesh);
    }

    return std::move(myModel);
}

//#define BuildMeshlets BuildMeshletsByMeshopt
#define BuildMeshlets BuildMeshletsByMETIS

void BuildMeshletsByMeshopt(const std::vector<uint32>& InIndices, const std::vector<Vector3>& InPositions, uint16 lod, std::vector<MyMeshlet>& OutMeshlets)
{
    static const size_t kMaxMeshletVertices = 255;
    static const size_t kMaxMeshletTriangles = 128;
    static const float kConeWeight = 0.0f;
    size_t maxMeshlets = meshopt_buildMeshletsBound(InIndices.size(), kMaxMeshletVertices, kMaxMeshletTriangles);

    // allocate result buffers.
    std::vector<meshopt_Meshlet> moMeshlets;
    std::vector<uint32> moMeshletVertices;
    std::vector<uint8> moMeshletTriangles;
    moMeshlets.resize(maxMeshlets);
    moMeshletVertices.resize(kMaxMeshletVertices * maxMeshlets);
    moMeshletTriangles.resize(kMaxMeshletTriangles * maxMeshlets * 3);

    // build meshlets.
    size_t numMeshlets = meshopt_buildMeshlets(
        moMeshlets.data(), moMeshletVertices.data(), moMeshletTriangles.data(),
        InIndices.data(), InIndices.size(),
        reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(InPositions[0]),
        kMaxMeshletVertices, kMaxMeshletTriangles, kConeWeight);

    // store meshlets.
    OutMeshlets.clear();
    OutMeshlets.resize(numMeshlets);
    for (size_t i = 0; i < numMeshlets; i++)
    {
        meshopt_Meshlet& moMeshlet = moMeshlets[i];
        meshopt_Bounds moBounds = meshopt_computeMeshletBounds(&moMeshletVertices[moMeshlet.vertex_offset], &moMeshletTriangles[moMeshlet.triangle_offset], moMeshlet.triangle_count, reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(InPositions[0]));

        MyMeshlet& myMeshlet = OutMeshlets[i];
        myMeshlet.bounds.sphereCenter = Vector3(moBounds.center[0], moBounds.center[1], moBounds.center[2]);
        myMeshlet.bounds.sphereRadius = moBounds.radius;
        myMeshlet.bounds.coneApex = Vector3(moBounds.cone_apex[0], moBounds.cone_apex[1], moBounds.cone_apex[2]);
        myMeshlet.bounds.coneAxis = Vector3(moBounds.cone_axis[0], moBounds.cone_axis[1], moBounds.cone_axis[2]);
        myMeshlet.bounds.coneCutoff = moBounds.cone_cutoff;
        myMeshlet.groupBounds = myMeshlet.bounds;
        myMeshlet.id = CurrMeshletID++;
        myMeshlet.lod = lod;
        myMeshlet.meshletError = 0.0f;
        myMeshlet.parentError = std::numeric_limits<float>::infinity();

        for (uint32 v = 0; v < moMeshlet.vertex_count; v++)
        {
            myMeshlet.meshletVertices.push_back(moMeshletVertices[moMeshlet.vertex_offset + v]);
        }
        for (uint32 t = 0; t < moMeshlet.triangle_count; t++)
        {
            myMeshlet.meshletTriangles.push_back(moMeshletTriangles[moMeshlet.triangle_offset + t * 3 + 0]);
            myMeshlet.meshletTriangles.push_back(moMeshletTriangles[moMeshlet.triangle_offset + t * 3 + 1]);
            myMeshlet.meshletTriangles.push_back(moMeshletTriangles[moMeshlet.triangle_offset + t * 3 + 2]);
        }
        for (uint8 t : myMeshlet.meshletTriangles)
        {
            myMeshlet.indices.push_back(myMeshlet.meshletVertices[t]);
        }
        //sl12::ConsolePrint("Meshlet triangles : %d\n", myMeshlet.indices.size() / 3);
    }
}
void BuildMeshletsByMETIS(const std::vector<uint32>& InIndices, const std::vector<Vector3>& InPositions, uint16 lod, std::vector<MyMeshlet>& OutMeshlets)
{
    if (InIndices.size() / 3 > 128)
    {
        using EdgeHash = std::pair<uint32, uint32>;
        auto GenEdgeHash = [](uint32 v0, uint32 v1)
        {
            return EdgeHash(std::min(v0, v1), std::max(v0, v1));
        };

        // generate edges.
        size_t numTris = InIndices.size() / 3;
        std::vector<std::set<EdgeHash>> triEdges;
        std::map<EdgeHash, std::set<uint32>> edgeHashToTriMap;
        triEdges.resize(numTris);
        for (size_t i = 0; i < numTris; i++)
        {
            uint32 v0 = InIndices[i * 3 + 0];
            uint32 v1 = InIndices[i * 3 + 1];
            uint32 v2 = InIndices[i * 3 + 2];
            auto e0 = GenEdgeHash(v0, v1);
            auto e1 = GenEdgeHash(v1, v2);
            auto e2 = GenEdgeHash(v2, v0);
            triEdges[i].insert(e0);
            triEdges[i].insert(e1);
            triEdges[i].insert(e2);

            edgeHashToTriMap[e0].insert((uint32)i);
            edgeHashToTriMap[e1].insert((uint32)i);
            edgeHashToTriMap[e2].insert((uint32)i);
        }

        // create graph.
        std::vector<std::vector<uint32>> linksInNodes;
        linksInNodes.resize(numTris);
        for (auto&& edgeHashToTri : edgeHashToTriMap)
        {
            auto&& tris = edgeHashToTri.second;
            for (auto t : tris)
            {
                auto&& nodes = linksInNodes[t];
                for (auto ot : tris)
                {
                    if (t != ot)
                    {
                        nodes.push_back(ot);
                    }
                }
            }
        }
    
        // create adjacency.
        std::vector<idx_t> xadj;
        std::vector<idx_t> adjncy;
        idx_t numEdges = 0;
        for (auto&& con : linksInNodes)
        {
            xadj.push_back(numEdges);
            for (auto t : con)
            {
                adjncy.push_back((idx_t)t);
                numEdges++;
            }
        }
        xadj.push_back(numEdges);

        // partition graph.
        static const size_t kMaxMeshletVertices = 255;
        static const size_t kMaxMeshletTriangles = 124;
        idx_t nVertices = (idx_t)numTris;
        idx_t nWeights = 1;
        idx_t nParts = (idx_t)((numTris + kMaxMeshletTriangles - 1) / kMaxMeshletTriangles);
        std::vector<idx_t> vwgt(nVertices, 1);
        std::vector<idx_t> adjwgt(numEdges, 1);
        std::vector<idx_t> options(METIS_NOPTIONS);
        METIS_SetDefaultOptions(&options[0]);
        std::vector<idx_t> part(nVertices);
        idx_t objval;

        int ret = METIS_PartGraphKway(&nVertices, &nWeights, &xadj[0], &adjncy[0], &vwgt[0], NULL, &adjwgt[0], &nParts, NULL, NULL, &options[0], &objval, &part[0]);
        assert(ret == METIS_OK);

        // store indices in meshlets.
        OutMeshlets.clear();
        OutMeshlets.resize(nParts);
        for (size_t i = 0; i < numTris; i++)
        {
            MyMeshlet& meshlet = OutMeshlets[part[i]];
            meshlet.indices.push_back(InIndices[i * 3 + 0]);
            meshlet.indices.push_back(InIndices[i * 3 + 1]);
            meshlet.indices.push_back(InIndices[i * 3 + 2]);
        }
    }
    else
    {
        OutMeshlets.clear();
        OutMeshlets.resize(1);
        OutMeshlets[0].indices = InIndices;
    }

    // optimize meshlets.
    for (auto&& meshlet : OutMeshlets)
    {
        if (meshlet.indices.size() > 128 * 3)
        {
            sl12::ConsolePrint("Error!! meshlet has > 128 triangles!!\n");
        }
        //sl12::ConsolePrint("Meshlet triangles : %d\n", meshlet.indices.size() / 3);

        // optimize vertex cache.
        std::vector<uint32> temp;
        temp.resize(meshlet.indices.size());
        meshopt_optimizeVertexCache(temp.data(), meshlet.indices.data(), meshlet.indices.size(), InPositions.size());
        meshlet.indices = temp;

        // compute bounds.
        meshopt_Bounds moBounds = meshopt_computeClusterBounds(meshlet.indices.data(), meshlet.indices.size(), reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(InPositions[0]));
        meshlet.bounds.sphereCenter = Vector3(moBounds.center[0], moBounds.center[1], moBounds.center[2]);
        meshlet.bounds.sphereRadius = moBounds.radius;
        meshlet.bounds.coneApex = Vector3(moBounds.cone_apex[0], moBounds.cone_apex[1], moBounds.cone_apex[2]);
        meshlet.bounds.coneAxis = Vector3(moBounds.cone_axis[0], moBounds.cone_axis[1], moBounds.cone_axis[2]);
        meshlet.bounds.coneCutoff = moBounds.cone_cutoff;
        meshlet.groupBounds = meshlet.bounds;
        meshlet.id = CurrMeshletID++;
        meshlet.lod = lod;
        meshlet.meshletError = 0.0f;
        meshlet.parentError = std::numeric_limits<float>::infinity();
    }
}

using EdgeHash = std::pair<uint32, uint32>;

void GenerateBorderEdgeMap(const MyMeshlet& InMeshlet, std::set<EdgeHash>& OutEdgeMap)
{
    // generate all edge map.
    std::map<EdgeHash, int> edgeUseCount;
    size_t numTris = InMeshlet.indices.size() / 3;
    for (size_t t = 0; t < numTris; t++)
    {
        uint32 v0 = InMeshlet.indices[t * 3 + 0];
        uint32 v1 = InMeshlet.indices[t * 3 + 1];
        uint32 v2 = InMeshlet.indices[t * 3 + 2];
        EdgeHash e0(std::min(v0, v1), std::max(v0, v1));
        EdgeHash e1(std::min(v1, v2), std::max(v1, v2));
        EdgeHash e2(std::min(v2, v0), std::max(v2, v0));
        edgeUseCount[e0] = edgeUseCount[e0] + 1;
        edgeUseCount[e1] = edgeUseCount[e1] + 1;
        edgeUseCount[e2] = edgeUseCount[e2] + 1;
    }

    // find border edges.
    OutEdgeMap.clear();
    for (auto edge : edgeUseCount)
    {
        if (edge.second == 1)
        {
            OutEdgeMap.insert(edge.first);
        }
    }
}

void GroupMeshlets(const std::vector<MyMeshlet>& InMeshlets, std::vector<std::vector<MyMeshlet>>& OutGroups)
{
    size_t targetGroupCount = InMeshlets.size() / 4;
    if (targetGroupCount <= 1)
    {
        OutGroups.resize(1);
        OutGroups[0] = InMeshlets;
        return;
    }
    
    // generate border edges of meshlet.
    size_t numMeshlets = InMeshlets.size();
    std::vector<std::set<EdgeHash>> meshletBorderEdges;
    meshletBorderEdges.resize(numMeshlets);
    for (size_t m = 0; m < numMeshlets; m++)
    {
        GenerateBorderEdgeMap(InMeshlets[m], meshletBorderEdges[m]);
    }

    // create edge hash to meshlet map.
    std::map<EdgeHash, std::set<uint32>> edgeHashToMeshletMap;
    uint32 meshletIndex = 0;
    for (auto&& edgeHashSet : meshletBorderEdges)
    {
        for (auto hash : edgeHashSet)
        {
            edgeHashToMeshletMap[hash].insert(meshletIndex);
        }
        meshletIndex++;
    }
    
    // create graph.
    std::vector<std::vector<uint32>> linksInNodes;
    linksInNodes.resize(numMeshlets);
    for (auto&& edgeHashToMeshlet : edgeHashToMeshletMap)
    {
        auto&& meshlets = edgeHashToMeshlet.second;
        for (auto m : meshlets)
        {
            auto&& nodes = linksInNodes[m];
            for (auto om : meshlets)
            {
                if (m != om)
                {
                    nodes.push_back(om);
                }
            }
        }
    }
    
    // create adjacency.
    std::vector<idx_t> xadj;
    std::vector<idx_t> adjncy;
    std::vector<idx_t> adjwgt;
    idx_t numEdges = 0;
    for (auto&& con : linksInNodes)
    {
        xadj.push_back(numEdges);
        std::map<uint32, idx_t> edgeCountMap;
        for (auto t : con)
        {
            edgeCountMap[t]++;
        }

        for (auto edgeCounts : edgeCountMap)
        {
            adjncy.push_back(edgeCounts.first);
            adjwgt.push_back(edgeCounts.second);
            numEdges++;
        }
    }
    xadj.push_back(numEdges);

    // partition graph.
    idx_t nVertices = (idx_t)numMeshlets;
    idx_t nWeights = 1;
    idx_t nParts = (idx_t)targetGroupCount;
    std::vector<idx_t> vwgt(nVertices, 1);
    std::vector<idx_t> options(METIS_NOPTIONS);
    METIS_SetDefaultOptions(&options[0]);
    std::vector<idx_t> part(nVertices);
    idx_t objval;

    int ret = METIS_PartGraphKway(&nVertices, &nWeights, &xadj[0], &adjncy[0], &vwgt[0], NULL, &adjwgt[0], &nParts, NULL, NULL, &options[0], &objval, &part[0]);
    assert(ret == METIS_OK);

    // generate groups.
    OutGroups.clear();
    OutGroups.resize(targetGroupCount);
    for (size_t m = 0; m < numMeshlets; m++)
    {
        auto& group = OutGroups[part[m]];
        auto& src = InMeshlets[m];

        group.push_back(src);
    }
}

void SimplifyMesh(const std::vector<uint32>& InIndices, const std::vector<Vector3>& InPositions, std::vector<uint32>& OutIndices, MyBounds& OutBounds, float& OutError)
{
    // compute group bounds.
    Vector3 groupBoundSphereCenter;
    float groupBoundSphereRadius;
    ComputeBoundingSphere(InIndices, InPositions, groupBoundSphereCenter, groupBoundSphereRadius);

    // optimize vertex cache.
    std::vector<uint32> optimizedIndices;
    optimizedIndices.resize(InIndices.size());
    meshopt_optimizeVertexCache(optimizedIndices.data(), InIndices.data(), InIndices.size(), InPositions.size());

    // simplify mesh.
    std::vector<uint32> simplifyIndices;
    simplifyIndices.resize(optimizedIndices.size());
    size_t numIndices = meshopt_simplify(
        simplifyIndices.data(),
        optimizedIndices.data(), optimizedIndices.size(),
        reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(Vector3),
        optimizedIndices.size() / 2, 0.05f, meshopt_SimplifyLockBorder, &OutError);

    // compute simplify scale.
    std::vector<uint32> remap;
    remap.resize(InPositions.size());
    size_t numRemapVertices = meshopt_generateVertexRemap(remap.data(), optimizedIndices.data(), optimizedIndices.size(), reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(Vector3));
    std::vector<Vector3> remapPositions;
    remapPositions.resize(numRemapVertices);
    meshopt_remapVertexBuffer(remapPositions.data(), reinterpret_cast<const float*>(InPositions.data()), InPositions.size(), sizeof(Vector3), remap.data());
    float localScale = meshopt_simplifyScale(reinterpret_cast<const float*>(remapPositions.data()), remapPositions.size(), sizeof(Vector3));
    
    // output.
    OutIndices.clear();
    OutIndices.resize(numIndices);
    for (size_t i = 0; i < numIndices; i++)
    {
        OutIndices[i] = simplifyIndices[i];
    }
    OutBounds.sphereCenter = groupBoundSphereCenter;
    OutBounds.sphereRadius = groupBoundSphereRadius;
    OutError *= localScale;
}

void ProcessModel(std::unique_ptr<MyModel>& myModel)
{
    CurrMeshletID = 0;
    CurrGroupID = 0;

    for (auto&& mesh : myModel->meshes)
    {
        // create most detailed meshlets.
        std::vector<MyMeshlet> baseMeshlets;
        BuildMeshlets(mesh.indices, mesh.positions, 0, baseMeshlets);

        // add meshlet map.
        for (auto&& meshlet : baseMeshlets)
        {
            mesh.meshletMap[meshlet.id] = meshlet;
        }
        mesh.meshletsLOD0 = baseMeshlets;

        auto StepLOD = [&mesh](const std::vector<MyMeshlet>& InMeshlets, uint16 lod, std::vector<MyMeshlet>& OutMeshlets)
        {
            // grouping.
            std::vector<std::vector<MyMeshlet>> groups;
            GroupMeshlets(InMeshlets, groups);

            // for each group.
            OutMeshlets.clear();
            for (auto&& group : groups)
            {
                // merge meshlet.
                std::vector<uint32> mergedIndices;
                for (auto&& meshlet : group)
                {
                    mergedIndices.insert(mergedIndices.end(), meshlet.indices.begin(), meshlet.indices.end());
                }

                // mesh simplify
                std::vector<uint32> simplifyIndices;
                MyBounds groupBounds;
                float resultError;
                SimplifyMesh(mergedIndices, mesh.positions, simplifyIndices, groupBounds, resultError);

                float maxChildrenError = 0.0f;
                for (auto&& child : group)
                {
                    auto&& instance = mesh.meshletMap[child.id];
                    maxChildrenError = std::max(maxChildrenError, instance.meshletError);
                }
                float groupError = resultError + maxChildrenError;

                // build meshlets.
                std::vector<MyMeshlet> simplifyMeshlets;
                BuildMeshlets(simplifyIndices, mesh.positions, lod, simplifyMeshlets);

                // build DAG.
                for (auto&& parent : simplifyMeshlets)
                {
                    parent.meshletError = groupError;
                    parent.groupBounds = groupBounds;
                    for (auto&& child : group)
                    {
                        parent.children.push_back(child.id);
                    }
                    mesh.meshletMap[parent.id] = parent;
                    OutMeshlets.push_back(parent);
                }
                for (auto&& child : group)
                {
                    auto&& instance = mesh.meshletMap[child.id];
                    instance.parentError = groupError;
                    instance.parentBounds = groupBounds;
                    for (auto&& parent : simplifyMeshlets)
                    {
                        instance.parents.push_back(parent.id);
                    }
                }
            }
        };

        // build LODs.
        static const int kMaxLOD = 128;
        mesh.maxLOD = 0;
        for (int i = 0; i < kMaxLOD; i++)
        {
            if (baseMeshlets.size() == 1)
            {
                mesh.rootMeshletID = baseMeshlets[0].id;
                break;
            }
            mesh.maxLOD++;
            std::vector<MyMeshlet> outMeshlets;
            StepLOD(baseMeshlets, mesh.maxLOD, outMeshlets);

            baseMeshlets = outMeshlets;
        }

        // build traverse tree.
        for (auto&& pm : mesh.meshletMap)
        {
            auto& meshlet = pm.second;
            if (meshlet.parents.empty())
            {
                continue;
            }

            auto& parent = mesh.meshletMap[meshlet.parents[0]];
            parent.traverseChildren.push_back(meshlet.id);
        }

        // build meshlet indices.
        mesh.meshletIndices.clear();
        for (auto&& meshlet : mesh.meshletMap)
        {
            meshlet.second.indexBufferOffset = mesh.meshletIndices.size();
            mesh.meshletIndices.insert(mesh.meshletIndices.end(), meshlet.second.indices.begin(), meshlet.second.indices.end());
        }
    }
}
