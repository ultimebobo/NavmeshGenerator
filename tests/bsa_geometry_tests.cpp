#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/extraction/asset_cache.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "core/navmesh/generator.h"
#include "core/geometry/convex_hull.h"

#include <NifFile.hpp>
#include <bhk.hpp>
#include <lz4frame.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <set>

namespace
{
    void AppendInteger(std::string &bytes, std::uint64_t value, std::size_t width = 4)
    {
        for (std::size_t index{}; index < width; ++index)
        {
            bytes.push_back(static_cast<char>(value >> (index * 8)));
        }
    }

    // Build a named SE archive with one prefixed, LZ4-compressed synthetic model.
    void WriteModelArchive(const std::filesystem::path &path, const std::string &model)
    {
        const std::string folder("meshes\0", 7);
        const std::string filename("fixture.nif\0", 12);
        const std::string prefix = "meshes\\fixture.nif";
        std::string compressed(LZ4F_compressFrameBound(model.size(), nullptr), '\0');
        const auto compressedSize =
            LZ4F_compressFrame(compressed.data(), compressed.size(), model.data(), model.size(), nullptr);
        assert(!LZ4F_isError(compressedSize));
        compressed.resize(compressedSize);
        std::string payload(1, static_cast<char>(prefix.size()));
        payload += prefix;
        AppendInteger(payload, model.size());
        payload += compressed;
        std::string archive("BSA\0", 4);
        for (const auto field : {105U, 36U, 0x107U, 1U, 1U, 7U, 12U, 1U})
        {
            AppendInteger(archive, field);
        }
        AppendInteger(archive, 0, 8);
        AppendInteger(archive, 1);
        AppendInteger(archive, 0);
        AppendInteger(archive, 60, 8);
        archive.push_back(static_cast<char>(folder.size()));
        archive += folder;
        AppendInteger(archive, 0, 8);
        AppendInteger(archive, payload.size());
        AppendInteger(archive, archive.size() + 4 + filename.size());
        archive += filename;
        archive += payload;
        std::ofstream stream(path, std::ios::binary | std::ios::trunc);
        stream.write(archive.data(), static_cast<std::streamsize>(archive.size()));
        assert(stream.good());
    }

    void WriteCollisionModel(const std::filesystem::path &path, float halfWidth, float top)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        auto data = std::make_unique<nifly::hkPackedNiTriStripsData>();
        constexpr float havokScale = 69.99125F;
        data->numVerts = 4;
        data->compressedVertData = {{-halfWidth / havokScale, -halfWidth / havokScale, top / havokScale},
                                    {halfWidth / havokScale, -halfWidth / havokScale, top / havokScale},
                                    {halfWidth / havokScale, halfWidth / havokScale, top / havokScale},
                                    {-halfWidth / havokScale, halfWidth / havokScale, top / havokScale}};
        data->keyCount = 2;
        data->triData.resize(2);
        data->triData[0].tri = {0, 1, 2};
        data->triData[1].tri = {0, 2, 3};
        auto shape = std::make_unique<nifly::bhkPackedNiTriStripsShape>();
        shape->dataRef.index = nif.GetHeader().AddBlock(std::move(data));
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        assert(nif.Save(path) == 0);
    }

    bool HasNavigationAt(const navmesh::core::CandidateNavMesh &candidate, navmesh::core::Vec3 point)
    {
        const auto cross = [](auto a, auto b, auto c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
        for (const auto &polygon : candidate.mesh.polygons)
        {
            const auto a = candidate.mesh.vertices[polygon.vertices[0]];
            const auto b = candidate.mesh.vertices[polygon.vertices[1]];
            const auto c = candidate.mesh.vertices[polygon.vertices[2]];
            const auto area = cross(a, b, c);
            if (std::abs(area) < 0.001F)
            {
                continue;
            }
            const auto u = cross(point, b, c) / area;
            const auto v = cross(a, point, c) / area;
            const auto w = 1 - u - v;
            if (u >= -0.001F && v >= -0.001F && w >= -0.001F && std::abs(a.z * u + b.z * v + c.z * w - point.z) < 8)
            {
                return true;
            }
        }
        return false;
    }

    void CheckConvexHulls()
    {
        using namespace navmesh::core;
        const std::vector<Vec3> corners = {{-2, -3, -4}, {2, -3, -4}, {2, 3, -4}, {-2, 3, -4},
                                           {-2, -3, 4},  {2, -3, 4},  {2, 3, 4},  {-2, 3, 4}};
        for (const float scale : {0.001F, 1.0F, 1000.0F})
        {
            auto points = corners;
            points.push_back({0, 0, 0});
            points.push_back(corners.front());
            points.push_back({0, 3, 0});
            std::reverse(points.begin(), points.end());
            const Vec3 translation{11 * scale, -23 * scale, 37 * scale};
            for (auto &point : points)
            {
                // A skewed box exercises non-axis-aligned faces without changing its known volume.
                point.x += point.z / 2;
                point = point * scale + translation;
            }
            const auto hull = TriangulateConvexHull(points);
            assert(hull && !hull->empty());
            std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
            double volume = 0;
            for (const auto &triangle : *hull)
            {
                const auto a = points[triangle.vertices[0]] - translation;
                const auto b = points[triangle.vertices[1]] - translation;
                const auto c = points[triangle.vertices[2]] - translation;
                const auto cross = [](Vec3 lhs, Vec3 rhs)
                {
                    return Vec3{lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
                                lhs.x * rhs.y - lhs.y * rhs.x};
                };
                const auto dot = [](Vec3 lhs, Vec3 rhs)
                {
                    return static_cast<double>(lhs.x) * rhs.x + static_cast<double>(lhs.y) * rhs.y +
                           static_cast<double>(lhs.z) * rhs.z;
                };
                volume += dot(a, cross(b, c)) / 6;
                const auto normal = cross(b - a, c - a);
                for (const auto &point : points)
                {
                    assert(dot(normal, point - translation - a) <= 0.001 * scale * scale * scale);
                }
                for (std::size_t edge = 0; edge < 3; ++edge)
                {
                    assert(edges.emplace(triangle.vertices[edge], triangle.vertices[(edge + 1) % 3]).second);
                }
            }
            for (const auto &[a, b] : edges)
            {
                assert(edges.contains({b, a}));
            }
            const auto expected = 192.0 * scale * scale * scale;
            assert(std::abs(volume - expected) < expected * 0.0001);
        }
        assert(!TriangulateConvexHull(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}));
        assert(!TriangulateConvexHull(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}}));
        assert(!TriangulateConvexHull(std::vector<Vec3>{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}}));
        auto invalid = corners;
        invalid[0].x = std::numeric_limits<float>::quiet_NaN();
        assert(!TriangulateConvexHull(invalid));
        // Almost coplanar exterior samples must retain a closed hull instead of producing duplicate horizon edges.
        const std::vector<Vec3> precise = {{-1, -1, -1},
                                           {1, -1, -1},
                                           {1, 1, -1},
                                           {-1, 1, -1},
                                           {-1, -1, 1},
                                           {1, -1, 1},
                                           {1, 1, 1},
                                           {-1, 1, 1},
                                           {1.00000024F, -0.389362335F, 0.0772417784F},
                                           {1.0000006F, -0.648997784F, -0.790054202F},
                                           {1.00000036F, -0.268090487F, 0.310449958F}};
        const auto preciseHull = TriangulateConvexHull(precise);
        assert(preciseHull && !preciseHull->empty());
    }

    void WriteConvexModel(const std::filesystem::path &path, bool solid = true, bool flat = false,
                          std::uint8_t layer = 1, nifly::hkResponseType response = nifly::RESPONSE_SIMPLE_CONTACT)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        auto shape = std::make_unique<nifly::bhkConvexVerticesShape>();
        constexpr float units = 69.99125F;
        const float top = flat ? 0 : 24 / units;
        // Duplicate and interior samples must not produce overlapping triangles or change the collision envelope.
        const std::vector<nifly::Vector4> vertices = {
            {-64 / units, -64 / units, 0, 0}, {64 / units, -64 / units, 0, 0},    {64 / units, 64 / units, 0, 0},
            {-64 / units, 64 / units, 0, 0},  {-64 / units, -64 / units, top, 0}, {64 / units, -64 / units, top, 0},
            {64 / units, 64 / units, top, 0}, {-64 / units, 64 / units, top, 0},  {0, 0, top / 2, 0},
            {-64 / units, -64 / units, 0, 0}};
        shape->verts.resize(static_cast<std::uint32_t>(vertices.size()));
        for (std::uint32_t index = 0; index < vertices.size(); ++index)
        {
            shape->verts[index] = vertices[index];
        }
        auto list = std::make_unique<nifly::bhkConvexListShape>();
        list->shapeRefs.AddBlockRef(nif.GetHeader().AddBlock(std::move(shape)));
        auto body = std::make_unique<nifly::bhkRigidBodyT>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(list));
        body->collisionFilter.layer = layer;
        body->collisionFilterCopy.layer = layer;
        body->collisionResponse = response;
        body->translation = {16 / units, 8 / units, 0, 0};
        body->rotation = {0, 0, std::sqrt(0.5F), std::sqrt(0.5F)};
        if (!solid)
        {
            body->collisionResponse = nifly::RESPONSE_NONE;
        }
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        // Visual geometry deliberately exceeds collision so fallback cannot accidentally satisfy obstruction checks.
        const std::vector<nifly::Vector3> visual = {{-512, -512, 0}, {512, -512, 0}, {0, 512, 64}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}};
        assert(nif.CreateShapeFromData("Display", &visual, &faces, nullptr));
        assert(nif.Save(path) == 0);
    }

    /// Attach a physical shape to a target node; stored transforms on non-T bodies are deliberately inactive.
    void AttachCollision(nifly::NifFile &nif, nifly::NiNode *target, std::uint32_t shape, bool transformed = false,
                         bool solid = true)
    {
        std::unique_ptr<nifly::bhkRigidBody> body =
            transformed ? std::make_unique<nifly::bhkRigidBodyT>() : std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = shape;
        body->translation = {91, 72, 53, 0};
        body->rotation = {0, 0, 1, 0};
        body->collisionResponse = solid ? nifly::RESPONSE_SIMPLE_CONTACT : nifly::RESPONSE_NONE;
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(target);
        target->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
    }

    /// Unpacked strips store model-unit vertices inside a Havok-unit transform hierarchy.
    void CheckStripCollisionUnits(const std::filesystem::path &root)
    {
        using namespace navmesh;
        constexpr float units = 69.99125F;
        const std::vector<nifly::Vector3> vertices = {{0, 0, 0}, {256, 0, 0}, {0, 256, 0}, {0, 0, 512}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {2, 1, 3}};
        for (const bool transformed : {false, true})
        {
            nifly::NifFile nif;
            nif.Create({nifly::V20_2_0_7, 12, 130});
            nif.GetRootNode()->transform.scale = 10;
            auto data = std::make_unique<nifly::NiTriStripsData>();
            data->Create(nif.GetHeader().GetVersion(), &vertices, &faces, nullptr, nullptr);
            data->stripsInfo.stripLengths.resize(1);
            data->stripsInfo.stripLengths[0] = 4;
            data->stripsInfo.points = {{0, 1, 2, 3}};
            auto shape = std::make_unique<nifly::bhkNiTriStripsShape>();
            shape->partRefs.AddBlockRef(nif.GetHeader().AddBlock(std::move(data)));
            auto wrapper = std::make_unique<nifly::bhkTransformShape>();
            wrapper->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
            wrapper->xform[12] = 1;
            wrapper->xform[13] = 2;
            wrapper->xform[14] = 3;
            nifly::MatTransform targetTransform;
            targetTransform.Clear();
            targetTransform.translation = {32, 64, 96};
            targetTransform.scale = 2;
            auto *target = nif.AddNode("StripTarget", targetTransform, nif.GetRootNode());
            AttachCollision(nif, target, nif.GetHeader().AddBlock(std::move(wrapper)), transformed);
            assert(nif.Save(root / "meshes/strip-collision.nif") == 0);
            core::Cell cell;
            cell.references = {{.id = 1,
                                .recordType = "REFR",
                                .modelPath = "strip-collision.nif",
                                .position = {1000, 2000, 3000},
                                .rotation = {0, 0, 1.57079632679F},
                                .scale = 0.5F,
                                .baseRecordType = "STAT"}};
            for (const bool navigationOnly : {false, true})
            {
                const auto geometry =
                    skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
                assert(geometry.collisionTriangles == faces.size());
                assert(geometry.scene.mesh.vertices.size() == vertices.size());
                assert(geometry.scene.HasCompleteTriangleProvenance());
                for (std::size_t index{}; index < vertices.size(); ++index)
                {
                    const auto input = vertices[index];
                    const auto actual = geometry.scene.mesh.vertices[index];
                    const float x = transformed ? 90 * units - input.x : input.x + units;
                    const float y = transformed ? 70 * units - input.y : input.y + 2 * units;
                    const float z = input.z + (transformed ? 56 : 3) * units;
                    assert(std::abs(actual.x - (1000 + y + 32)) < 0.01F);
                    assert(std::abs(actual.y - (2000 - x - 16)) < 0.01F);
                    assert(std::abs(actual.z - (3000 + z + 48)) < 0.01F);
                }
                core::ValidateRecastSceneHeightRange(geometry.scene, {}, {}, {});
            }
        }
    }

    void CheckCollisionShapes(const std::filesystem::path &root)
    {
        using namespace navmesh;
        constexpr float units = 69.99125F;
        WriteCollisionModel(root / "meshes/shape-floor.nif", 768, 0);
        for (const float radius : {64.0F / units, 0.0F, -1.0F, std::numeric_limits<float>::quiet_NaN()})
        {
            for (const bool solid : {true, false})
            {
                nifly::NifFile nif;
                nif.Create({nifly::V20_2_0_7, 12, 130});
                auto capsule = std::make_unique<nifly::bhkCapsuleShape>();
                capsule->point1 = {0, 0, 0};
                capsule->point2 = {4, 0, 0};
                capsule->radius = radius;
                capsule->radius1 = radius;
                capsule->radius2 = radius;
                // A node rotates the Havok X-axis into model Z, as in tree skeleton collision.
                nifly::MatTransform targetTransform;
                targetTransform.Clear();
                auto *target = nif.AddNode("CapsuleTarget", targetTransform, nif.GetRootNode());
                target->transform.rotation[0] = {0, 0, -1};
                target->transform.rotation[1] = {0, 1, 0};
                target->transform.rotation[2] = {1, 0, 0};
                AttachCollision(nif, target, nif.GetHeader().AddBlock(std::move(capsule)), false, solid);
                assert(nif.Save(root / "meshes/capsule.nif") == 0);
                core::Cell cell;
                cell.references = {
                    {.id = 1, .recordType = "REFR", .modelPath = "shape-floor.nif", .baseRecordType = "STAT"},
                    {.id = 2, .recordType = "REFR", .modelPath = "capsule.nif", .baseRecordType = "TREE"}};
                for (const bool navigationOnly : {false, true})
                {
                    const auto geometry =
                        skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
                    const bool valid = solid && radius > 0 && std::isfinite(radius);
                    assert(geometry.collisionModelsLoaded == (valid ? 2 : 1));
                    if (!valid)
                    {
                        continue;
                    }
                    assert(geometry.references[1].collisionType == "bhkCapsuleShape");
                    core::AABB bounds;
                    const auto &report = geometry.references[1];
                    for (std::size_t i = report.meshVertexOffset; i < report.meshVertexOffset + report.vertices; ++i)
                    {
                        bounds.Expand(geometry.scene.mesh.vertices[i]);
                    }
                    assert(std::abs(bounds.min.x + 64) < 0.01F && std::abs(bounds.max.x - 64) < 0.01F);
                    assert(std::abs(bounds.min.z + 64) < 0.01F && std::abs(bounds.max.z - (4 * units + 64)) < 0.01F);
                    for (const auto partitioning :
                         {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                          core::RegionPartitioningAlgorithm::Layers})
                    {
                        const auto candidate = core::RecastCandidateGenerator{}.Generate(
                            geometry.scene, {}, std::nullopt, {}, partitioning, {},
                            core::CandidateRetention::AllWalkable);
                        assert(candidate.topology.valid);
                        assert(!HasNavigationAt(candidate, {0, 0, 0}));
                        assert(HasNavigationAt(candidate, {256, 0, 0}));
                    }
                }
            }
        }

        // A shared box under two wrappers must appear at both placements, after target ancestry and reference rotation.
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        nif.GetRootNode()->transform.translation = {100, 200, 0};
        nif.GetRootNode()->transform.scale = 2;
        auto *ancestor = nif.AddNode("CollisionAncestor", nif.GetRootNode()->transform, nif.GetRootNode());
        nifly::MatTransform nodeTransform;
        nodeTransform.Clear();
        nodeTransform.translation = {32, 16, 0};
        nodeTransform.rotation[0] = {0, -1, 0};
        nodeTransform.rotation[1] = {1, 0, 0};
        auto *target = nif.AddNode("CollisionTarget", nodeTransform, ancestor);
        auto box = std::make_unique<nifly::bhkBoxShape>();
        box->dimensions = {32 / units, 16 / units, 64 / units};
        const auto boxId = nif.GetHeader().AddBlock(std::move(box));
        auto list = std::make_unique<nifly::bhkListShape>();
        for (const float translation : {100.0F, -100.0F})
        {
            auto wrapper = std::make_unique<nifly::bhkConvexTransformShape>();
            wrapper->shapeRef.index = boxId;
            wrapper->xform[12] = translation / units;
            list->subShapeRefs.AddBlockRef(nif.GetHeader().AddBlock(std::move(wrapper)));
        }
        AttachCollision(nif, target, nif.GetHeader().AddBlock(std::move(list)));
        assert(nif.Save(root / "meshes/instanced-box.nif") == 0);
        core::Cell cell;
        cell.references = {{.id = 1,
                            .recordType = "REFR",
                            .modelPath = "instanced-box.nif",
                            .position = {1000, 2000, 300},
                            .rotation = {0, 0, 1.57079632679F},
                            .scale = 0.5F,
                            .baseRecordType = "STAT"}};
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
            assert(geometry.collisionTriangles == 24 && geometry.scene.mesh.vertices.size() == 16);
            for (std::size_t instance = 0; instance < 2; ++instance)
            {
                core::AABB bounds;
                for (std::size_t i = instance * 8; i < (instance + 1) * 8; ++i)
                {
                    bounds.Expand(geometry.scene.mesh.vertices[i]);
                }
                const float centerX = instance == 0 ? 1216.0F : 1016.0F;
                assert(std::abs(bounds.min.x - (centerX - 32)) < 0.01F);
                assert(std::abs(bounds.max.x - (centerX + 32)) < 0.01F);
                assert(std::abs(bounds.min.y - 1902) < 0.01F && std::abs(bounds.max.y - 1934) < 0.01F);
                assert(std::abs(bounds.min.z - 236) < 0.01F && std::abs(bounds.max.z - 364) < 0.01F);
            }
        }

        // Chunk rotations precede chunk translation, body transforms and target transforms.
        nifly::NifFile compressed;
        compressed.Create({nifly::V20_2_0_7, 12, 130});
        auto data = std::make_unique<nifly::bhkCompressedMeshShapeData>();
        data->error = 1;
        data->transforms.resize(2);
        data->chunks.resize(2);
        for (std::uint16_t i = 0; i < 2; ++i)
        {
            data->transforms[i].translation = {i == 0 ? 4.0F : -4.0F, 2, 0, 0};
            data->transforms[i].rotation = {0, 0, std::sqrt(0.5F), std::sqrt(0.5F)};
            auto &chunk = data->chunks[i];
            chunk.reference = 0xFFFF;
            chunk.transformIndex = i;
            chunk.translation = {1, 0, 0, 0};
            chunk.verts.resize(9);
            chunk.verts[3] = 1;
            chunk.verts[7] = 1;
            chunk.indices.resize(3);
            chunk.indices[1] = 1;
            chunk.indices[2] = 2;
        }
        auto shape = std::make_unique<nifly::bhkCompressedMeshShape>();
        shape->dataRef.index = compressed.GetHeader().AddBlock(std::move(data));
        AttachCollision(compressed, compressed.GetRootNode(), compressed.GetHeader().AddBlock(std::move(shape)));
        assert(compressed.Save(root / "meshes/chunks.nif") == 0);
        cell.references[0] = {.id = 1, .recordType = "REFR", .modelPath = "chunks.nif", .baseRecordType = "STAT"};
        const auto chunks = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(chunks.collisionTriangles == 2 && chunks.scene.mesh.vertices.size() == 6);
        for (std::size_t i = 0; i < 2; ++i)
        {
            const auto vertex = chunks.scene.mesh.vertices[i * 3];
            assert(std::abs(vertex.x - (i == 0 ? 4 : -4) * units) < 0.01F);
            assert(std::abs(vertex.y - 3 * units) < 0.01F);
        }
    }

    void CheckPlacedModelTransforms(const std::filesystem::path &root)
    {
        using namespace navmesh;
        constexpr float units = 69.99125F;
        WriteCollisionModel(root / "meshes/placement-floor.nif", 512, 0);
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        // A root rotation would turn this side wall across the corridor and disconnect the entrance.
        auto &rootTransform = nif.GetRootNode()->transform;
        rootTransform.rotation[0] = {0, -1, 0};
        rootTransform.rotation[1] = {1, 0, 0};
        nifly::MatTransform targetTransform;
        targetTransform.Clear();
        targetTransform.translation = {256, 0, 128};
        auto *target = nif.AddNode("WallTarget", targetTransform, nif.GetRootNode());
        auto box = std::make_unique<nifly::bhkBoxShape>();
        box->dimensions = {8 / units, 512 / units, 128 / units};
        AttachCollision(nif, target, nif.GetHeader().AddBlock(std::move(box)));
        const std::vector<nifly::Vector3> vertices = {{0, -512, -128}, {0, 512, -128}, {0, 512, 128}, {0, -512, 128}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {0, 2, 3}};
        auto *display = nif.CreateShapeFromData("WallDisplay", &vertices, &faces, nullptr);
        assert(display);
        display->transform = targetTransform;
        assert(nif.Save(root / "meshes/placement-wall.nif") == 0);
        core::Cell cell;
        cell.references = {
            {.id = 1, .recordType = "REFR", .modelPath = "placement-floor.nif", .baseRecordType = "STAT"},
            {.id = 2, .recordType = "REFR", .modelPath = "placement-wall.nif", .baseRecordType = "STAT"}};
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
            assert(geometry.collisionModelsLoaded == 2);
            if (!navigationOnly)
            {
                const auto &mesh = geometry.scene.renderFallbackMesh;
                assert(mesh.vertices.size() == vertices.size());
                core::AABB bounds;
                for (const auto &vertex : mesh.vertices)
                {
                    bounds.Expand(vertex);
                }
                assert(bounds.min.x == 256 && bounds.max.x == 256);
                assert(bounds.min.y == -512 && bounds.max.y == 512 && bounds.min.z == 0 && bounds.max.z == 256);
            }
            for (const auto partitioning :
                 {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                  core::RegionPartitioningAlgorithm::Layers})
            {
                const auto candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, {}, std::nullopt, {{.referenceId = 3, .position = {0, -384, 0}}}, partitioning);
                assert(candidate.topology.valid && candidate.regions.size() == 1 && candidate.exits[0].polygon);
                assert(HasNavigationAt(candidate, {0, -384, 0}) && HasNavigationAt(candidate, {0, 384, 0}));
            }
        }
    }

    void CheckFixedClutter(const std::filesystem::path &root)
    {
        using namespace navmesh;
        CheckConvexHulls();
        std::filesystem::create_directories(root / "meshes/clutter/common");
        WriteCollisionModel(root / "meshes/floor.nif", 768, 0);
        WriteCollisionModel(root / "meshes/clutter/barrel.nif", 64, 24);
        WriteConvexModel(root / "meshes/clutter/common/crate.nif");
        WriteConvexModel(root / "meshes/clutter/common/sack.nif");
        core::Cell cell;
        cell.references = {{.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
                           {.id = 2,
                            .recordType = "REFR",
                            .modelPath = "MESHES\\CLUTTER\\Barrel.nif",
                            .position = {-256, 0, 0},
                            .baseRecordType = "STAT"},
                           {.id = 3,
                            .recordType = "REFR",
                            .modelPath = "clutter/common/crate.nif",
                            .position = {256, 0, 0},
                            .rotation = {0, 0, 1.57079632679F},
                            .scale = 2,
                            .baseRecordType = "STAT"},
                           {.id = 4,
                            .recordType = "REFR",
                            .modelPath = "clutter/common/sack.nif",
                            .position = {0, -256, 0},
                            .baseRecordType = "CONT"},
                           {.id = 5,
                            .recordType = "REFR",
                            .modelPath = "clutter/common/crate.nif",
                            .position = {0, 256, 0},
                            .baseRecordType = "MISC"}};
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 4 && geometry.modelsExcluded == 1);
            assert(geometry.collisionModelsLoaded == 4 && geometry.collisionTriangles == 28);
            assert(geometry.scene.HasCompleteTriangleProvenance());
            for (const auto &source : geometry.scene.geometrySources)
            {
                assert(source.reference.formId != 5);
                if (source.sourceType == core::GeometrySourceType::Collision)
                {
                    assert(source.navigationObstacle == (source.reference.formId != 1));
                }
            }
            assert(geometry.references[2].collisionType == "bhkConvexVerticesShape");
            assert(!geometry.references[2].usedRenderFallback);
            core::AABB bounds;
            const auto &crate = geometry.references[2];
            for (std::size_t index = crate.meshVertexOffset; index < crate.meshVertexOffset + crate.vertices; ++index)
            {
                bounds.Expand(geometry.scene.mesh.vertices[index]);
            }
            assert(std::abs(bounds.min.x - 144) < 0.01F && std::abs(bounds.max.x - 400) < 0.01F);
            assert(std::abs(bounds.min.y + 160) < 0.01F && std::abs(bounds.max.y - 96) < 0.01F);
            assert(std::abs(bounds.max.z - 48) < 0.01F);
            for (const auto partitioning :
                 {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                  core::RegionPartitioningAlgorithm::Layers})
            {
                const auto candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, {}, std::nullopt, {}, partitioning, {}, core::CandidateRetention::AllWalkable);
                assert(candidate.topology.valid);
                assert(!HasNavigationAt(candidate, {-256, 0, 0}));
                assert(!HasNavigationAt(candidate, {-256, 0, 24}));
                assert(!HasNavigationAt(candidate, {272, -32, 0}));
                assert(!HasNavigationAt(candidate, {272, -32, 48}));
                assert(!HasNavigationAt(candidate, {16, -248, 0}));
                assert(HasNavigationAt(candidate, {0, 256, 0}));
                assert(HasNavigationAt(candidate, {-512, 0, 0}));
            }
        }
        // Non-solid and degenerate hulls must not borrow display geometry in navigation-only extraction.
        WriteConvexModel(root / "meshes/clutter/common/nonsolid.nif", false);
        WriteConvexModel(root / "meshes/clutter/common/flat.nif", true, true);
        cell.references.resize(1);
        cell.references[0].modelPath = "clutter/common/nonsolid.nif";
        const auto nonSolid = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(nonSolid.modelsExcluded == 1 && nonSolid.collisionTriangles == 0);
        cell.references[0].modelPath = "clutter/common/flat.nif";
        const auto flat = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(flat.modelsUnsupported == 1 && flat.collisionTriangles == 0);
    }

    void CheckCollisionEligibility(const std::filesystem::path &root)
    {
        using namespace navmesh;
        std::filesystem::create_directories(root / "meshes/plants");
        core::Cell cell;
        cell.references = {
            {.id = 1, .recordType = "REFR", .modelPath = "plants/fixture.nif", .baseRecordType = "FLOR"}};
        // A geometric shape with normal contact response can still be authored as a nonblocking volume.
        for (const std::uint8_t layer :
             {11, 12, 15, 21, 22, 23, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 51, 53, 54})
        {
            WriteConvexModel(root / "meshes/plants/fixture.nif", true, false, layer);
            for (const bool navigationOnly : {false, true})
            {
                const auto geometry =
                    skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
                assert(geometry.modelsExcluded == 1 && geometry.collisionTriangles == 0);
                assert(geometry.scene.mesh.triangles.empty() && geometry.scene.renderFallbackMesh.triangles.empty());
                assert(geometry.scene.coverage[0].status == core::GeometryCoverage::Excluded);
            }
        }
        for (const auto response : {nifly::RESPONSE_NONE, nifly::RESPONSE_REPORTING, nifly::RESPONSE_INVALID})
        {
            WriteConvexModel(root / "meshes/plants/fixture.nif", true, false, 1, response);
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
            assert(geometry.modelsExcluded == 1 && geometry.collisionTriangles == 0);
        }
        // Real solid collision on a flora record remains eligible; record names never decide solidity.
        WriteConvexModel(root / "meshes/plants/fixture.nif");
        const auto solid = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(solid.collisionModelsLoaded == 1 && solid.collisionTriangles == 12);

        // The layer filter applies to packed meshes too, and keeps a solid sibling in a mixed NIF.
        WriteCollisionModel(root / "meshes/plants/packed.nif", 64, 24);
        nifly::NifFile packed(root / "meshes/plants/packed.nif");
        for (std::uint32_t block = 0; block < packed.GetHeader().GetNumBlocks(); ++block)
        {
            if (auto *body = packed.GetHeader().GetBlock<nifly::bhkRigidBody>(block))
            {
                body->collisionFilter.layer = 15;
                body->collisionFilterCopy.layer = 15;
            }
        }
        assert(packed.Save(root / "meshes/plants/packed.nif") == 0);
        cell.references[0].modelPath = "plants/packed.nif";
        const auto ignoredPacked = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(ignoredPacked.modelsExcluded == 1 && ignoredPacked.collisionTriangles == 0);
        nifly::bhkPackedNiTriStripsShape *solidShape = nullptr;
        for (std::uint32_t block = 0; block < packed.GetHeader().GetNumBlocks(); ++block)
        {
            if (auto *body = packed.GetHeader().GetBlock<nifly::bhkRigidBody>(block);
                body && body->collisionFilter.layer == 15)
            {
                body->collisionFilter.layer = 1;
                body->collisionFilterCopy.layer = 1;
                solidShape = packed.GetHeader().GetBlock<nifly::bhkPackedNiTriStripsShape>(body->shapeRef);
            }
        }
        assert(solidShape);
        auto queryShape = std::make_unique<nifly::bhkPackedNiTriStripsShape>(*solidShape);
        auto query = std::make_unique<nifly::bhkRigidBody>();
        query->collisionFilter.layer = 40;
        query->collisionFilterCopy.layer = 40;
        query->shapeRef.index = packed.GetHeader().AddBlock(std::move(queryShape));
        auto queryObject = std::make_unique<nifly::bhkCollisionObject>();
        queryObject->bodyRef.index = packed.GetHeader().AddBlock(std::move(query));
        queryObject->targetRef.index = packed.GetBlockID(packed.GetRootNode());
        packed.GetHeader().AddBlock(std::move(queryObject));
        assert(packed.Save(root / "meshes/plants/mixed.nif") == 0);
        cell.references[0].modelPath = "plants/mixed.nif";
        const auto mixed = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, true);
        assert(mixed.collisionModelsLoaded == 1 && mixed.collisionTriangles == 2);

        // A static clutter tag cannot turn a purely visual mesh into supported collision or block the floor.
        nifly::NifFile visual;
        visual.Create({nifly::V20_2_0_7, 12, 130});
        const std::vector<nifly::Vector3> vertices = {{-128, -128, 24}, {128, -128, 24}, {0, 128, 24}};
        const std::vector<nifly::Triangle> triangles = {{0, 1, 2}};
        assert(visual.CreateShapeFromData("Display", &vertices, &triangles, nullptr));
        assert(visual.Save(root / "meshes/clutter/display.nif") == 0);
        cell.references = {
            {.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
            {.id = 2, .recordType = "REFR", .modelPath = "clutter/display.nif", .baseRecordType = "STAT"}};
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, nullptr, navigationOnly);
            assert(geometry.collisionTriangles == 2 && geometry.collisionModelsLoaded == 1);
            const auto candidate = core::RecastCandidateGenerator{}.Generate(
                geometry.scene, {}, std::nullopt, {}, core::RegionPartitioningAlgorithm::Watershed, {},
                core::CandidateRetention::AllWalkable);
            assert(candidate.topology.valid && HasNavigationAt(candidate, {0, 0, 0}));
            assert(!HasNavigationAt(candidate, {0, 0, 24}));
        }
    }

    // Exercise the extraction policy and the neutral generator together. Low furniture
    // and an overlapping fuel bed would be climbable without obstacle-only evidence.
    void CheckFurnitureAndFire(const std::filesystem::path &root)
    {
        using namespace navmesh;
        std::filesystem::create_directories(root / "meshes/effects");
        std::filesystem::create_directories(root / "meshes/furniture");
        WriteCollisionModel(root / "meshes/floor.nif", 768, 0);
        WriteCollisionModel(root / "meshes/furniture/fixture.nif", 64, 24);
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        nif.GetRootNode()->transform.translation = {32, 16, 0};
        const std::vector<nifly::Vector3> fuel = {{-64, -64, 0}, {64, -64, 0}, {64, 64, 8}, {-64, 64, 8}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {0, 2, 3}};
        auto *bed = nif.CreateShapeFromData("L2_CoalsBed", &fuel, &faces, nullptr);
        assert(bed);
        bed->transform.translation = {16, 8, 0};
        auto *fuelParent = nif.AddNode("FuelParent", nif.GetRootNode()->transform, nif.GetRootNode());
        nif.SetParentNode(bed, fuelParent);
        const std::vector<nifly::Vector3> glow = {{-512, -512, 0}, {512, -512, 0}, {512, 512, 512}};
        const std::vector<nifly::Triangle> glowFaces = {{0, 1, 2}};
        assert(nif.CreateShapeFromData("glow", &glow, &glowFaces, nullptr));
        assert(nif.Save(root / "meshes/effects/fxfirefixture.nif") == 0);

        core::Cell cell;
        cell.references = {{.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
                           {.id = 2,
                            .recordType = "REFR",
                            .modelPath = "Furniture/fixture.nif",
                            .position = {-256, 0, 0},
                            .baseRecordType = "FURN"},
                           {.id = 3,
                            .recordType = "REFR",
                            .modelPath = "MESHES/Effects/FXFireFixture.nif",
                            .position = {256, 0, -2},
                            .rotation = {0, 0, 1.57079632679F},
                            .scale = 2,
                            .baseRecordType = "STAT"}};
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 3 && geometry.modelsExcluded == 0);
            assert(geometry.collisionModelsLoaded == 2 && geometry.scene.HasCompleteTriangleProvenance());
            // Display mode appends a visual source after each exclusion.
            const auto source = std::find_if(
                geometry.scene.geometrySources.begin(), geometry.scene.geometrySources.end(),
                [](const auto &value) { return value.sourceType == core::GeometrySourceType::NavigationExclusion; });
            assert(source != geometry.scene.geometrySources.end() && source->navigationObstacle);
            assert(geometry.references[2].sourceType == "navigation_exclusion" &&
                   !geometry.references[2].usedRenderFallback);
            assert(geometry.references[2].triangles == 12);
            const auto offset = geometry.references[2].meshVertexOffset;
            core::AABB bounds;
            for (std::size_t index = offset; index < offset + 8; ++index)
            {
                bounds.Expand(geometry.scene.mesh.vertices[index]);
            }
            assert(std::abs(bounds.min.x - 176) < 0.01F && std::abs(bounds.max.x - 432) < 0.01F);
            assert(std::abs(bounds.min.y + 224) < 0.01F && std::abs(bounds.max.y - 32) < 0.01F);
            assert(std::abs(bounds.min.z + 2) < 0.01F && std::abs(bounds.max.z - 14) < 0.01F);
            for (const auto partitioning :
                 {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                  core::RegionPartitioningAlgorithm::Layers})
            {
                const auto candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, {}, std::nullopt, {}, partitioning, {}, core::CandidateRetention::AllWalkable);
                assert(candidate.topology.valid);
                assert(!HasNavigationAt(candidate, {-256, 0, 24}));
                assert(!HasNavigationAt(candidate, {-256, 0, 0}));
                assert(!HasNavigationAt(candidate, {304, -96, 0}));
                assert(!HasNavigationAt(candidate, {304, -96, 14}));
                assert(HasNavigationAt(candidate, {0, 256, 0}));
                assert(HasNavigationAt(candidate, {304, 256, 0}));
            }
        }
        // Runtime sweeps retain furniture and fire policy while covering every possible position.
        auto uncertain = cell;
        for (auto &reference : uncertain.references)
        {
            if (reference.id == 1)
            {
                continue;
            }
            reference.inputConfig = "Synthetic_SWAP.ini:1";
            reference.envelope = core::PlacementEnvelope{reference.position,
                                                         reference.position + core::Vec3{1024, 0, 0}, reference.scale};
        }
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry =
                skyrim::ExtractGeometry(root, uncertain, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 3 && geometry.modelsExcluded == 0);
            for (const auto &source : geometry.scene.geometrySources)
            {
                if (source.reference.formId == 1)
                {
                    continue;
                }
                assert(source.navigationObstacle && source.solidBounds);
                assert(source.solidBounds->max.x - source.solidBounds->min.x > 1024);
                assert(source.inputConfig == "Synthetic_SWAP.ini:1");
                assert(source.sourceType != core::GeometrySourceType::RenderFallback);
            }
        }
        for (const auto *type : {"FURN", "CONT", "STAT"})
        {
            cell.references.resize(2);
            cell.references[1].baseRecordType = type;
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
            assert(geometry.scene.geometrySources[1].navigationObstacle);
        }
        cell.references[1].modelPath = "effects/DecorativeGlow.nif";
        cell.references[1].baseRecordType = "STAT";
        const auto decorative = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(decorative.modelsExcluded == 1 && decorative.modelsMissing == 0);

        // A flame-only asset still defines an exclusion; an effect without fuel or
        // flame triangles remains unsupported rather than borrowing billboard bounds.
        nifly::NifFile flame;
        flame.Create({nifly::V20_2_0_7, 12, 130});
        assert(flame.CreateShapeFromData("Flames", &fuel, &faces, nullptr));
        assert(flame.Save(root / "meshes/effects/fxfireflame.nif") == 0);
        cell.references[1].modelPath = "effects/FXFireFlame.nif";
        const auto flameOnly = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(flameOnly.modelsLoaded == 2 && flameOnly.references[1].collisionType == "fire flame bounds");
        nifly::NifFile emptyFire;
        emptyFire.Create({nifly::V20_2_0_7, 12, 130});
        assert(emptyFire.CreateShapeFromData("glow", &glow, &glowFaces, nullptr));
        assert(emptyFire.Save(root / "meshes/effects/fxfireglow.nif") == 0);
        cell.references[1].modelPath = "effects/FXFireGlow.nif";
        const auto unsupported = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(unsupported.modelsUnsupported == 1 && unsupported.modelsLoaded == 1);
        assert(unsupported.scene.coverage.back().status == core::GeometryCoverage::Unsupported);
    }

    // Rectangular fuel bounds ignore decorative flames and unused vertices.
    void CheckFireFootprints(const std::filesystem::path &root)
    {
        using namespace navmesh;
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        nif.GetRootNode()->transform.translation = {32, 16, 0};
        const std::vector<nifly::Vector3> fuel = {{256, 0, 0},  {181, 181, 0},     {0, 256, 8},  {-181, 181, 8},
                                                  {-256, 0, 8}, {-181, -181, 8},   {0, -256, 0}, {181, -181, 0},
                                                  {0, 0, 4},    {2048, 2048, 2048}};
        const std::vector<nifly::Triangle> faces = {{8, 0, 1}, {8, 1, 2}, {8, 2, 3}, {8, 3, 4}, {8, 4, 5},
                                                    {8, 5, 6}, {8, 6, 7}, {8, 7, 0}, {9, 9, 9}};
        auto *bed = nif.CreateShapeFromData("EmberBed", &fuel, &faces, nullptr);
        assert(bed);
        bed->transform.translation = {8, 4, 0};
        nifly::MatTransform parentTransform;
        parentTransform.Clear();
        parentTransform.translation = {16, 8, 0};
        auto *ancestor = nif.AddNode("FuelAncestor", nif.GetRootNode()->transform, nif.GetRootNode());
        auto *parent = nif.AddNode("FuelParent", parentTransform, ancestor);
        assert(parent);
        nif.SetParentNode(bed, parent);
        const std::vector<nifly::Vector3> flames = {{-512, -512, 0}, {512, -512, 0}, {0, 512, 512}};
        const std::vector<nifly::Triangle> flameFaces = {{0, 1, 2}};
        assert(nif.CreateShapeFromData("Flames", &flames, &flameFaces, nullptr));
        assert(nif.Save(root / "meshes/effects/fxfirebed.nif") == 0);

        core::Cell cell;
        cell.references = {{.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
                           {.id = 2,
                            .recordType = "REFR",
                            .modelPath = "Effects/FXFireBed.nif",
                            .position = {-56, -28, -2},
                            .baseRecordType = "MSTT"}};
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 2 && geometry.scene.HasCompleteTriangleProvenance());
            const auto &report = geometry.references[1];
            assert(report.sourceType == "navigation_exclusion" && report.collisionType == "fire fuel bounds");
            assert(report.vertices == 8 && report.triangles == 12 && report.degenerateTriangles == 1);
            core::AABB bounds;
            for (std::size_t index = report.meshVertexOffset; index < report.meshVertexOffset + report.vertices;
                 ++index)
            {
                const auto &vertex = geometry.scene.mesh.vertices[index];
                bounds.Expand(vertex);
            }
            assert(bounds.min.x == -256 && bounds.max.x == 256 && bounds.min.y == -256 && bounds.max.y == 256);
            assert(bounds.min.z == -2 && bounds.max.z == 6);
            for (const auto partitioning :
                 {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                  core::RegionPartitioningAlgorithm::Layers})
            {
                const auto candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, {}, std::nullopt, {}, partitioning, {}, core::CandidateRetention::AllWalkable);
                assert(candidate.topology.valid);
                assert(!HasNavigationAt(candidate, {0, 0, 0}) && !HasNavigationAt(candidate, {0, 0, 6}));
                for (const float x : {-230.0F, 230.0F})
                {
                    for (const float y : {-230.0F, 230.0F})
                    {
                        assert(!HasNavigationAt(candidate, {x, y, 0}));
                    }
                }
                assert(HasNavigationAt(candidate, {320, 320, 0}));
            }

            // Scripted activators and emitted hazards may reuse a persistent fire's
            // exact NIF. Record semantics still forbid a permanent fire exclusion.
            for (const auto *type : {"", "ACTI", "HAZD", "PROJ", "EXPL"})
            {
                auto transient = cell;
                transient.references[1].baseRecordType = type;
                const auto extracted =
                    skyrim::ExtractGeometry(root, transient, root, {}, {}, nullptr, &cache, navigationOnly);
                assert(extracted.modelsExcluded == 1 && extracted.modelsLoaded == 1);
                assert(extracted.scene.coverage.back().status == core::GeometryCoverage::Excluded);
                assert(extracted.scene.renderFallbackMesh.triangles.empty());
            }
            for (const auto *path : {"MESHES/Effects/FXFireTrap.nif", "FX/FireJet.nif", "Effects/FXFireOilHazard.nif",
                                     "Effects/FXFireWithEmbersToggle.nif"})
            {
                auto transient = cell;
                transient.references[1].modelPath = path;
                const auto extracted =
                    skyrim::ExtractGeometry(root, transient, root, {}, {}, nullptr, &cache, navigationOnly);
                assert(extracted.modelsExcluded == 1 && extracted.modelsLoaded == 1 && extracted.modelsMissing == 0);
            }
        }

        // A fire trap's solid mechanism retains ordinary collision, independently
        // of the transient flame effect and without receiving a fire obstacle tag.
        std::filesystem::create_directories(root / "meshes/traps");
        WriteCollisionModel(root / "meshes/traps/firefixture.nif", 32, 0);
        cell.references[1].modelPath = "Traps/FireFixture.nif";
        cell.references[1].baseRecordType = "ACTI";
        const auto trap = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(trap.modelsLoaded == 2 && trap.modelsExcluded == 0);
        assert(trap.scene.geometrySources[1].sourceType == core::GeometrySourceType::Collision);
        assert(!trap.scene.geometrySources[1].navigationObstacle);
    }

    // The hearth bed, rim and surrounding floor are separate visual components.
    // Only the recessed bed determines the rectangle, including under rotation/scale.
    void CheckHearthFireBounds(const std::filesystem::path &root)
    {
        using namespace navmesh;
        std::filesystem::create_directories(root / "meshes/architecture");
        nifly::NifFile hearth;
        assert(hearth.Load(root / "meshes/floor.nif") == 0);
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {0, 2, 3}};
        const std::vector<nifly::Vector3> floor = {{-512, -512, 0}, {512, -512, 0}, {512, 512, 0}, {-512, 512, 0}};
        const std::vector<nifly::Vector3> rim = {{-80, -160, 16}, {80, -160, 16}, {80, 160, 16}, {-80, 160, 16}};
        const std::vector<nifly::Vector3> bed = {{-48, -128, -16}, {48, -128, -16}, {48, 128, -16}, {-48, 128, -16}};
        assert(hearth.CreateShapeFromData("Floor", &floor, &faces, nullptr));
        assert(hearth.CreateShapeFromData("Rim", &rim, &faces, nullptr));
        assert(hearth.CreateShapeFromData("RecessedBed", &bed, &faces, nullptr));
        assert(hearth.Save(root / "meshes/architecture/firepitfixture.nif") == 0);
        assert(hearth.Save(root / "meshes/architecture/unmarkedfixture.nif") == 0);

        core::Cell cell;
        cell.references = {{.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
                           {.id = 2,
                            .recordType = "REFR",
                            .modelPath = "Effects/FXFireBed.nif",
                            .position = {72, 100, -2},
                            .baseRecordType = "MSTT"},
                           {.id = 3,
                            .recordType = "REFR",
                            .modelPath = "MESHES/Architecture/FirePitFixture.nif",
                            .position = {128, 128, 0},
                            .rotation = {0, 0, 1.57079632679F},
                            .scale = 2,
                            .baseRecordType = "STAT"}};
        const auto pose = core::Transform::FromSkyrimReference(cell.references[2].position, cell.references[2].rotation,
                                                               cell.references[2].scale);
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            for (const bool hearthFirst : {false, true})
            {
                auto input = cell;
                if (hearthFirst)
                {
                    std::swap(input.references[1], input.references[2]);
                }
                const auto geometry =
                    skyrim::ExtractGeometry(root, input, root, {}, {}, nullptr, &cache, navigationOnly);
                assert(geometry.modelsLoaded == 3 && geometry.scene.HasCompleteTriangleProvenance());
                const auto &report = geometry.references[hearthFirst ? 2 : 1];
                assert(report.collisionType == "fire hearth bounds" && report.vertices == 8 && report.triangles == 12);
                core::AABB bounds;
                for (std::size_t index = report.meshVertexOffset; index < report.meshVertexOffset + report.vertices;
                     ++index)
                {
                    bounds.Expand(geometry.scene.mesh.vertices[index]);
                }
                assert(std::abs(bounds.min.x + 128) < 0.01F && std::abs(bounds.max.x - 384) < 0.01F);
                assert(std::abs(bounds.min.y - 32) < 0.01F && std::abs(bounds.max.y - 224) < 0.01F);
                assert(std::abs(bounds.min.z + 32) < 0.01F && std::abs(bounds.max.z - 6) < 0.01F);
                for (const auto partitioning :
                     {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                      core::RegionPartitioningAlgorithm::Layers})
                {
                    const auto candidate = core::RecastCandidateGenerator{}.Generate(
                        geometry.scene, {}, std::nullopt, {}, partitioning, {}, core::CandidateRetention::AllWalkable);
                    assert(candidate.topology.valid);
                    assert(!HasNavigationAt(candidate, pose.ApplyPoint({0, 96, 0})));
                    assert(HasNavigationAt(candidate, pose.ApplyPoint({96, 0, 0})));
                }
            }

            // Reusing the same cached fire without a matching host restores its own
            // model bounds. Disabled/deleted or unrelated hosts cannot affect it.
            for (const auto state : {0, 1, 2, 3})
            {
                auto input = cell;
                if (state == 0)
                {
                    input.references.resize(2);
                }
                else if (state == 1)
                {
                    input.references[2].initiallyDisabled = true;
                }
                else if (state == 2)
                {
                    input.references[2].deleted = true;
                }
                else
                {
                    input.references[2].modelPath = "Architecture/UnmarkedFixture.nif";
                }
                const auto geometry =
                    skyrim::ExtractGeometry(root, input, root, {}, {}, nullptr, &cache, navigationOnly);
                assert(geometry.references[1].collisionType == "fire fuel bounds");
                core::AABB bounds;
                const auto &report = geometry.references[1];
                for (std::size_t index = report.meshVertexOffset; index < report.meshVertexOffset + report.vertices;
                     ++index)
                {
                    bounds.Expand(geometry.scene.mesh.vertices[index]);
                }
                assert(bounds.min.x == -128 && bounds.max.x == 384 && bounds.min.y == -128 && bounds.max.y == 384);
            }
        }
    }
} // namespace

void TestNativeBsaGeometry()
{
    using namespace navmesh;
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-bsa-geometry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    struct Cleanup
    {
        std::filesystem::path path;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove_all(path, error);
        }
    } cleanup{root};

    nifly::NifFile nif;
    nif.Create({nifly::V20_2_0_7, 12, 130});
    const std::vector<nifly::Vector3> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    const std::vector<nifly::Triangle> triangles = {{0, 1, 2}};
    assert(nif.CreateShapeFromData("Fixture", &vertices, &triangles, nullptr));
    std::ostringstream model(std::ios::binary);
    assert(nif.Save(model) == 0);
    WriteModelArchive(root / "Fixture.BSA", model.str());
    const auto snapshot = skyrim::ModelAssetCacheDirectory(root, nullptr, root / "cache");
    assert(skyrim::ModelArchives(root, nullptr).size() == 1);
    core::Cell cell;
    cell.references.push_back({.id = 1,
                               .baseObjectId = 2,
                               .recordType = "REFR",
                               .modelPath = "fixture.nif",
                               .position = {10, 20, 30},
                               .baseRecordType = "STAT"});
    skyrim::ModelGeometryCache cache;
    const auto geometry = skyrim::ExtractGeometry(root, cell, snapshot, {}, {}, nullptr, &cache);
    assert(geometry.modelsLoaded == 1 && geometry.scene.mesh.triangles.size() == 1);
    assert(geometry.scene.mesh.vertices[0].x == 10 && geometry.scene.mesh.vertices[0].z == 30);
    assert(geometry.scene.HasCompleteTriangleProvenance());
    // Decoded geometry remains valid if disk retention evicts its source NIF.
    skyrim::TrimModelAssetCache(snapshot, 0);
    assert(!std::filesystem::exists(snapshot / "meshes/fixture.nif"));
    const auto repeated = skyrim::ExtractGeometry(root, cell, snapshot, {}, {}, nullptr, &cache);
    assert(repeated.modelsLoaded == 1 && cache.Statistics().modelHits == 1);
    assert(!std::filesystem::exists(snapshot / "meshes/fixture.nif"));

    skyrim::ModelAssetSources assets;
    assets.archives = {root / "Fixture.BSA"};
    std::set<std::string> changed;
    assert(skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed == std::set<std::string>{"meshes/fixture.nif"});
    changed.clear();
    assets.looseModels["meshes/fixture.nif"] = root / "loose.nif";
    assert(skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed.empty());
    WriteModelArchive(assets.archives[0], model.str() + "revision");
    assert(skyrim::ModelAssetCacheDirectory(root, &assets, root / "cache") != snapshot);
    std::ofstream(assets.archives[0], std::ios::binary | std::ios::trunc) << "broken";
    changed.insert("sentinel");
    assert(!skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed == std::set<std::string>{"sentinel"});

    // Asset classification belongs to Skyrim extraction; the neutral generator consumes only the tag.
    const auto rockDirectory = root / "meshes/Landscape/Rocks";
    std::filesystem::create_directories(rockDirectory);
    std::ofstream(rockDirectory / "fixture.nif", std::ios::binary) << model.str();
    for (const auto *logicalPath : {"Landscape\\Rocks\\fixture.nif", "MESHES/LANDSCAPE/ROCKS/fixture.nif"})
    {
        cell.references[0].modelPath = logicalPath;
        const auto rock = skyrim::ExtractGeometry(root, cell, snapshot);
        assert(rock.modelsLoaded == 1 && rock.scene.geometrySources[0].navigationObstacle);
    }
    std::filesystem::create_directories(root / "meshes/Architecture");
    std::ofstream(root / "meshes/Architecture/RockStairs.nif", std::ios::binary) << model.str();
    cell.references[0].modelPath = "Architecture/RockStairs.nif";
    const auto architecture = skyrim::ExtractGeometry(root, cell, snapshot);
    assert(architecture.modelsLoaded == 1 && !architecture.scene.geometrySources[0].navigationObstacle);
    CheckFurnitureAndFire(root);
    CheckFixedClutter(root);
    CheckStripCollisionUnits(root);
    CheckCollisionShapes(root);
    CheckPlacedModelTransforms(root);
    CheckCollisionEligibility(root);
    CheckFireFootprints(root);
    CheckHearthFireBounds(root);
}
