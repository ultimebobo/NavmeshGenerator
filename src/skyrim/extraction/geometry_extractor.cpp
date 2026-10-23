#include "skyrim/extraction/geometry_extractor.h"
#include "core/io/file_path.h"
#include "skyrim/extraction/asset_cache.h"
#include "skyrim/extraction/bsa_archive.h"
#include "core/geometry/convex_hull.h"

#include <NifFile.hpp>
#include <Animation.hpp>
#include <bhk.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <format>
#include <functional>
#include <iostream>
#include <unordered_set>
#include <map>
#include <set>
#include <mutex>
#include <memory>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace
{
    /// Recognize the landscape rock asset directory, independent of placement, path case and separators.
    [[nodiscard]] bool IsLandscapeRock(std::string modelPath)
    {
        std::replace(modelPath.begin(), modelPath.end(), '\\', '/');
        std::transform(modelPath.begin(), modelPath.end(), modelPath.begin(),
                       [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        if (modelPath.starts_with("meshes/"))
        {
            modelPath.erase(0, std::string_view("meshes/").size());
        }
        return modelPath.starts_with("landscape/rocks/");
    }

    /// Normalize logical asset names before applying location-independent navigation policies.
    [[nodiscard]] std::string NormalizedModelPath(std::string path)
    {
        std::replace(path.begin(), path.end(), '\\', '/');
        std::transform(path.begin(), path.end(), path.begin(),
                       [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        if (path.starts_with("meshes/"))
        {
            path.erase(0, std::string_view("meshes/").size());
        }
        return path;
    }

    /// Behavior-driven passage assets can keep their state names outside the NIF in an HKX graph.
    [[nodiscard]] bool IsPassageAsset(const std::string &path)
    {
        const auto name = navmesh::core::PathToUtf8(navmesh::core::PathFromUtf8(NormalizedModelPath(path)).stem());
        return name.find("door") != std::string::npos || name.find("gate") != std::string::npos ||
               name.find("portcullis") != std::string::npos;
    }

    [[nodiscard]] bool IsAnimatedAsset(const std::string &path)
    {
        return ("/" + NormalizedModelPath(path)).find("/animated/") != std::string::npos;
    }

    /// Only persistent static fire placements may supply a permanent navigation exclusion.
    [[nodiscard]] bool IsFireEffect(const navmesh::core::Reference &reference)
    {
        if (reference.baseRecordType != "STAT" && reference.baseRecordType != "MSTT")
        {
            return false;
        }
        const auto normalized = NormalizedModelPath(reference.modelPath);
        const auto name = navmesh::core::PathToUtf8(navmesh::core::PathFromUtf8(normalized).filename());
        constexpr std::array transientKinds{"trap", "hazard", "toggle", "projectile", "breath", "stream", "jet"};
        if (std::any_of(transientKinds.begin(), transientKinds.end(),
                        [&](const char *kind) { return normalized.find(kind) != std::string::npos; }))
        {
            return false;
        }
        return (normalized.starts_with("effects/") || normalized.starts_with("fx/")) &&
               (name.starts_with("fxfire") || name.starts_with("fire"));
    }

    /// Hearth model names identify possible hosts; only an actual recessed bed permits fitting a fire.
    [[nodiscard]] bool IsHearthAsset(const std::string &path)
    {
        const auto normalized = NormalizedModelPath(path);
        const auto name = navmesh::core::PathToUtf8(navmesh::core::PathFromUtf8(normalized).stem());
        return normalized.find("trap") == std::string::npos &&
               (name.find("firepit") != std::string::npos || name.find("fireplace") != std::string::npos ||
                name.find("hearth") != std::string::npos);
    }

    /// Placed furniture and containers obstruct movement without supplying climbable navigation support.
    [[nodiscard]] bool IsFurniture(const navmesh::core::Reference &reference)
    {
        const auto path = NormalizedModelPath(reference.modelPath);
        const auto name = navmesh::core::PathToUtf8(navmesh::core::PathFromUtf8(path).stem());
        constexpr std::array furnitureKinds{"chair",   "table",    "shelf", "cupboard",
                                            "dresser", "wardrobe", "bench", "bed"};
        const bool clutterFurniture = path.starts_with("clutter/") &&
                                      std::any_of(furnitureKinds.begin(), furnitureKinds.end(), [&](const char *kind)
                                                  { return name.find(kind) != std::string::npos; });
        return reference.baseRecordType == "FURN" || reference.baseRecordType == "CONT" ||
               path.starts_with("furniture/") || clutterFurniture;
    }

    /// Static clutter and containers are permanent obstacles; inventory and movable bases keep their own policies.
    [[nodiscard]] bool IsFixedClutter(const navmesh::core::Reference &reference)
    {
        return (reference.baseRecordType == "STAT" || reference.baseRecordType == "CONT") &&
               NormalizedModelPath(reference.modelPath).starts_with("clutter/");
    }

    [[nodiscard]] std::string EscapeJson(std::string value)
    {
        std::string escaped;
        for (const auto character : value)
        {
            if (character == '\\')
            {
                escaped += "\\\\";
            }
            else if (character == '"')
            {
                escaped += "\\\"";
            }
            else if (static_cast<unsigned char>(character) < 0x20 || static_cast<unsigned char>(character) >= 0x80)
            {
                escaped += std::format("\\u00{:02X}", static_cast<unsigned char>(character));
            }
            else
            {
                escaped += character;
            }
        }
        return escaped;
    }
    [[nodiscard]] std::string JsonStringArray(const std::vector<std::string> &values)
    {
        std::string result = "[";
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            if (index != 0)
            {
                result += ",";
            }
            result += "\"" + EscapeJson(values[index]) + "\"";
        }
        result += "]";
        return result;
    }
    [[nodiscard]] const char *CoverageName(const navmesh::core::GeometryCoverage coverage)
    {
        switch (coverage)
        {
        case navmesh::core::GeometryCoverage::Found:
            return "found";
        case navmesh::core::GeometryCoverage::Excluded:
            return "excluded";
        case navmesh::core::GeometryCoverage::Missing:
            return "missing";
        case navmesh::core::GeometryCoverage::Unreadable:
            return "unreadable";
        case navmesh::core::GeometryCoverage::Unsupported:
            return "unsupported";
        }
        return "unknown";
    }
    [[nodiscard]] const char *SourceTypeName(const navmesh::core::GeometrySourceType sourceType)
    {
        switch (sourceType)
        {
        case navmesh::core::GeometrySourceType::Terrain:
            return "terrain";
        case navmesh::core::GeometrySourceType::Collision:
            return "collision";
        case navmesh::core::GeometrySourceType::RenderFallback:
            return "render_fallback";
        case navmesh::core::GeometrySourceType::NavigationExclusion:
            return "navigation_exclusion";
        }
        return "unknown";
    }
    [[nodiscard]] bool IsFiniteVec3(const navmesh::core::Vec3 &value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
               std::abs(value.x) < 1.0e7F && std::abs(value.y) < 1.0e7F && std::abs(value.z) < 1.0e7F;
    }
    struct TriangleGeometry
    {
        std::vector<navmesh::core::Vec3> vertices;
        std::vector<navmesh::core::Triangle> triangles;
        std::size_t invalidIndices{};
        std::size_t degenerateTriangles{};
        std::vector<std::string> shapes;
    };
    struct NifGeometry
    {
        TriangleGeometry render;
        TriangleGeometry collision;
        TriangleGeometry fireExclusion;
        navmesh::core::AABB hearthBedBounds;
        std::vector<std::string> collisionShapeTypes;
        bool nonSolidCollision{};
        /// Physical collision presence and decode completeness distinguish unavailable colliders from visual-only NIFs.
        bool solidCollision{}, incompleteCollision{};
        bool operablePartsExcluded{};
        bool operablePassage{};
        std::string version;
        std::string readFailure;
    };

    using OperableNodes = std::unordered_set<const nifly::NiAVObject *>;

    [[nodiscard]] std::string AnimationStateName(std::string name)
    {
        const auto first = name.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            return {};
        }
        name = name.substr(first, name.find_last_not_of(" \t\r\n") - first + 1);
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        return name;
    }

    /// A direct controller target is authoritative; name-only links require a unique matching scene object.
    [[nodiscard]] const nifly::NiAVObject *SequenceTarget(const nifly::NiHeader &header,
                                                          const nifly::ControllerLink &link)
    {
        if (const auto *controller = header.GetBlock<nifly::NiTransformController>(link.controllerRef))
        {
            if (const auto *target = header.GetBlock<nifly::NiAVObject>(controller->targetRef))
            {
                return target;
            }
        }
        const nifly::NiAVObject *target{};
        for (std::uint32_t block{}; block < header.GetNumBlocks(); ++block)
        {
            const auto *object = header.GetBlock<nifly::NiAVObject>(block);
            if (object && !link.nodeName.get().empty() && object->name.get() == link.nodeName.get())
            {
                if (target)
                {
                    return nullptr;
                }
                target = object;
            }
        }
        return target;
    }

    /// Missing channels and constant keys do not make a fixed frame an operable panel.
    [[nodiscard]] bool HasTransformMotion(const nifly::NiTransformData &data)
    {
        const auto changes = [](const auto &keys)
        {
            for (std::uint32_t index = 1; index < keys.GetNumKeys(); ++index)
            {
                if (!(keys.GetKey(index).value == keys.GetKey(0).value))
                {
                    return true;
                }
            }
            return false;
        };
        for (std::size_t index = 1; index < data.quaternionKeys.size(); ++index)
        {
            const auto &first = data.quaternionKeys.front().value;
            const auto &value = data.quaternionKeys[index].value;
            // Opposite quaternion signs encode the same orientation.
            const bool same = first.w == value.w && first.x == value.x && first.y == value.y && first.z == value.z;
            const bool opposite =
                first.w == -value.w && first.x == -value.x && first.y == -value.y && first.z == -value.z;
            if (!same && !opposite)
            {
                return true;
            }
        }
        return changes(data.translations) || changes(data.scales) || changes(data.xRotations) ||
               changes(data.yRotations) || changes(data.zRotations);
    }

    /** Identify movable passage branches without evaluating external runtime behavior graphs.
     * Paired Open/Close sequences identify operable activators independently of asset names. Behavior-driven
     * door/gate assets instead expose direct transform controllers. Only animated targets are excluded;
     * fixed frame and threshold siblings remain physical geometry in their authored model coordinates.
     */
    [[nodiscard]] OperableNodes FindOperableNodes(const nifly::NifFile &nif, bool passageAsset)
    {
        const auto &header = nif.GetHeader();
        OperableNodes result;
        bool behaviorDriven{};
        const auto hasMotion = [&](const auto &interpolatorRef)
        {
            const auto *interpolator = header.GetBlock<nifly::NiTransformInterpolator>(interpolatorRef);
            const auto *data = interpolator ? header.GetBlock(interpolator->dataRef) : nullptr;
            return data && HasTransformMotion(*data);
        };
        for (std::uint32_t block{}; block < header.GetNumBlocks(); ++block)
        {
            behaviorDriven = behaviorDriven || header.GetBlock<nifly::BSBehaviorGraphExtraData>(block) != nullptr;
            const auto *manager = header.GetBlock<nifly::NiControllerManager>(block);
            if (!manager)
            {
                continue;
            }
            bool hasOpen{}, hasClose{};
            for (auto sequenceRef = manager->controllerSequenceRefs.cbegin();
                 sequenceRef != manager->controllerSequenceRefs.cend(); ++sequenceRef)
            {
                if (const auto *sequence = header.GetBlock(*sequenceRef))
                {
                    const auto name = AnimationStateName(sequence->name.get());
                    hasOpen = hasOpen || name == "open";
                    hasClose = hasClose || name == "close";
                }
            }
            if (!hasOpen || !hasClose)
            {
                continue;
            }
            for (auto sequenceRef = manager->controllerSequenceRefs.cbegin();
                 sequenceRef != manager->controllerSequenceRefs.cend(); ++sequenceRef)
            {
                const auto *sequence = header.GetBlock(*sequenceRef);
                if (!sequence || (AnimationStateName(sequence->name.get()) != "open" &&
                                  AnimationStateName(sequence->name.get()) != "close"))
                {
                    continue;
                }
                for (auto linkAt = sequence->controlledBlocks.cbegin(); linkAt != sequence->controlledBlocks.cend();
                     ++linkAt)
                {
                    const auto &link = *linkAt;
                    if (!hasMotion(link.interpolatorRef))
                    {
                        continue;
                    }
                    if (const auto *target = SequenceTarget(header, link))
                    {
                        result.insert(target);
                    }
                }
            }
        }
        if (behaviorDriven && passageAsset)
        {
            for (std::uint32_t block{}; block < header.GetNumBlocks(); ++block)
            {
                const auto *controller = header.GetBlock<nifly::NiTransformController>(block);
                if (controller && hasMotion(controller->interpolatorRef))
                {
                    if (const auto *target = header.GetBlock<nifly::NiAVObject>(controller->targetRef))
                    {
                        result.insert(target);
                    }
                }
            }
        }
        return result;
    }

    /// A panel may own collision/render descendants rather than attaching geometry to its controller target.
    [[nodiscard]] bool IsOperablePart(const nifly::NifFile &nif, nifly::NiAVObject *object,
                                      const OperableNodes &operableNodes)
    {
        if (operableNodes.empty())
        {
            return false;
        }
        std::unordered_set<const nifly::NiAVObject *> ancestors;
        while (object && ancestors.insert(object).second)
        {
            if (operableNodes.contains(object))
            {
                return true;
            }
            object = nif.GetParentNode(object);
        }
        return false;
    }

    /** Enclose every possible model placement, including continuous translation and rotation ranges.
     * Support vertices already include NIF/Havok transforms or exclusion bounds in model-local Skyrim units.
     * Including the model origin covers all intermediate scales without sampling random outcomes.
     */
    TriangleGeometry CollisionEnvelope(const TriangleGeometry &collision, const navmesh::core::Reference &reference)
    {
        using namespace navmesh::core;
        const auto &envelope = *reference.envelope;
        AABB local{{0, 0, 0}, {0, 0, 0}};
        const auto rotation = Transform::FromSkyrimReference({}, reference.rotation, envelope.maximumScale);
        float radius{};
        for (const auto &vertex : collision.vertices)
        {
            if (envelope.arbitraryRotation)
            {
                radius = std::max(radius, std::hypot(vertex.x, vertex.y, vertex.z) * envelope.maximumScale);
            }
            else
            {
                local.Expand(rotation.ApplyPoint(vertex));
            }
        }
        if (envelope.arbitraryRotation)
        {
            local = {{-radius, -radius, -radius}, {radius, radius, radius}};
        }
        const auto low = local.min + envelope.minimumPosition;
        const auto high = local.max + envelope.maximumPosition;
        if (!std::isfinite(low.x) || !std::isfinite(low.y) || !std::isfinite(low.z) || !std::isfinite(high.x) ||
            !std::isfinite(high.y) || !std::isfinite(high.z))
        {
            throw std::runtime_error("BOS collision envelope exceeds finite Skyrim world coordinates");
        }
        TriangleGeometry result;
        result.vertices = {{low.x, low.y, low.z},    {high.x, low.y, low.z}, {high.x, high.y, low.z},
                           {low.x, high.y, low.z},   {low.x, low.y, high.z}, {high.x, low.y, high.z},
                           {high.x, high.y, high.z}, {low.x, high.y, high.z}};
        result.triangles = {{{0, 2, 1}}, {{0, 3, 2}}, {{4, 5, 6}}, {{4, 6, 7}}, {{0, 1, 5}}, {{0, 5, 4}},
                            {{1, 2, 6}}, {{1, 6, 5}}, {{2, 3, 7}}, {{2, 7, 6}}, {{3, 0, 4}}, {{3, 4, 7}}};
        result.shapes = {"BOS conservative collision envelope"};
        return result;
    }

    constexpr float SkyrimUnitsPerHavokUnit = 69.99125F;

    /// Shape wrappers use Havok units; the collision target and its ancestors use NIF/Skyrim units.
    struct CollisionTransform
    {
        nifly::Matrix4 shape;
        const nifly::bhkRigidBody *body{};
        nifly::Matrix4 target;
    };

    /** Resolve geometry into placed-model coordinates by block identity, including the object itself.
     * Skyrim replaces the model root's local transform with the REFR transform. Only descendants
     * contribute NIF transforms; unnamed and duplicate-named ancestors are valid. Cycles supply no geometry.
     */
    [[nodiscard]] std::optional<nifly::Matrix4> ModelSpaceTransform(const nifly::NifFile &nif,
                                                                    nifly::NiAVObject *object)
    {
        nifly::Matrix4 transform;
        std::unordered_set<const nifly::NiAVObject *> ancestors;
        while (object && object != nif.GetRootNode())
        {
            if (!ancestors.insert(object).second)
            {
                return std::nullopt;
            }
            transform = object->transform.ToMatrix() * transform;
            object = nif.GetParentNode(object);
        }
        return transform;
    }

    /// Rotate an xyzw quaternion and translate without changing the input coordinate units.
    [[nodiscard]] nifly::Vector3 ApplyHavokTransform(const nifly::QuaternionXYZW &q, const nifly::Vector4 &translation,
                                                     const nifly::Vector3 &vertex)
    {
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        return {((1 - 2 * (yy + zz)) * vertex.x + 2 * (xy - wz) * vertex.y + 2 * (xz + wy) * vertex.z + translation.x),
                (2 * (xy + wz) * vertex.x + (1 - 2 * (xx + zz)) * vertex.y + 2 * (yz - wx) * vertex.z + translation.y),
                (2 * (xz - wy) * vertex.x + 2 * (yz + wx) * vertex.y + (1 - 2 * (xx + yy)) * vertex.z + translation.z)};
    }

    /// Resolve shape -> body -> collision target in model coordinates; placed-reference transforms follow later.
    [[nodiscard]] navmesh::core::Vec3 ApplyCollisionTransform(const CollisionTransform &transform,
                                                              const nifly::Vector3 &vertex)
    {
        auto position = transform.shape * vertex;
        // Only the T body activates its stored translation and rotation.
        if (dynamic_cast<const nifly::bhkRigidBodyT *>(transform.body))
        {
            position = ApplyHavokTransform(transform.body->rotation, transform.body->translation, position);
        }
        position = transform.target * (position * SkyrimUnitsPerHavokUnit);
        return {position.x, position.y, position.z};
    }
    void AddPackedTriangles(const nifly::hkPackedNiTriStripsData &data, const CollisionTransform &transform,
                            TriangleGeometry &result)
    {
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(result.vertices.size() + data.compressedVertData.size());
        for (const auto &vertex : data.compressedVertData)
        {
            result.vertices.push_back(ApplyCollisionTransform(transform, vertex));
        }
        const auto &triangles = data.triData;
        for (const auto &triangle : triangles)
        {
            if (triangle.tri.p1 >= data.compressedVertData.size() ||
                triangle.tri.p2 >= data.compressedVertData.size() || triangle.tri.p3 >= data.compressedVertData.size())
            {
                ++result.invalidIndices;
                continue;
            }
            if (triangle.tri.p1 == triangle.tri.p2 || triangle.tri.p1 == triangle.tri.p3 ||
                triangle.tri.p2 == triangle.tri.p3)
            {
                ++result.degenerateTriangles;
                continue;
            }
            result.triangles.push_back({{base + triangle.tri.p1, base + triangle.tri.p2, base + triangle.tri.p3}});
        }
    }

    /// Unpacked strip vertices use model units; normalize to Havok units before wrapper/body transforms.
    void AddNiTriStripsTriangles(const nifly::NiTriStripsData &data, const CollisionTransform &transform,
                                 TriangleGeometry &result)
    {
        std::vector<nifly::Triangle> triangles;
        if (!data.GetTriangles(triangles))
        {
            return;
        }
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(result.vertices.size() + data.vertices.size());
        for (const auto &vertex : data.vertices)
        {
            result.vertices.push_back(ApplyCollisionTransform(transform, vertex / SkyrimUnitsPerHavokUnit));
        }
        for (const auto &triangle : triangles)
        {
            if (triangle.p1 >= data.vertices.size() || triangle.p2 >= data.vertices.size() ||
                triangle.p3 >= data.vertices.size())
            {
                ++result.invalidIndices;
                continue;
            }
            if (triangle.p1 == triangle.p2 || triangle.p1 == triangle.p3 || triangle.p2 == triangle.p3)
            {
                ++result.degenerateTriangles;
                continue;
            }
            result.triangles.push_back({{base + triangle.p1, base + triangle.p2, base + triangle.p3}});
        }
    }

    void AddCompressedBigTriangles(const nifly::bhkCompressedMeshShapeData &data, const CollisionTransform &transform,
                                   TriangleGeometry &result)
    {
        // "Big" vertices are already stored as floating-point coordinates.
        // They cover the non-quantised triangle stream in compressed Havok
        // meshes, so they can be exported exactly without guessing at the
        // packed-chunk bit layout.
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(result.vertices.size() + data.bigVerts.size());
        for (auto vertex = data.bigVerts.cbegin(); vertex != data.bigVerts.cend(); ++vertex)
        {
            result.vertices.push_back(ApplyCollisionTransform(transform, {vertex->x, vertex->y, vertex->z}));
        }
        for (auto triangle = data.bigTris.cbegin(); triangle != data.bigTris.cend(); ++triangle)
        {
            if (triangle->triangle1 >= data.bigVerts.size() || triangle->triangle2 >= data.bigVerts.size() ||
                triangle->triangle3 >= data.bigVerts.size())
            {
                ++result.invalidIndices;
                continue;
            }
            if (triangle->triangle1 == triangle->triangle2 || triangle->triangle1 == triangle->triangle3 ||
                triangle->triangle2 == triangle->triangle3)
            {
                ++result.degenerateTriangles;
                continue;
            }
            result.triangles.push_back(
                {{base + triangle->triangle1, base + triangle->triangle2, base + triangle->triangle3}});
        }
    }

    void AddCompressedChunkTriangles(const nifly::bhkCompressedMeshShapeData &data, const CollisionTransform &transform,
                                     TriangleGeometry &result)
    {
        for (auto chunk = data.chunks.cbegin(); chunk != data.chunks.cend(); ++chunk)
        {
            // Each chunk indexes its own Havok-space rotation/translation after vertex dequantization.
            if (chunk->transformIndex >= data.transforms.size())
            {
                ++result.invalidIndices;
                continue;
            }
            const auto &chunkTransform = *(data.transforms.cbegin() + chunk->transformIndex);
            // Chunk::verts is a flat [x, y, z] u16 component array.  Each
            // component is quantized using the mesh-wide error term around
            // this chunk's local translation.  Indices address the resulting
            // vertices directly (not the flat component array).
            const auto vertexCount = static_cast<std::uint32_t>(chunk->verts.size() / 3);
            const auto base = static_cast<std::uint32_t>(result.vertices.size());
            auto component = chunk->verts.cbegin();
            for (std::uint32_t vertex{}; vertex < vertexCount; ++vertex)
            {
                const nifly::Vector3 position{chunk->translation.x + static_cast<float>(*component++) * data.error,
                                              chunk->translation.y + static_cast<float>(*component++) * data.error,
                                              chunk->translation.z + static_cast<float>(*component++) * data.error};
                result.vertices.push_back(ApplyCollisionTransform(
                    transform, ApplyHavokTransform(chunkTransform.rotation, chunkTransform.translation, position)));
            }

            std::vector<std::uint16_t> indices;
            indices.reserve(chunk->indices.size());
            for (auto index = chunk->indices.cbegin(); index != chunk->indices.cend(); ++index)
            {
                indices.push_back(*index);
            }
            const auto addTriangle =
                [&](const std::uint16_t first, const std::uint16_t second, const std::uint16_t third)
            {
                if (first >= vertexCount || second >= vertexCount || third >= vertexCount)
                {
                    ++result.invalidIndices;
                    return;
                }
                if (first == second || first == third || second == third)
                {
                    ++result.degenerateTriangles;
                    return;
                }
                result.triangles.push_back({{base + first, base + second, base + third}});
            };

            std::size_t offset{};
            for (auto stripLength = chunk->strips.cbegin();
                 stripLength != chunk->strips.cend() && offset < indices.size(); ++stripLength)
            {
                const auto end = std::min(offset + static_cast<std::size_t>(*stripLength), indices.size());
                for (std::size_t index = offset; index + 2 < end; ++index)
                {
                    if ((index - offset) % 2 == 0)
                    {
                        addTriangle(indices[index], indices[index + 1], indices[index + 2]);
                    }
                    else
                    {
                        addTriangle(indices[index + 1], indices[index], indices[index + 2]);
                    }
                }
                offset = end;
            }
            for (; offset + 2 < indices.size(); offset += 3)
            {
                addTriangle(indices[offset], indices[offset + 1], indices[offset + 2]);
            }
        }
    }

    /// Decode the closed vertex hull in Havok units before applying rigid-body and placed-reference transforms.
    /// Degenerate or nonfinite hulls supply no partial collision; the render mesh remains diagnostic fallback.
    void AddHullTriangles(const std::vector<navmesh::core::Vec3> &vertices, const CollisionTransform &transform,
                          const std::string &shapeName, TriangleGeometry &result)
    {
        const auto triangles = navmesh::core::TriangulateConvexHull(vertices);
        if (!triangles)
        {
            return;
        }
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        for (const auto &vertex : vertices)
        {
            result.vertices.push_back(ApplyCollisionTransform(transform, {vertex.x, vertex.y, vertex.z}));
        }
        for (auto triangle : *triangles)
        {
            for (auto &index : triangle.vertices)
            {
                index += base;
            }
            result.triangles.push_back(triangle);
        }
        result.shapes.push_back(shapeName);
    }

    void AddConvexTriangles(const nifly::bhkConvexVerticesShape &shape, const CollisionTransform &transform,
                            TriangleGeometry &result)
    {
        std::vector<navmesh::core::Vec3> vertices;
        for (auto vertex = shape.verts.cbegin(); vertex != shape.verts.cend(); ++vertex)
        {
            vertices.push_back({vertex->x, vertex->y, vertex->z});
        }
        AddHullTriangles(vertices, transform, "bhkConvexVerticesShape", result);
    }

    /// Box dimensions are half extents in Havok units; skin radii do not expand the explicit faces.
    void AddBoxTriangles(const nifly::bhkBoxShape &shape, const CollisionTransform &transform, TriangleGeometry &result)
    {
        const auto &extent = shape.dimensions;
        if (!std::isfinite(extent.x) || !std::isfinite(extent.y) || !std::isfinite(extent.z) || extent.x <= 0 ||
            extent.y <= 0 || extent.z <= 0)
        {
            return;
        }
        std::vector<navmesh::core::Vec3> vertices;
        for (const float x : {-extent.x, extent.x})
        {
            for (const float y : {-extent.y, extent.y})
            {
                for (const float z : {-extent.z, extent.z})
                {
                    vertices.push_back({x, y, z});
                }
            }
        }
        AddHullTriangles(vertices, transform, "bhkBoxShape", result);
    }

    /// Tessellate a rounded segment in Havok units along its arbitrary local axis.
    /// Coincident endpoints describe a sphere; a closed hull joins equators and supplies outward winding.
    void AddRoundedSegmentTriangles(const nifly::Vector3 &first, const nifly::Vector3 &second, float radius,
                                    const std::string &shapeName, const CollisionTransform &transform,
                                    TriangleGeometry &result)
    {
        if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(first.x) || !std::isfinite(first.y) ||
            !std::isfinite(first.z) || !std::isfinite(second.x) || !std::isfinite(second.y) || !std::isfinite(second.z))
        {
            return;
        }
        auto axis = second - first;
        const float length = axis.length();
        if (!std::isfinite(length))
        {
            return;
        }
        axis = length > 0 ? axis / length : nifly::Vector3{0, 0, 1};
        auto radial = std::abs(axis.z) < 0.9F ? nifly::Vector3{axis.y, -axis.x, 0} : nifly::Vector3{-axis.z, 0, axis.x};
        radial.Normalize();
        const nifly::Vector3 tangent{axis.y * radial.z - axis.z * radial.y, axis.z * radial.x - axis.x * radial.z,
                                     axis.x * radial.y - axis.y * radial.x};
        constexpr unsigned sectors = 16;
        constexpr unsigned hemisphereSteps = 4;
        constexpr float pi = 3.14159265358979323846F;
        std::vector<navmesh::core::Vec3> vertices;
        for (unsigned end = 0; end < 2; ++end)
        {
            const auto center = end == 0 ? first : second;
            const float direction = end == 0 ? -1.0F : 1.0F;
            const auto pole = center + axis * (direction * radius);
            vertices.push_back({pole.x, pole.y, pole.z});
            for (unsigned step = 0; step < hemisphereSteps; ++step)
            {
                const float latitude = static_cast<float>(step) * pi / (2 * hemisphereSteps);
                for (unsigned sector = 0; sector < sectors; ++sector)
                {
                    const float angle = static_cast<float>(sector) * 2 * pi / sectors;
                    const auto position =
                        center + axis * (direction * radius * std::sin(latitude)) +
                        (radial * std::cos(angle) + tangent * std::sin(angle)) * (radius * std::cos(latitude));
                    vertices.push_back({position.x, position.y, position.z});
                }
            }
        }
        AddHullTriangles(vertices, transform, shapeName, result);
    }

    /// Skyrim's nonphysical layers describe queries or event volumes, even when their bodies have contact response.
    /// Values follow the SkyrimLayer enum in the NIF format; older Bethesda assets use different layer tables.
    enum class NonPhysicalSkyrimLayer : std::uint8_t
    {
        Water = 11,
        Trigger = 12,
        NonCollidable = 15,
        AcousticSpace = 21,
        ActorZone = 22,
        ProjectileZone = 23,
        CameraSphere = 36,
        DoorDetection = 37,
        ConeProjectile = 38,
        CameraPick = 39,
        ItemPick = 40,
        LineOfSight = 41,
        PathPick = 42,
        CustomPick1 = 43,
        CustomPick2 = 44,
        SpellExplosion = 45,
        DroppingPick = 46,
        DeadActorZone = 47,
        FallingTrapTrigger = 48,
        SpellTrigger = 51,
        Detection = 53,
        TrapTrigger = 54
    };

    /// Only physical contact bodies outside known nonblocking Skyrim layers can supply solid navigation geometry.
    [[nodiscard]] bool HasSolidNavigationCollision(const nifly::bhkRigidBody &body, const nifly::NiVersion &version)
    {
        if (body.collisionResponse != nifly::RESPONSE_SIMPLE_CONTACT)
        {
            return false;
        }
        if (!version.IsBethesda() || version.Stream() < 83)
        {
            return true;
        }
        switch (static_cast<NonPhysicalSkyrimLayer>(body.collisionFilter.layer))
        {
        case NonPhysicalSkyrimLayer::Water:
        case NonPhysicalSkyrimLayer::Trigger:
        case NonPhysicalSkyrimLayer::NonCollidable:
        case NonPhysicalSkyrimLayer::AcousticSpace:
        case NonPhysicalSkyrimLayer::ActorZone:
        case NonPhysicalSkyrimLayer::ProjectileZone:
        case NonPhysicalSkyrimLayer::CameraSphere:
        case NonPhysicalSkyrimLayer::DoorDetection:
        case NonPhysicalSkyrimLayer::ConeProjectile:
        case NonPhysicalSkyrimLayer::CameraPick:
        case NonPhysicalSkyrimLayer::ItemPick:
        case NonPhysicalSkyrimLayer::LineOfSight:
        case NonPhysicalSkyrimLayer::PathPick:
        case NonPhysicalSkyrimLayer::CustomPick1:
        case NonPhysicalSkyrimLayer::CustomPick2:
        case NonPhysicalSkyrimLayer::SpellExplosion:
        case NonPhysicalSkyrimLayer::DroppingPick:
        case NonPhysicalSkyrimLayer::DeadActorZone:
        case NonPhysicalSkyrimLayer::FallingTrapTrigger:
        case NonPhysicalSkyrimLayer::SpellTrigger:
        case NonPhysicalSkyrimLayer::Detection:
        case NonPhysicalSkyrimLayer::TrapTrigger:
            return false;
        default:
            return true;
        }
    }

    /// Decode each reachable shape placement. The active path prevents cycles without discarding shared instances.
    void LoadPackedCollision(const nifly::NifFile &nif, TriangleGeometry &result, std::vector<std::string> &shapeTypes,
                             bool &nonSolidCollision, const OperableNodes &operableNodes, bool &operablePartsExcluded,
                             bool &solidCollision, bool &incompleteCollision)
    {
        const auto &header = nif.GetHeader();
        std::function<void(nifly::bhkShape *, CollisionTransform, std::vector<std::uint32_t>)> visit =
            [&](nifly::bhkShape *shape, CollisionTransform transform, std::vector<std::uint32_t> path)
        {
            if (!shape)
            {
                incompleteCollision = true;
                return;
            }
            const auto id = header.GetBlockID(shape);
            if (std::find(path.begin(), path.end(), id) != path.end())
            {
                incompleteCollision = true;
                return;
            }
            path.push_back(id);
            const std::string shapeName = shape->GetBlockName();
            const auto trianglesBefore = result.triangles.size();
            if (std::find(shapeTypes.begin(), shapeTypes.end(), shapeName) == shapeTypes.end())
            {
                shapeTypes.push_back(shapeName);
            }
            if (auto *wrapper = dynamic_cast<nifly::bhkTransformShape *>(shape))
            {
                // NIF stores this Havok matrix column-major; Nifly's arithmetic uses row-major matrices.
                nifly::Matrix4 local;
                for (int row = 0; row < 4; ++row)
                {
                    for (int column = 0; column < 4; ++column)
                    {
                        local[row * 4 + column] = wrapper->xform[column * 4 + row];
                    }
                }
                transform.shape = transform.shape * local;
                visit(header.GetBlock(wrapper->shapeRef), transform, path);
            }
            else if (auto *mopp = dynamic_cast<nifly::bhkMoppBvTreeShape *>(shape))
            {
                visit(header.GetBlock(mopp->shapeRef), transform, path);
            }
            else if (auto *list = dynamic_cast<nifly::bhkListShape *>(shape))
            {
                for (const auto &child : list->subShapeRefs)
                {
                    visit(header.GetBlock(child), transform, path);
                }
            }
            else if (auto *convexList = dynamic_cast<nifly::bhkConvexListShape *>(shape))
            {
                for (const auto &child : convexList->shapeRefs)
                {
                    visit(header.GetBlock(child), transform, path);
                }
            }
            else if (auto *capsule = dynamic_cast<nifly::bhkCapsuleShape *>(shape))
            {
                AddRoundedSegmentTriangles(capsule->point1, capsule->point2, capsule->radius, shapeName, transform,
                                           result);
            }
            else if (auto *sphere = dynamic_cast<nifly::bhkSphereShape *>(shape))
            {
                AddRoundedSegmentTriangles({}, {}, sphere->radius, shapeName, transform, result);
            }
            else if (auto *box = dynamic_cast<nifly::bhkBoxShape *>(shape))
            {
                AddBoxTriangles(*box, transform, result);
            }
            else if (auto *convex = dynamic_cast<nifly::bhkConvexVerticesShape *>(shape))
            {
                AddConvexTriangles(*convex, transform, result);
            }
            else if (auto *packed = dynamic_cast<nifly::bhkPackedNiTriStripsShape *>(shape))
            {
                if (const auto *data = header.GetBlock(packed->dataRef))
                {
                    AddPackedTriangles(*data, transform, result);
                    result.shapes.push_back("hkPackedNiTriStripsData");
                }
            }
            else if (auto *triStrips = dynamic_cast<nifly::bhkNiTriStripsShape *>(shape))
            {
                for (const auto &part : triStrips->partRefs)
                {
                    if (const auto *data = header.GetBlock(part))
                    {
                        AddNiTriStripsTriangles(*data, transform, result);
                        result.shapes.push_back("NiTriStripsData");
                    }
                }
            }
            else if (auto *compressed = dynamic_cast<nifly::bhkCompressedMeshShape *>(shape))
            {
                if (const auto *data = header.GetBlock(compressed->dataRef))
                {
                    const auto compressedTrianglesBefore = result.triangles.size();
                    AddCompressedBigTriangles(*data, transform, result);
                    AddCompressedChunkTriangles(*data, transform, result);
                    if (result.triangles.size() > compressedTrianglesBefore)
                    {
                        result.shapes.push_back("bhkCompressedMeshShape");
                    }
                }
            }
            else
            {
                incompleteCollision = true;
            }
            const bool container =
                dynamic_cast<nifly::bhkTransformShape *>(shape) || dynamic_cast<nifly::bhkMoppBvTreeShape *>(shape) ||
                dynamic_cast<nifly::bhkListShape *>(shape) || dynamic_cast<nifly::bhkConvexListShape *>(shape);
            if (!container && result.triangles.size() == trianglesBefore)
            {
                incompleteCollision = true;
            }
        };
        for (std::uint32_t block = 0; block < header.GetNumBlocks(); ++block)
        {
            const auto *collision = header.GetBlock<nifly::bhkNiCollisionObject>(block);
            if (!collision)
            {
                continue;
            }
            if (IsOperablePart(nif, header.GetBlock(collision->targetRef), operableNodes))
            {
                operablePartsExcluded = true;
                continue;
            }
            const auto *collisionBody = header.GetBlock<nifly::NiObject>(collision->bodyRef);
            // Phantoms are overlap/query volumes, independent of their layer and enclosed shape type.
            // Exclude only that body so physical siblings still supply their complete collision.
            if (dynamic_cast<const nifly::bhkPhantom *>(collisionBody))
            {
                nonSolidCollision = true;
                continue;
            }
            const auto *body = dynamic_cast<const nifly::bhkRigidBody *>(collisionBody);
            if (!body)
            {
                incompleteCollision = true;
                solidCollision = true;
                continue;
            }
            if (!HasSolidNavigationCollision(*body, header.GetVersion()))
            {
                nonSolidCollision = true;
                continue;
            }
            solidCollision = true;
            const auto target = ModelSpaceTransform(nif, header.GetBlock(collision->targetRef));
            if (!target)
            {
                incompleteCollision = true;
                continue;
            }
            CollisionTransform transform{.body = body, .target = *target};
            visit(header.GetBlock(body->shapeRef), transform, {});
        }
    }

    /// Valid triangle samples in model coordinates; unused or malformed vertices cannot enlarge the footprint.
    struct FireFootprint
    {
        std::vector<navmesh::core::Vec3> points;
        std::size_t invalidIndices{};
        std::size_t degenerateTriangles{};
    };

    /// Collect finite, nondegenerate fuel/flame triangles after applying shape and ancestor transforms.
    void CollectFireFootprint(const nifly::NifFile &nif, nifly::NiShape *shape, FireFootprint &footprint)
    {
        std::vector<nifly::Vector3> vertices;
        std::vector<nifly::Triangle> triangles;
        if (!nif.GetVertsForShape(shape, vertices) || !shape->GetTriangles(triangles))
        {
            return;
        }
        const auto transform = ModelSpaceTransform(nif, shape);
        if (!transform)
        {
            return;
        }
        for (auto &vertex : vertices)
        {
            vertex = *transform * vertex;
        }
        for (const auto &triangle : triangles)
        {
            if (triangle.p1 >= vertices.size() || triangle.p2 >= vertices.size() || triangle.p3 >= vertices.size())
            {
                ++footprint.invalidIndices;
                continue;
            }
            const auto convert = [](const nifly::Vector3 &point)
            { return navmesh::core::Vec3{point.x, point.y, point.z}; };
            const auto a = convert(vertices[triangle.p1]);
            const auto b = convert(vertices[triangle.p2]);
            const auto c = convert(vertices[triangle.p3]);
            if (!IsFiniteVec3(a) || !IsFiniteVec3(b) || !IsFiniteVec3(c))
            {
                continue;
            }
            const auto ab = b - a;
            const auto ac = c - a;
            const double normalX = static_cast<double>(ab.y) * ac.z - static_cast<double>(ab.z) * ac.y;
            const double normalY = static_cast<double>(ab.z) * ac.x - static_cast<double>(ab.x) * ac.z;
            const double normalZ = static_cast<double>(ab.x) * ac.y - static_cast<double>(ab.y) * ac.x;
            if (normalX == 0 && normalY == 0 && normalZ == 0)
            {
                ++footprint.degenerateTriangles;
                continue;
            }
            footprint.points.insert(footprint.points.end(), {a, b, c});
        }
    }

    /// Build a closed rectangular exclusion in model coordinates; planar bounds remain a flat exclusion.
    [[nodiscard]] TriangleGeometry BuildBoundsExclusion(const navmesh::core::AABB &bounds, const std::string &shape)
    {
        TriangleGeometry result;
        if (!bounds.IsValid() || bounds.min.x >= bounds.max.x || bounds.min.y >= bounds.max.y)
        {
            return result;
        }
        const auto &low = bounds.min;
        const auto &high = bounds.max;
        result.vertices = {{low.x, low.y, low.z},    {high.x, low.y, low.z}, {high.x, high.y, low.z},
                           {low.x, high.y, low.z},   {low.x, low.y, high.z}, {high.x, low.y, high.z},
                           {high.x, high.y, high.z}, {low.x, high.y, high.z}};
        result.triangles = {{{0, 2, 1}}, {{0, 3, 2}}, {{4, 5, 6}}, {{4, 6, 7}}, {{0, 1, 5}}, {{0, 5, 4}},
                            {{1, 2, 6}}, {{1, 6, 5}}, {{2, 3, 7}}, {{2, 7, 6}}, {{3, 0, 4}}, {{3, 4, 7}}};
        result.shapes.push_back(shape);
        return result;
    }

    /// Rasterize a teleport threshold as a volume separator; ordinary hinged doors provide no boundary.
    bool AppendTeleportBoundary(navmesh::skyrim::GeometryExtraction &output, const navmesh::core::Reference &reference,
                                navmesh::core::GeometrySource source, navmesh::skyrim::GeometryReferenceReport &report)
    {
        using namespace navmesh;
        // Automatic doors activate while traversing their bounds. Treating that trigger as solid
        // would remove its arrival floor; surrounding static collision supplies physical barriers.
        if (!reference.teleportExit || reference.automaticDoor || !reference.teleportArrival ||
            !reference.localBounds || reference.baseRecordType != "DOOR" || !std::isfinite(reference.scale) ||
            reference.scale <= 0)
        {
            return false;
        }
        const auto boundary = BuildBoundsExclusion(*reference.localBounds, "teleport threshold bounds");
        if (boundary.triangles.empty())
        {
            return false;
        }
        const auto transform =
            core::Transform::FromSkyrimReference(reference.position, reference.rotation, reference.scale);
        std::vector<core::Vec3> placed;
        for (const auto vertex : boundary.vertices)
        {
            const auto point = transform.ApplyPoint(vertex);
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                return false;
            }
            placed.push_back(point);
        }
        source.sourceType = core::GeometrySourceType::NavigationExclusion;
        source.navigationObstacle = true;
        source.confidence = 1.0F;
        source.collisionType = "teleport threshold bounds";
        const auto sourceIndex = output.scene.geometrySources.size();
        output.scene.geometrySources.push_back(source);
        const auto vertexBase = static_cast<std::uint32_t>(output.scene.mesh.vertices.size());
        report.meshVertexOffset = vertexBase;
        report.meshTriangleOffset = output.scene.mesh.triangles.size();
        output.scene.mesh.vertices.insert(output.scene.mesh.vertices.end(), placed.begin(), placed.end());
        for (auto triangle : boundary.triangles)
        {
            for (auto &vertex : triangle.vertices)
            {
                vertex += vertexBase;
            }
            output.scene.mesh.triangles.push_back(triangle);
            output.scene.triangleProvenance.push_back(
                {sourceIndex, output.scene.mesh.triangles.size() - 1 - report.meshTriangleOffset});
        }
        report.vertices = boundary.vertices.size();
        report.triangles = boundary.triangles.size();
        report.sourceType = "navigation_exclusion";
        report.collisionType = source.collisionType;
        report.shapes = boundary.shapes;
        output.scene.coverage.push_back(
            {core::GeometryCoverage::Found, source, "exterior teleport threshold navigation boundary"});
        return true;
    }

    /** Bound valid transformed fuel triangles, using flames only when fuel is absent.
     * Glow and distortion billboards never define the footprint. Bounds use model-local Skyrim units;
     * the rectangular box is policy geometry, not Havok collision.
     */
    [[nodiscard]] TriangleGeometry BuildFireExclusion(const nifly::NifFile &nif)
    {
        FireFootprint fuelFootprint;
        FireFootprint flameFootprint;
        for (auto *shape : nif.GetShapes())
        {
            auto name = shape->name.get();
            std::transform(name.begin(), name.end(), name.begin(),
                           [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            const bool fuel = name.find("coal") != std::string::npos || name.find("ember") != std::string::npos ||
                              name.find("log") != std::string::npos;
            const bool flame = name.find("flame") != std::string::npos;
            if (!fuel && !flame)
            {
                continue;
            }
            CollectFireFootprint(nif, shape, fuel ? fuelFootprint : flameFootprint);
        }
        const bool hasFuel = !fuelFootprint.points.empty();
        auto &footprint = hasFuel ? fuelFootprint : flameFootprint;
        navmesh::core::AABB bounds;
        for (const auto &point : footprint.points)
        {
            bounds.Expand(point);
        }
        auto result = BuildBoundsExclusion(bounds, hasFuel ? "fire fuel bounds" : "fire flame bounds");
        result.invalidIndices = footprint.invalidIndices;
        result.degenerateTriangles = footprint.degenerateTriangles;
        return result;
    }

    /** Identify a hearth's recessed bed from model geometry, excluding its rim and surrounding floor.
     * The horizontal plane with the greatest projected area defines the surrounding floor.
     * Triangles descending below that plane define the bed's rectangular bounds. Missing floor/recess
     * evidence leaves invalid bounds, so a model name alone cannot enlarge a fire exclusion.
     */
    [[nodiscard]] navmesh::core::AABB FindHearthBedBounds(const nifly::NifFile &nif)
    {
        FireFootprint geometry;
        for (auto *shape : nif.GetShapes())
        {
            CollectFireFootprint(nif, shape, geometry);
        }
        constexpr float planeTolerance = 0.001F;
        std::map<float, double> floorAreas;
        for (std::size_t index = 0; index + 2 < geometry.points.size(); index += 3)
        {
            const auto &a = geometry.points[index];
            const auto &b = geometry.points[index + 1];
            const auto &c = geometry.points[index + 2];
            if (std::max({a.z, b.z, c.z}) - std::min({a.z, b.z, c.z}) <= planeTolerance)
            {
                const auto height = std::round((a.z + b.z + c.z) / (3 * planeTolerance)) * planeTolerance;
                floorAreas[height] += std::abs((static_cast<double>(b.x) - a.x) * (c.y - a.y) -
                                               (static_cast<double>(b.y) - a.y) * (c.x - a.x));
            }
        }
        navmesh::core::AABB bounds;
        if (floorAreas.empty())
        {
            return bounds;
        }
        const auto floor = std::max_element(floorAreas.begin(), floorAreas.end(),
                                            [](const auto &a, const auto &b) { return a.second < b.second; })
                               ->first;
        for (std::size_t index = 0; index + 2 < geometry.points.size(); index += 3)
        {
            const auto &a = geometry.points[index];
            const auto &b = geometry.points[index + 1];
            const auto &c = geometry.points[index + 2];
            if (std::min({a.z, b.z, c.z}) < floor - planeTolerance)
            {
                bounds.Expand(a);
                bounds.Expand(b);
                bounds.Expand(c);
            }
        }
        if (bounds.IsValid())
        {
            bounds.max.z = std::max(bounds.max.z, floor);
        }
        return bounds;
    }

    [[nodiscard]] NifGeometry LoadNif(const std::filesystem::path &path, bool includeRender, bool includeHearth,
                                      bool operableActivator, bool passageAsset)
    {
        NifGeometry result;
        nifly::NifFile nif;
        if (nif.Load(navmesh::core::NativeFilePath(path)) != 0 || !nif.IsValid())
        {
            return result;
        }
        result.version = nif.GetHeader().GetVersion().String();
        const auto operableNodes = operableActivator ? FindOperableNodes(nif, passageAsset) : OperableNodes{};
        result.operablePassage = !operableNodes.empty();
        LoadPackedCollision(nif, result.collision, result.collisionShapeTypes, result.nonSolidCollision, operableNodes,
                            result.operablePartsExcluded, result.solidCollision, result.incompleteCollision);
        if (includeHearth)
        {
            result.hearthBedBounds = FindHearthBedBounds(nif);
        }
        if (!includeRender)
        {
            return result;
        }
        result.fireExclusion = BuildFireExclusion(nif);
        for (auto *shape : nif.GetShapes())
        {
            if (IsOperablePart(nif, shape, operableNodes))
            {
                result.operablePartsExcluded = true;
                continue;
            }
            std::vector<nifly::Vector3> vertices;
            std::vector<nifly::Triangle> triangles;
            if (!nif.GetVertsForShape(shape, vertices) || !shape->GetTriangles(triangles))
            {
                continue;
            }
            const auto transform = ModelSpaceTransform(nif, shape);
            if (!transform)
            {
                continue;
            }
            const auto base = static_cast<std::uint32_t>(result.render.vertices.size());
            result.render.shapes.push_back(shape->name.get());
            result.render.vertices.reserve(result.render.vertices.size() + vertices.size());
            for (const auto &vertex : vertices)
            {
                const auto transformed = *transform * vertex;
                result.render.vertices.push_back({transformed.x, transformed.y, transformed.z});
            }
            for (const auto &triangle : triangles)
            {
                if (triangle.p1 >= vertices.size() || triangle.p2 >= vertices.size() || triangle.p3 >= vertices.size())
                {
                    ++result.render.invalidIndices;
                    continue;
                }
                if (triangle.p1 == triangle.p2 || triangle.p1 == triangle.p3 || triangle.p2 == triangle.p3)
                {
                    ++result.render.degenerateTriangles;
                    continue;
                }
                result.render.triangles.push_back({{base + triangle.p1, base + triangle.p2, base + triangle.p3}});
            }
        }
        return result;
    }

    [[nodiscard]] std::filesystem::path ModelRelativePath(const std::string &modelPath)
    {
        auto normalized = modelPath;
        for (auto &character : normalized)
        {
            character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
            if (character == '\\')
            {
                character = '/';
            }
        }
        auto relativePath = navmesh::core::PathFromUtf8(normalized);
        const auto first = relativePath.begin();
        if (first == relativePath.end() || *first != "meshes")
        {
            relativePath = std::filesystem::path("meshes") / relativePath;
        }
        return relativePath;
    }

    /// A resolved, enabled static hearth bed and its model-to-world frame, independent of fire placement order.
    struct HearthPlacement
    {
        std::uint32_t referenceId{};
        navmesh::core::AABB bounds;
        navmesh::core::Transform transform;
        float scale{};
    };

    /// Invert the uniform scale and orthogonal rotation of a placed Skyrim reference.
    [[nodiscard]] navmesh::core::Vec3 HearthLocalPoint(const HearthPlacement &hearth, navmesh::core::Vec3 worldPoint)
    {
        const auto &matrix = hearth.transform.matrix;
        const auto relative = worldPoint - navmesh::core::Vec3{matrix[12], matrix[13], matrix[14]};
        const auto scaleSquared = static_cast<double>(hearth.scale) * hearth.scale;
        return {static_cast<float>((matrix[0] * relative.x + matrix[1] * relative.y + matrix[2] * relative.z) /
                                   scaleSquared),
                static_cast<float>((matrix[4] * relative.x + matrix[5] * relative.y + matrix[6] * relative.z) /
                                   scaleSquared),
                static_cast<float>((matrix[8] * relative.x + matrix[9] * relative.y + matrix[10] * relative.z) /
                                   scaleSquared)};
    }

    /** Fit fire boxes to containing hearth beds after every source has been decoded.
     * Matching occurs in each hearth's model frame and requires the fire center to lie within the bed in XY
     * together with overlapping vertical extents. The closest bed wins, with reference identity breaking ties.
     * Only copied scene vertices change; shared model/placement cache entries and triangle identities stay valid.
     */
    void FitFireExclusionsToHearths(navmesh::skyrim::GeometryExtraction &geometry,
                                    const std::vector<HearthPlacement> &hearths)
    {
        for (auto &report : geometry.references)
        {
            if (report.sourceType != "navigation_exclusion")
            {
                continue;
            }
            const auto sourceIndex = geometry.scene.triangleProvenance[report.meshTriangleOffset].geometrySource;
            auto &source = geometry.scene.geometrySources[sourceIndex];
            // An uncertain fire occupies a world volume and cannot be fitted to a single hearth pose.
            if (source.solidBounds)
            {
                continue;
            }
            const HearthPlacement *matched = nullptr;
            navmesh::core::AABB matchedFireBounds;
            float closestDistance = std::numeric_limits<float>::max();
            for (const auto &hearth : hearths)
            {
                navmesh::core::AABB fireBounds;
                for (std::size_t index = report.meshVertexOffset; index < report.meshVertexOffset + report.vertices;
                     ++index)
                {
                    fireBounds.Expand(HearthLocalPoint(hearth, geometry.scene.mesh.vertices[index]));
                }
                const auto center = fireBounds.Center();
                if (center.x < hearth.bounds.min.x || center.x > hearth.bounds.max.x ||
                    center.y < hearth.bounds.min.y || center.y > hearth.bounds.max.y ||
                    fireBounds.min.z > hearth.bounds.max.z || fireBounds.max.z < hearth.bounds.min.z)
                {
                    continue;
                }
                const auto offset = center - hearth.bounds.Center();
                const auto distance =
                    (offset.x * offset.x + offset.y * offset.y + offset.z * offset.z) * hearth.scale * hearth.scale;
                if (distance < closestDistance ||
                    (distance == closestDistance && matched && hearth.referenceId < matched->referenceId))
                {
                    matched = &hearth;
                    matchedFireBounds = fireBounds;
                    closestDistance = distance;
                }
            }
            if (!matched)
            {
                continue;
            }
            auto bounds = matched->bounds;
            bounds.min.z = std::min(bounds.min.z, matchedFireBounds.min.z);
            bounds.max.z = std::max(bounds.max.z, matchedFireBounds.max.z);
            const auto fitted = BuildBoundsExclusion(bounds, "fire hearth bounds");
            for (std::size_t index = 0; index < fitted.vertices.size(); ++index)
            {
                geometry.scene.mesh.vertices[report.meshVertexOffset + index] =
                    matched->transform.ApplyPoint(fitted.vertices[index]);
            }
            report.collisionType = fitted.shapes.front();
            report.shapes = fitted.shapes;
            source.collisionType = report.collisionType;
            for (auto &displaySource : geometry.scene.geometrySources)
            {
                if (displaySource.reference.formId == report.formId &&
                    displaySource.sourceType == navmesh::core::GeometrySourceType::RenderFallback)
                {
                    displaySource.collisionType = "render mesh (supplemental; collision: fire hearth bounds)";
                }
            }
            for (auto &coverage : geometry.scene.coverage)
            {
                if (coverage.source.reference.formId == report.formId &&
                    coverage.source.sourceType == navmesh::core::GeometrySourceType::NavigationExclusion)
                {
                    coverage.source = source;
                    coverage.detail = std::format("fire navigation exclusion fitted to recessed hearth REFR {:08X}",
                                                  matched->referenceId);
                }
            }
        }
    }

    [[nodiscard]] bool IsFilteredReference(const navmesh::core::Reference &reference)
    {
        if (reference.editorMarker)
        {
            return true;
        }
        // Indexing a base for runtime rule resolution does not make every ordinary
        // placement navigation input. Additional classes require an effective BOS
        // placement; unclassified reference-local models remain eligible.
        const auto &type = reference.baseRecordType;
        const bool ordinaryBase = type.empty() || type == "STAT" || type == "MSTT" || type == "ACTI" ||
                                  type == "TREE" || type == "FLOR" || type == "CONT" || type == "FURN";
        if (reference.inputConfig.empty() && !ordinaryBase)
        {
            return true;
        }
        // Actors and transient or decorative effects do not provide stable scene support.
        // Operable doors are traversable portals. Their closed model pose cannot
        // supply permanent obstruction or floor evidence; surrounding static
        // frames and thresholds remain separate collision placements.
        if (reference.recordType == "ACHR" || reference.baseRecordType == "DOOR" ||
            reference.baseRecordType == "HAZD" || reference.baseRecordType == "PROJ" ||
            reference.baseRecordType == "EXPL")
        {
            return true;
        }
        if (IsFurniture(reference))
        {
            return false;
        }
        auto path = reference.modelPath;
        std::transform(path.begin(), path.end(), path.begin(),
                       [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
        std::replace(path.begin(), path.end(), '/', '\\');
        path.insert(path.begin(), '\\');
        return (!IsFireEffect(reference) && path.find("\\effects\\") != std::string::npos) ||
               (IsAnimatedAsset(reference.modelPath) && reference.baseRecordType != "ACTI") ||
               (!IsFireEffect(reference) && path.find("\\fx\\") != std::string::npos);
    }

    [[nodiscard]] std::string AssetKey(const std::filesystem::path &path)
    {
        auto key = navmesh::core::PathToUtf8(path);
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    }

    bool ExtractBsaModels(const std::filesystem::path &dataDirectory, const std::filesystem::path &cacheDirectory,
                          const navmesh::core::Cell &cell, const navmesh::skyrim::ModelAssetSources *assets,
                          const std::function<bool(const std::filesystem::path &)> &alreadyDecoded,
                          const std::function<bool(const std::set<std::string> &)> &extract)
    {
        std::set<std::string> requested;
        for (const auto &reference : cell.references)
        {
            if (reference.modelPath.empty() || reference.initiallyDisabled || reference.deleted ||
                IsFilteredReference(reference))
            {
                continue;
            }
            const auto relative = ModelRelativePath(reference.modelPath);
            if (assets && assets->looseModels.contains(AssetKey(relative)))
            {
                continue;
            }
            if (std::filesystem::exists(navmesh::core::NativeFilePath(dataDirectory / relative)))
            {
                continue;
            }
            if (alreadyDecoded(relative) ||
                std::filesystem::exists(navmesh::core::NativeFilePath(cacheDirectory / relative)))
            {
                continue;
            }
            auto logical = navmesh::core::PathToUtf8(relative);
            std::replace(logical.begin(), logical.end(), '/', '\\');
            std::transform(logical.begin(), logical.end(), logical.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            requested.insert(logical);
        }
        if (requested.empty())
        {
            return true;
        }
        if (cacheDirectory.empty())
        {
            return navmesh::skyrim::ModelArchives(dataDirectory, assets).empty();
        }
        if (!extract(requested))
        {
            return false;
        }
        return true;
    }
} // namespace

namespace navmesh::skyrim
{
    std::string DescribeGeometryFailure(const GeometryFailure &failure)
    {
        return std::format("{} geometry: '{}' (provider '{}'); REFR {:08X} in '{}', base {:08X} in '{}': {}",
                           failure.category, failure.model, failure.provider, failure.reference.formId,
                           failure.reference.plugin, failure.baseObject.formId, failure.baseObject.plugin,
                           failure.reason);
    }

    bool IsHearthModelPath(const std::string &modelPath)
    {
        return IsHearthAsset(modelPath);
    }

    bool IsPersistentFireReference(const core::Reference &reference)
    {
        return IsFireEffect(reference);
    }

    namespace
    {
        /// Cache identities distinguish asset revisions and render/collision extraction policies.
        std::string ModelKey(const std::filesystem::path &path, bool includeRender)
        {
            std::error_code error;
            const auto nativePath = core::NativeFilePath(path);
            const auto size = std::filesystem::file_size(nativePath, error);
            const auto stamp = std::filesystem::last_write_time(nativePath, error).time_since_epoch().count();
            return navmesh::core::PathToUtf8(path.lexically_normal()) + ":" + std::to_string(size) + ":" +
                   std::to_string(stamp) + (includeRender ? ":display" : ":navigation");
        }
        std::size_t GeometryBytes(const TriangleGeometry &geometry)
        {
            std::size_t strings = geometry.shapes.capacity() * sizeof(std::string);
            for (const auto &shape : geometry.shapes)
            {
                strings += shape.capacity();
            }
            return strings + sizeof(geometry) + geometry.vertices.capacity() * sizeof(core::Vec3) +
                   geometry.triangles.capacity() * sizeof(core::Triangle);
        }
        struct PlacedGeometry
        {
            core::Mesh mesh;
            std::size_t invalidVertices{};
        };
    } // namespace

    struct ModelGeometryCache::Impl
    {
        template <class T> struct Entry
        {
            std::shared_ptr<const T> value;
            std::size_t bytes{}, used{};
        };
        std::unique_ptr<BsaModelExtractor> archives;
        std::filesystem::path archiveSnapshot;

        bool ExtractArchives(const std::set<std::string> &requested, const std::filesystem::path &dataDirectory,
                             const std::filesystem::path &snapshot, const ModelAssetSources *assets,
                             std::vector<BsaFailure> &failures)
        {
            std::lock_guard lock(mutex);
            if (!archives || archiveSnapshot != snapshot)
            {
                archives = std::make_unique<BsaModelExtractor>(ModelArchives(dataDirectory, assets), snapshot);
                archiveSnapshot = snapshot;
            }
            const bool complete = archives->Extract(requested);
            failures = archives->Failures();
            return complete;
        }

        mutable std::mutex mutex;
        std::size_t budget{}, clock{};
        ModelCacheStatistics statistics;
        std::map<std::string, Entry<NifGeometry>> models;
        std::map<std::string, Entry<PlacedGeometry>> placements;

        /// A combined LRU bounds derived geometry and negative entries; pinned values survive eviction.
        void Evict()
        {
            while (statistics.retainedBytes > budget)
            {
                auto oldestModel = std::min_element(models.begin(), models.end(), [](const auto &a, const auto &b)
                                                    { return a.second.used < b.second.used; });
                auto oldestPlacement =
                    std::min_element(placements.begin(), placements.end(),
                                     [](const auto &a, const auto &b) { return a.second.used < b.second.used; });
                if (oldestModel != models.end() &&
                    (oldestPlacement == placements.end() || oldestModel->second.used < oldestPlacement->second.used))
                {
                    statistics.retainedBytes -= oldestModel->second.bytes;
                    models.erase(oldestModel);
                }
                else if (oldestPlacement != placements.end())
                {
                    statistics.retainedBytes -= oldestPlacement->second.bytes;
                    placements.erase(oldestPlacement);
                }
                else
                {
                    break;
                }
            }
        }
        bool ContainsReadableModel(const std::string &key)
        {
            std::lock_guard lock(mutex);
            const auto found = models.find(key);
            return found != models.end() && !found->second.value->version.empty();
        }
        std::shared_ptr<const NifGeometry> Model(const std::filesystem::path &path, bool includeRender,
                                                 bool includeHearth, bool operableActivator, bool passageAsset,
                                                 const std::string &key)
        {
            std::lock_guard lock(mutex);
            if (auto found = models.find(key); found != models.end())
            {
                found->second.used = ++clock;
                ++statistics.modelHits;
                return found->second.value;
            }
            std::shared_ptr<const NifGeometry> value;
            try
            {
                value = std::make_shared<const NifGeometry>(
                    LoadNif(path, includeRender, includeHearth, operableActivator, passageAsset));
            }
            catch (const std::bad_alloc &)
            {
                throw std::runtime_error("Memory allocation failed while decoding NIF '" +
                                         navmesh::core::PathToUtf8(path) +
                                         "'. Close other applications, lower Working memory budget and Generation "
                                         "workers, then retry. If the same NIF still fails, inspect that model for "
                                         "corrupt or unsupported data.");
            }
            catch (const std::length_error &)
            {
                throw std::runtime_error("NIF decoder allocation limit exceeded for '" + core::PathToUtf8(path) + "'");
            }
            catch (const std::exception &error)
            {
                NifGeometry unreadable;
                unreadable.readFailure = error.what();
                value = std::make_shared<const NifGeometry>(std::move(unreadable));
            }
            ++statistics.modelsDecoded;
            std::size_t shapeBytes = value->collisionShapeTypes.capacity() * sizeof(std::string);
            for (const auto &shape : value->collisionShapeTypes)
            {
                shapeBytes += shape.capacity();
            }
            const auto bytes = GeometryBytes(value->render) + GeometryBytes(value->collision) +
                               GeometryBytes(value->fireExclusion) + key.capacity() + sizeof(NifGeometry) -
                               3 * sizeof(TriangleGeometry) + shapeBytes + value->version.capacity() +
                               value->readFailure.capacity() + sizeof(Entry<NifGeometry>) + 64;
            if (bytes <= budget)
            {
                models.emplace(key, Entry<NifGeometry>{value, bytes, ++clock});
                statistics.retainedBytes += bytes;
                Evict();
            }
            return value;
        }
        std::shared_ptr<const PlacedGeometry> Placement(const TriangleGeometry &geometry,
                                                        const core::Transform &transform, std::string key)
        {
            key.append(reinterpret_cast<const char *>(transform.matrix.data()), sizeof(transform.matrix));
            std::lock_guard lock(mutex);
            if (auto found = placements.find(key); found != placements.end())
            {
                found->second.used = ++clock;
                ++statistics.placementHits;
                return found->second.value;
            }
            auto value = std::make_shared<PlacedGeometry>();
            value->mesh.triangles = geometry.triangles;
            value->mesh.vertices.reserve(geometry.vertices.size());
            for (const auto vertex : geometry.vertices)
            {
                const auto placed = transform.ApplyPoint(vertex);
                if (!IsFiniteVec3(placed))
                {
                    ++value->invalidVertices;
                }
                value->mesh.vertices.push_back(placed);
            }
            ++statistics.placementsBuilt;
            const auto bytes = sizeof(PlacedGeometry) + key.size() +
                               value->mesh.vertices.capacity() * sizeof(core::Vec3) +
                               value->mesh.triangles.capacity() * sizeof(core::Triangle);
            if (bytes <= budget)
            {
                placements.emplace(key, Entry<PlacedGeometry>{value, bytes, ++clock});
                statistics.retainedBytes += bytes;
                Evict();
            }
            return value;
        }
    };
    ModelGeometryCache::ModelGeometryCache(std::size_t byteBudget) : impl_(std::make_unique<Impl>())
    {
        impl_->budget = byteBudget;
    }
    ModelGeometryCache::~ModelGeometryCache() = default;
    ModelCacheStatistics ModelGeometryCache::Statistics() const
    {
        std::lock_guard lock(impl_->mutex);
        return impl_->statistics;
    }

    GeometryExtraction ExtractGeometry(const std::filesystem::path &dataDirectory, const core::Cell &cell,
                                       const std::filesystem::path &cacheDirectory,
                                       const GeometryProgressCallback &progress,
                                       const GeometryCancellationCallback &cancelled, const ModelAssetSources *assets,
                                       ModelGeometryCache *modelCache, bool navigationOnly)
    {
        GeometryExtraction output;
        ModelGeometryCache localCache;
        auto &cache = modelCache ? *modelCache : localCache;
        const bool snapshotOwned =
            std::filesystem::exists(core::NativeFilePath(cacheDirectory / ".navmesh-assets.json"));
        const auto cachedKey = [&](const std::filesystem::path &relative)
        {
            return navmesh::core::PathToUtf8((cacheDirectory / relative).lexically_normal()) +
                   (navigationOnly ? ":navigation-snapshot" : ":display-snapshot") +
                   (IsHearthAsset(navmesh::core::PathToUtf8(relative)) ? ":hearth" : "");
        };
        std::vector<BsaFailure> archiveFailures;
        output.archiveSearchComplete = ExtractBsaModels(
            dataDirectory, cacheDirectory, cell, assets,
            [&](const auto &relative)
            {
                const auto key = cachedKey(relative);
                return snapshotOwned && (cache.impl_->ContainsReadableModel(key) ||
                                         cache.impl_->ContainsReadableModel(key + ":operable-activator") ||
                                         cache.impl_->ContainsReadableModel(key + ":operable-activator:passage"));
            },
            [&](const auto &requested)
            {
                return cache.impl_->ExtractArchives(requested, dataDirectory, cacheDirectory, assets, archiveFailures);
            });
        std::set<std::uint32_t> archiveFailureReferences;
        for (const auto &failure : archiveFailures)
        {
            for (const auto &reference : cell.references)
            {
                if (AssetKey(ModelRelativePath(reference.modelPath)) == failure.model)
                {
                    output.failures.push_back(
                        {"archive_lookup",
                         reference.modelPath,
                         core::PathToUtf8(failure.archive),
                         failure.reason,
                         {reference.sourcePlugin, reference.id, reference.recordType},
                         {reference.basePlugin, reference.baseObjectId, reference.baseRecordType}});
                }
            }
        }
        if (!output.archiveSearchComplete && output.failures.empty())
        {
            if (!cacheDirectory.empty())
            {
                throw std::runtime_error("Required archive search failed without recoverable asset evidence");
            }
            for (const auto &reference : cell.references)
            {
                if (!reference.modelPath.empty() && !reference.deleted && !reference.initiallyDisabled &&
                    !IsFilteredReference(reference))
                {
                    output.failures.push_back(
                        {"archive_lookup",
                         reference.modelPath,
                         {},
                         "Archive lookup requires a writable cache directory",
                         {reference.sourcePlugin, reference.id, reference.recordType},
                         {reference.basePlugin, reference.baseObjectId, reference.baseRecordType}});
                }
            }
        }
        const auto totalReferences = cell.references.size();
        std::vector<HearthPlacement> hearths;
        for (std::size_t referenceIndex{}; referenceIndex < totalReferences; ++referenceIndex)
        {
            if (cancelled && cancelled())
            {
                return output;
            }
            if (progress)
            {
                progress(referenceIndex, totalReferences);
            }
            const auto &reference = cell.references[referenceIndex];
            const bool fireEffect = IsFireEffect(reference);
            const bool hearthAsset = IsHearthAsset(reference.modelPath);
            const bool includeRender = !navigationOnly || fireEffect;
            GeometryReferenceReport report{.formId = reference.id,
                                           .baseFormId = reference.baseObjectId,
                                           .recordType = reference.recordType,
                                           .modelPath = reference.modelPath,
                                           .position = reference.position,
                                           .rotation = reference.rotation,
                                           .scale = reference.scale};
            core::GeometrySource source{
                .modelPath = reference.modelPath,
                .reference = {reference.sourcePlugin, reference.id, reference.recordType},
                .baseObject = {reference.basePlugin, reference.baseObjectId, reference.baseRecordType},
                .navigationObstacle = IsLandscapeRock(reference.modelPath) || IsFurniture(reference) ||
                                      IsFixedClutter(reference) || fireEffect || reference.envelope.has_value(),
                .inputConfig = reference.inputConfig};
            if (reference.deleted || reference.initiallyDisabled || reference.editorMarker)
            {
                report.failure = reference.deleted        ? "winning reference record is deleted"
                                 : reference.editorMarker ? "base record supplies an editor-only marker"
                                                          : "winning reference record is initially disabled";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            if (!cell.isInterior && AppendTeleportBoundary(output, reference, source, report))
            {
                output.references.push_back(std::move(report));
                continue;
            }
            if (reference.modelPath.empty())
            {
                report.failure = "reference has no model path";
                output.scene.coverage.push_back({core::GeometryCoverage::Missing, std::move(source), report.failure});
                output.references.push_back(std::move(report));
                continue;
            }
            if (IsFilteredReference(reference))
            {
                report.failure = "excluded by navigation policy (base class without BOS placement, "
                                 "transient/decorative effect, door, "
                                 "animated, or actor reference)";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            ++output.referencesWithModels;
            const auto relativePath = ModelRelativePath(reference.modelPath);
            const auto loosePath = dataDirectory / relativePath;
            const auto cachedPath = cacheDirectory.empty() ? loosePath : cacheDirectory / relativePath;
            const auto looseWinner = assets ? assets->looseModels.find(AssetKey(relativePath))
                                            : std::unordered_map<std::string, std::filesystem::path>::const_iterator{};
            const auto modelPath = assets && looseWinner != assets->looseModels.end()         ? looseWinner->second
                                   : std::filesystem::exists(core::NativeFilePath(loosePath)) ? loosePath
                                                                                              : cachedPath;
            const bool operableActivator = reference.baseRecordType == "ACTI";
            const auto modelIdentity =
                (snapshotOwned && modelPath == cachedPath
                     ? cachedKey(relativePath)
                     : ModelKey(modelPath, includeRender) + (hearthAsset ? ":hearth" : "")) +
                (operableActivator
                     ? (IsPassageAsset(reference.modelPath) ? ":operable-activator:passage" : ":operable-activator")
                     : "");
            const auto model = cache.impl_->Model(modelPath, includeRender, hearthAsset, operableActivator,
                                                  IsPassageAsset(reference.modelPath), modelIdentity);
            const auto &nifGeometry = *model;
            const auto addFailure = [&](std::string category, std::string reason)
            {
                if (archiveFailureReferences.contains(reference.id))
                {
                    return;
                }
                output.failures.push_back({std::move(category), reference.modelPath, core::PathToUtf8(modelPath),
                                           std::move(reason), source.reference, source.baseObject});
            };
            if (operableActivator && IsAnimatedAsset(reference.modelPath) && !nifGeometry.version.empty() &&
                !nifGeometry.operablePassage)
            {
                report.failure = "excluded animated activator without an identified operable passage";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            if (hearthAsset && !reference.envelope &&
                (reference.baseRecordType == "STAT" || reference.baseRecordType == "MSTT") &&
                nifGeometry.hearthBedBounds.IsValid() &&
                nifGeometry.hearthBedBounds.min.x < nifGeometry.hearthBedBounds.max.x &&
                nifGeometry.hearthBedBounds.min.y < nifGeometry.hearthBedBounds.max.y && reference.scale > 0 &&
                std::isfinite(reference.scale) && IsFiniteVec3(reference.position) && IsFiniteVec3(reference.rotation))
            {
                hearths.push_back(
                    {reference.id, nifGeometry.hearthBedBounds,
                     core::Transform::FromSkyrimReference(reference.position, reference.rotation, reference.scale),
                     reference.scale});
            }
            const auto &mesh = fireEffect                                 ? nifGeometry.fireExclusion
                               : !nifGeometry.collision.triangles.empty() ? nifGeometry.collision
                                                                          : nifGeometry.render;
            const bool hasCollision = !fireEffect && !nifGeometry.collision.triangles.empty();
            if (nifGeometry.incompleteCollision || nifGeometry.collision.invalidIndices)
            {
                addFailure("unsupported", "NIF contains incomplete or unsupported solid collision");
            }
            if (mesh.triangles.empty() && nifGeometry.operablePartsExcluded && !nifGeometry.solidCollision)
            {
                report.failure = "excluded operable passage panels; no fixed geometry remains";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            if (!fireEffect && !hasCollision && nifGeometry.nonSolidCollision && !nifGeometry.solidCollision)
            {
                report.failure = "excluded non-solid Havok collision; render fallback is forbidden";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            if (nifGeometry.version.empty() || mesh.vertices.empty() || mesh.triangles.empty())
            {
                const auto exists = std::filesystem::exists(core::NativeFilePath(modelPath));
                const bool unavailableLooseWinner = assets && looseWinner != assets->looseModels.end() && !exists;
                const auto status = unavailableLooseWinner        ? core::GeometryCoverage::Unreadable
                                    : !exists                     ? core::GeometryCoverage::Missing
                                    : nifGeometry.version.empty() ? core::GeometryCoverage::Unreadable
                                                                  : core::GeometryCoverage::Unsupported;
                report.failure =
                    status == core::GeometryCoverage::Missing
                        ? (assets ? "model not found in resolved loose assets or enabled BSAs" : "missing loose NIF")
                    : unavailableLooseWinner                       ? "resolved loose model provider is unavailable"
                    : status == core::GeometryCoverage::Unreadable ? "NIF could not be read"
                    : fireEffect ? "fire effect contains no recognized fuel or flame triangle geometry"
                                 : "NIF contains no supported triangle geometry";
                if (nifGeometry.version.empty() || fireEffect || nifGeometry.solidCollision)
                {
                    addFailure(status == core::GeometryCoverage::Missing      ? "missing"
                               : status == core::GeometryCoverage::Unreadable ? "unreadable"
                                                                              : "unsupported",
                               nifGeometry.readFailure.empty() ? report.failure : nifGeometry.readFailure);
                }
                if (status == core::GeometryCoverage::Unsupported)
                {
                    ++output.modelsUnsupported;
                }
                else if (status == core::GeometryCoverage::Unreadable)
                {
                    ++output.modelsUnreadable;
                }
                else
                {
                    ++output.modelsMissing;
                }
                output.scene.coverage.push_back({status, std::move(source), report.failure});
                output.references.push_back(std::move(report));
                continue;
            }
            const bool hasEnvelope = (hasCollision || fireEffect) && reference.envelope.has_value();
            const auto envelopeGeometry = hasEnvelope ? CollisionEnvelope(mesh, reference) : TriangleGeometry{};
            const auto transform =
                hasEnvelope
                    ? core::Transform::Identity()
                    : core::Transform::FromSkyrimReference(reference.position, reference.rotation, reference.scale);
            if (hasEnvelope)
            {
                core::AABB bounds;
                for (const auto &vertex : envelopeGeometry.vertices)
                {
                    bounds.Expand(vertex);
                }
                source.solidBounds = bounds;
            }
            const auto appendSource = [&](const TriangleGeometry &sourceMesh, core::GeometrySource meshSource,
                                          core::Mesh &destination,
                                          std::vector<core::TriangleProvenance> &destinationProvenance)
            {
                const auto sourceIndex = output.scene.geometrySources.size();
                output.scene.geometrySources.push_back(std::move(meshSource));
                output.scene.nodes.push_back(
                    {.name = reference.editorId.empty() ? reference.modelPath : reference.editorId,
                     .localTransform = transform,
                     .geometrySource = sourceIndex});
                auto placementIdentity = modelIdentity;
                if (reference.envelope)
                {
                    // The envelope's world-space vertices vary independently of the identity transform.
                    placementIdentity.append(reinterpret_cast<const char *>(sourceMesh.vertices.data()),
                                             sourceMesh.vertices.size() * sizeof(core::Vec3));
                }
                const auto placed = cache.impl_->Placement(
                    sourceMesh, transform,
                    placementIdentity + ":" + SourceTypeName(output.scene.geometrySources[sourceIndex].sourceType));
                const auto vertexBase = static_cast<std::uint32_t>(destination.vertices.size());
                destination.vertices.insert(destination.vertices.end(), placed->mesh.vertices.begin(),
                                            placed->mesh.vertices.end());
                for (std::size_t index{}; index < placed->mesh.triangles.size(); ++index)
                {
                    auto triangle = placed->mesh.triangles[index];
                    for (auto &vertex : triangle.vertices)
                    {
                        vertex += vertexBase;
                    }
                    destination.triangles.push_back(triangle);
                    destinationProvenance.push_back({sourceIndex, index});
                }
                output.invalidVertices += placed->invalidVertices;
                return sourceIndex;
            };
            source.sourceType = fireEffect     ? core::GeometrySourceType::NavigationExclusion
                                : hasCollision ? core::GeometrySourceType::Collision
                                               : core::GeometrySourceType::RenderFallback;
            source.materialClass = hasCollision ? core::MaterialCollisionClass::HavokPackedTriangles
                                                : core::MaterialCollisionClass::RenderVisual;
            if (fireEffect)
            {
                source.collisionType = nifGeometry.fireExclusion.shapes.front();
            }
            else if (hasCollision)
            {
                source.collisionType = nifGeometry.collision.shapes.front();
            }
            else if (nifGeometry.collisionShapeTypes.empty())
            {
                source.collisionType = "render mesh (no supported collision)";
            }
            else
            {
                source.collisionType = "render mesh (unsupported Havok collision: ";
                for (std::size_t index{}; index < nifGeometry.collisionShapeTypes.size(); ++index)
                {
                    if (index != 0)
                    {
                        source.collisionType += " -> ";
                    }
                    source.collisionType += nifGeometry.collisionShapeTypes[index];
                }
                source.collisionType += ")";
            }
            source.confidence = hasCollision ? 1.0F : 0.35F;
            const auto &supportMesh = hasEnvelope ? envelopeGeometry : mesh;
            const auto supportSourceIndex =
                appendSource(supportMesh, source, output.scene.mesh, output.scene.triangleProvenance);
            report.meshVertexOffset = output.scene.mesh.vertices.size() - supportMesh.vertices.size();
            report.meshTriangleOffset = output.scene.mesh.triangles.size() - supportMesh.triangles.size();
            output.invalidIndices += supportMesh.invalidIndices;
            report.vertices = supportMesh.vertices.size();
            report.triangles = supportMesh.triangles.size();
            report.invalidIndices = supportMesh.invalidIndices;
            report.degenerateTriangles = supportMesh.degenerateTriangles;
            report.shapes = supportMesh.shapes;
            report.nifVersion = nifGeometry.version;
            report.sourceType = SourceTypeName(source.sourceType);
            report.collisionType = source.collisionType;
            report.usedRenderFallback = !hasCollision && !fireEffect;
            ++output.modelsLoaded;
            output.references.push_back(std::move(report));
            if (hasCollision || fireEffect)
            {
                if (hasCollision)
                {
                    ++output.collisionModelsLoaded;
                    output.collisionTriangles += supportMesh.triangles.size();
                }
                if (!navigationOnly && !reference.envelope && !nifGeometry.render.triangles.empty())
                {
                    auto renderSource = source;
                    renderSource.sourceType = core::GeometrySourceType::RenderFallback;
                    renderSource.materialClass = core::MaterialCollisionClass::RenderVisual;
                    renderSource.collisionType =
                        std::format("render mesh (supplemental; collision: {})", source.collisionType);
                    renderSource.confidence = 0.35F;
                    appendSource(nifGeometry.render, std::move(renderSource), output.scene.renderFallbackMesh,
                                 output.scene.renderFallbackTriangleProvenance);
                    ++output.renderFallbackModels;
                    output.renderFallbackTriangles += nifGeometry.render.triangles.size();
                    output.invalidIndices += nifGeometry.render.invalidIndices;
                }
            }
            else
            {
                ++output.renderFallbackModels;
                output.renderFallbackTriangles += supportMesh.triangles.size();
            }
            const auto &supportSource = output.scene.geometrySources[supportSourceIndex];
            output.scene.coverage.push_back(
                {core::GeometryCoverage::Found, supportSource,
                 nifGeometry.operablePartsExcluded ? "fixed passage geometry loaded; operable panels excluded"
                 : fireEffect   ? "fire navigation exclusion derived from visible fuel or flame bounds"
                 : hasCollision ? "Havok collision loaded; render mesh retained as display-only fallback"
                                : "render NIF loaded as low-confidence fallback; no supported collision"});
        }
        FitFireExclusionsToHearths(output, hearths);
        if (progress)
        {
            progress(totalReferences, totalReferences);
        }

        return output;
    }
    bool WriteGeometryObj(const std::filesystem::path &outputPath, const GeometryExtraction &geometry)
    {
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output)
        {
            return false;
        }
        for (const auto &vertex : geometry.scene.mesh.vertices)
        {
            output << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
        }
        for (const auto &reference : geometry.references)
        {
            if (reference.triangles == 0)
            {
                continue;
            }
            output << std::format("g REF_{:08X}_BASE_{:08X}\n", reference.formId, reference.baseFormId);
            for (std::size_t index = reference.meshTriangleOffset;
                 index < reference.meshTriangleOffset + reference.triangles; ++index)
            {
                const auto &triangle = geometry.scene.mesh.triangles[index];
                output << std::format("f {} {} {}\n", triangle.vertices[0] + 1, triangle.vertices[1] + 1,
                                      triangle.vertices[2] + 1);
            }
        }
        return true;
    }

    bool WriteGeometryJson(const std::filesystem::path &outputPath, const core::Cell &cell,
                           const GeometryExtraction &geometry, const reproducibility::ExportMetadata &metadata)
    {
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output)
        {
            return false;
        }
        output << "{\n  \"metadata\": " << reproducibility::ToJson(metadata, "    ") << ",\n";
        output << std::format(
            "  \"cell\": \"{:08X}\",\n  \"references\": {},\n  \"referencesWithModels\": {},\n  \"modelsLoaded\": "
            "{},\n  \"modelsMissing\": {},\n  \"modelsExcluded\": {},\n  \"modelsUnreadable\": {},\n  "
            "\"modelsUnsupported\": {},\n  \"collisionModelsLoaded\": {},\n  \"collisionTriangles\": {},\n  "
            "\"renderFallbackModels\": {},\n  \"renderFallbackTriangles\": {},\n  \"vertices\": {},\n  \"triangles\": "
            "{},\n  \"terrainSupported\": {},\n  \"collisionGeometrySupported\": {},\n  \"invalidVertices\": {},\n  "
            "\"invalidIndices\": {},\n  \"referenceDetails\": [\n",
            cell.id, cell.references.size(), geometry.referencesWithModels, geometry.modelsLoaded,
            geometry.modelsMissing, geometry.modelsExcluded, geometry.modelsUnreadable, geometry.modelsUnsupported,
            geometry.collisionModelsLoaded, geometry.collisionTriangles, geometry.renderFallbackModels,
            geometry.renderFallbackTriangles, geometry.scene.mesh.vertices.size(), geometry.scene.mesh.triangles.size(),
            geometry.terrainSupported ? "true" : "false", geometry.collisionGeometrySupported ? "true" : "false",
            geometry.invalidVertices, geometry.invalidIndices);
        for (std::size_t index = 0; index < geometry.references.size(); ++index)
        {
            const auto &reference = geometry.references[index];
            output << std::format(
                "    "
                "{{\"formId\":\"{:08X}\",\"baseFormId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\","
                "\"nifVersion\":\"{}\",\"sourceType\":\"{}\",\"collisionType\":\"{}\",\"usedRenderFallback\":{},"
                "\"shapes\":{},\"position\":[{},{},{}],\"rotation\":[{},{},{}],\"scale\":{},\"vertices\":{},"
                "\"triangles\":{},\"invalidIndices\":{},\"degenerateTriangles\":{},\"failure\":\"{}\"}}{}\n",
                reference.formId, reference.baseFormId, EscapeJson(reference.recordType),
                EscapeJson(reference.modelPath), EscapeJson(reference.nifVersion), EscapeJson(reference.sourceType),
                EscapeJson(reference.collisionType), reference.usedRenderFallback ? "true" : "false",
                JsonStringArray(reference.shapes), reference.position.x, reference.position.y, reference.position.z,
                reference.rotation.x, reference.rotation.y, reference.rotation.z, reference.scale, reference.vertices,
                reference.triangles, reference.invalidIndices, reference.degenerateTriangles,
                EscapeJson(reference.failure), index + 1 == geometry.references.size() ? "" : ",");
        }
        output << "  ],\n  \"coverage\": [\n";
        for (std::size_t index = 0; index < geometry.scene.coverage.size(); ++index)
        {
            const auto &entry = geometry.scene.coverage[index];
            const auto &source = entry.source;
            output << std::format("    "
                                  "{{\"status\":\"{}\",\"detail\":\"{}\",\"model\":\"{}\",\"reference\":{{\"plugin\":"
                                  "\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},\"baseObject\":{{\"plugin\":\"{"
                                  "}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},\"inputConfig\":\"{}\"}}{}\n",
                                  CoverageName(entry.status), EscapeJson(entry.detail), EscapeJson(source.modelPath),
                                  EscapeJson(source.reference.plugin), source.reference.formId,
                                  EscapeJson(source.reference.recordType), EscapeJson(source.baseObject.plugin),
                                  source.baseObject.formId, EscapeJson(source.baseObject.recordType),
                                  EscapeJson(source.inputConfig),
                                  index + 1 == geometry.scene.coverage.size() ? "" : ",");
        }
        output << "  ],\n  \"triangleProvenance\": [\n";
        for (std::size_t index = 0; index < geometry.scene.triangleProvenance.size(); ++index)
        {
            const auto &provenance = geometry.scene.triangleProvenance[index];
            const auto &source = geometry.scene.geometrySources[provenance.geometrySource];
            const auto terrain =
                provenance.terrain
                    ? std::format(",\"terrain\":{{\"cell\":[{},{}],\"landFormId\":\"{:08X}\",\"sample\":[{},{}]}}",
                                  provenance.terrain->cellX, provenance.terrain->cellY, provenance.terrain->landFormId,
                                  provenance.terrain->sampleX, provenance.terrain->sampleY)
                    : "";
            output << std::format(
                "    "
                "{{\"triangle\":{},\"sourceTriangle\":{},\"sourceType\":\"{}\",\"collisionType\":\"{}\",\"confidence\":"
                "{},\"model\":\"{}\",\"navigationObstacle\":{},\"reference\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\","
                "\"recordType\":\"{}\"}},"
                "\"baseObject\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},\"inputConfig\":\"{}"
                "\"{}}}{}\n",
                index, provenance.sourceTriangle, SourceTypeName(source.sourceType), EscapeJson(source.collisionType),
                source.confidence, EscapeJson(source.modelPath), source.navigationObstacle ? "true" : "false",
                EscapeJson(source.reference.plugin), source.reference.formId, EscapeJson(source.reference.recordType),
                EscapeJson(source.baseObject.plugin), source.baseObject.formId,
                EscapeJson(source.baseObject.recordType), EscapeJson(source.inputConfig), terrain,
                index + 1 == geometry.scene.triangleProvenance.size() ? "" : ",");
        }
        output << "  ]\n}\n";
        return true;
    }
} // namespace navmesh::skyrim
