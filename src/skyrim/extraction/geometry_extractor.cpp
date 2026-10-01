#include "skyrim/extraction/geometry_extractor.h"

#include <NifFile.hpp>
#include <bhk.hpp>
#include <cmath>
#include <fstream>
#include <format>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <unordered_set>

namespace
{
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
        std::vector<std::string> collisionShapeTypes;
        bool nonSolidCollision{};
        std::string version;
    };

    [[nodiscard]] navmesh::core::Vec3 ApplyRigidBodyTransform(const nifly::bhkRigidBody &body,
                                                              const nifly::Vector3 &vertex)
    {
        // Havok stores rigid-body rotation as xyzw quaternion and translation
        // separately, in Havok units. Convert the complete rigid-body result
        // to Skyrim units before applying the reference transform. Without
        // this, collision shrinks to roughly 1/70 of the render geometry.
        constexpr float skyrimUnitsPerHavokUnit = 69.99125F;
        const auto &q = body.rotation;
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        return {((1 - 2 * (yy + zz)) * vertex.x + 2 * (xy - wz) * vertex.y + 2 * (xz + wy) * vertex.z +
                 body.translation.x) *
                    skyrimUnitsPerHavokUnit,
                (2 * (xy + wz) * vertex.x + (1 - 2 * (xx + zz)) * vertex.y + 2 * (yz - wx) * vertex.z +
                 body.translation.y) *
                    skyrimUnitsPerHavokUnit,
                (2 * (xz - wy) * vertex.x + 2 * (yz + wx) * vertex.y + (1 - 2 * (xx + yy)) * vertex.z +
                 body.translation.z) *
                    skyrimUnitsPerHavokUnit};
    }
    void AddPackedTriangles(const nifly::hkPackedNiTriStripsData &data, const nifly::bhkRigidBody &body,
                            TriangleGeometry &result)
    {
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(result.vertices.size() + data.compressedVertData.size());
        for (const auto &vertex : data.compressedVertData)
        {
            result.vertices.push_back(ApplyRigidBodyTransform(body, vertex));
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

    void AddNiTriStripsTriangles(const nifly::NiTriStripsData &data, const nifly::bhkRigidBody &body,
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
            result.vertices.push_back(ApplyRigidBodyTransform(body, vertex));
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

    void AddCompressedBigTriangles(const nifly::bhkCompressedMeshShapeData &data, const nifly::bhkRigidBody &body,
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
            result.vertices.push_back(ApplyRigidBodyTransform(body, {vertex->x, vertex->y, vertex->z}));
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

    void AddCompressedChunkTriangles(const nifly::bhkCompressedMeshShapeData &data, const nifly::bhkRigidBody &body,
                                     TriangleGeometry &result)
    {
        for (auto chunk = data.chunks.cbegin(); chunk != data.chunks.cend(); ++chunk)
        {
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
                result.vertices.push_back(ApplyRigidBodyTransform(body, position));
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

    // SE static-world collision is normally bhkMoppBvTreeShape ->
    // bhkPackedNiTriStripsShape -> hkPackedNiTriStripsData.  Nifly retains the
    // packed vertices/triangles losslessly, so support that compact path first.
    // Older assets can instead use bhkNiTriStripsShape -> NiTriStripsData;
    // this is triangle data too, not a render-geometry approximation.
    // Other Havok primitives are intentionally reported as unsupported rather
    // than approximated with decorative render geometry.
    void LoadPackedCollision(const nifly::NifFile &nif, TriangleGeometry &result, std::vector<std::string> &shapeTypes,
                             bool &nonSolidCollision)
    {
        const auto &header = nif.GetHeader();
        std::unordered_set<std::uint32_t> reachablePacked;
        std::function<void(nifly::bhkShape *, const nifly::bhkRigidBody &)> visit =
            [&](nifly::bhkShape *shape, const nifly::bhkRigidBody &body)
        {
            if (!shape)
            {
                return;
            }
            const std::string shapeName = shape->GetBlockName();
            if (std::find(shapeTypes.begin(), shapeTypes.end(), shapeName) == shapeTypes.end())
            {
                shapeTypes.push_back(shapeName);
            }
            if (auto *mopp = dynamic_cast<nifly::bhkMoppBvTreeShape *>(shape))
            {
                visit(header.GetBlock(mopp->shapeRef), body);
                return;
            }
            if (auto *list = dynamic_cast<nifly::bhkListShape *>(shape))
            {
                for (const auto &child : list->subShapeRefs)
                {
                    visit(header.GetBlock(child), body);
                }
                return;
            }
            if (auto *packed = dynamic_cast<nifly::bhkPackedNiTriStripsShape *>(shape))
            {
                const auto id = header.GetBlockID(packed);
                if (reachablePacked.insert(id).second)
                {
                    if (const auto *data = header.GetBlock(packed->dataRef))
                    {
                        AddPackedTriangles(*data, body, result);
                        result.shapes.push_back("hkPackedNiTriStripsData");
                    }
                }
                return;
            }
            if (auto *triStrips = dynamic_cast<nifly::bhkNiTriStripsShape *>(shape))
            {
                const auto id = header.GetBlockID(triStrips);
                if (reachablePacked.insert(id).second)
                {
                    for (const auto &part : triStrips->partRefs)
                    {
                        if (const auto *data = header.GetBlock(part))
                        {
                            AddNiTriStripsTriangles(*data, body, result);
                            result.shapes.push_back("NiTriStripsData");
                        }
                    }
                }
                return;
            }
            if (auto *compressed = dynamic_cast<nifly::bhkCompressedMeshShape *>(shape))
            {
                const auto id = header.GetBlockID(compressed);
                if (reachablePacked.insert(id).second)
                {
                    if (const auto *data = header.GetBlock(compressed->dataRef))
                    {
                        const auto trianglesBefore = result.triangles.size();
                        AddCompressedBigTriangles(*data, body, result);
                        AddCompressedChunkTriangles(*data, body, result);
                        if (result.triangles.size() > trianglesBefore)
                        {
                            result.shapes.push_back("bhkCompressedMeshShape");
                        }
                    }
                }
            }
        };
        for (std::uint32_t block = 0; block < header.GetNumBlocks(); ++block)
        {
            const auto *collision = header.GetBlock<nifly::bhkNiCollisionObject>(block);
            if (!collision)
            {
                continue;
            }
            const auto *body =
                dynamic_cast<nifly::bhkRigidBody *>(header.GetBlock<nifly::NiObject>(collision->bodyRef));
            if (body && body->collisionResponse == nifly::RESPONSE_NONE)
            {
                nonSolidCollision = true;
                continue;
            }
            if (body)
            {
                visit(header.GetBlock(body->shapeRef), *body);
            }
        }
    }

    [[nodiscard]] NifGeometry LoadNif(const std::filesystem::path &path)
    {
        NifGeometry result;
        nifly::NifFile nif;
        if (nif.Load(path) != 0 || !nif.IsValid())
        {
            return result;
        }
        result.version = nif.GetHeader().GetVersion().String();
        LoadPackedCollision(nif, result.collision, result.collisionShapeTypes, result.nonSolidCollision);
        for (auto *shape : nif.GetShapes())
        {
            std::vector<nifly::Vector3> vertices;
            std::vector<nifly::Triangle> triangles;
            if (!nif.GetVertsForShape(shape, vertices) || !shape->GetTriangles(triangles))
            {
                continue;
            }
            nifly::MatTransform nodeTransform;
            nodeTransform.Clear();
            if (const auto *parentNode = nif.GetParentNode(shape); parentNode != nullptr)
            {
                nif.GetNodeTransformToGlobal(parentNode->name.get(), nodeTransform);
            }
            const auto base = static_cast<std::uint32_t>(result.render.vertices.size());
            result.render.shapes.push_back(shape->name.get());
            result.render.vertices.reserve(result.render.vertices.size() + vertices.size());
            for (const auto &vertex : vertices)
            {
                const auto transformed = nodeTransform.ApplyTransform(vertex);
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

    [[nodiscard]] std::string QuoteShell(const std::filesystem::path &path)
    {
        return "\"" + path.string() + "\"";
    }

    [[nodiscard]] std::filesystem::path ModelRelativePath(const std::string &modelPath)
    {
        auto normalized = modelPath;
        for (auto &character : normalized)
        {
            if (character == '\\')
            {
                character = '/';
            }
        }
        auto relativePath = std::filesystem::path(normalized);
        const auto first = relativePath.begin();
        if (first == relativePath.end() || first->string() != "meshes")
        {
            relativePath = std::filesystem::path("meshes") / relativePath;
        }
        return relativePath;
    }

    [[nodiscard]] bool IsVisualEffectModel(const std::string &modelPath)
    {
        auto normalized = modelPath;
        for (auto &character : normalized)
        {
            character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
            if (character == '/')
            {
                character = '\\';
            }
        }
        return normalized.rfind("effects\\", 0) == 0;
    }
    [[nodiscard]] bool IsFilteredReference(const navmesh::core::Reference &reference)
    {
        // These classes are not stable solid scene support.  The policy is
        // intentionally conservative until their collision semantics are
        // modelled explicitly.
        if (reference.recordType == "ACHR" || reference.baseRecordType == "FURN")
        {
            return true;
        }
        auto path = reference.modelPath;
        std::transform(path.begin(), path.end(), path.begin(),
                       [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return path.find("\\effects\\") != std::string::npos || path.find("\\animated\\") != std::string::npos ||
               path.find("\\fx\\") != std::string::npos;
    }

    void AppendGeometry(const TriangleGeometry &geometry, const navmesh::core::Transform &transform,
                        const std::size_t sourceIndex, navmesh::core::Mesh &mesh,
                        std::vector<navmesh::core::TriangleProvenance> &provenance, std::size_t &invalidVertices)
    {
        const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const auto &vertex : geometry.vertices)
        {
            const auto transformed = transform.ApplyPoint(vertex);
            if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z) ||
                std::abs(transformed.x) > 1.0e7F || std::abs(transformed.y) > 1.0e7F ||
                std::abs(transformed.z) > 1.0e7F)
            {
                ++invalidVertices;
            }
            mesh.vertices.push_back(transformed);
        }
        for (std::size_t triangleIndex{}; triangleIndex < geometry.triangles.size(); ++triangleIndex)
        {
            const auto &triangle = geometry.triangles[triangleIndex];
            mesh.triangles.push_back(
                {{base + triangle.vertices[0], base + triangle.vertices[1], base + triangle.vertices[2]}});
            provenance.push_back({sourceIndex, triangleIndex});
        }
    }

    [[nodiscard]] std::string AssetKey(const std::filesystem::path &path)
    {
        auto key = path.generic_string();
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    }

    void ExtractBsaModels(const std::filesystem::path &dataDirectory, const std::filesystem::path &cacheDirectory,
                          const navmesh::core::Cell &cell, const navmesh::skyrim::offline::ModelAssetSources *assets)
    {
        if (cacheDirectory.empty())
        {
            return;
        }
        std::filesystem::create_directories(cacheDirectory);
        const auto manifest = cacheDirectory / "requested_models.txt";
        std::ofstream manifestStream(manifest, std::ios::trunc);
        if (!manifestStream)
        {
            return;
        }
        std::size_t modelCount = 0;
        for (const auto &reference : cell.references)
        {
            if (reference.modelPath.empty() || reference.initiallyDisabled || reference.deleted)
            {
                continue;
            }
            const auto relative = ModelRelativePath(reference.modelPath);
            if (assets && assets->looseModels.contains(AssetKey(relative)))
            {
                continue;
            }
            if (std::filesystem::exists(dataDirectory / relative))
            {
                continue;
            }
            if (std::filesystem::exists(cacheDirectory / relative))
            {
                continue;
            }
            manifestStream << relative.string() << "\n";
            ++modelCount;
        }
        manifestStream.close();
        if (modelCount == 0)
        {
            return;
        }
        const auto script = std::filesystem::current_path() / "tools" / "extract_bsa_models.py";
        if (!std::filesystem::exists(script))
        {
            return;
        }
        std::string python = "python";
#ifdef _WIN32
        char *configuredPython{};
        std::size_t configuredSize{};
        if (_dupenv_s(&configuredPython, &configuredSize, "NAVMESH_PYTHON") == 0 && configuredPython &&
            *configuredPython)
        {
            python = QuoteShell(configuredPython);
        }
        std::free(configuredPython);
#else
        if (const auto *configuredPython = std::getenv("NAVMESH_PYTHON"); configuredPython && *configuredPython)
        {
            python = QuoteShell(configuredPython);
        }
#endif
        std::string command = python + " " + QuoteShell(script) + " --data " + QuoteShell(dataDirectory) +
                              " --output " + QuoteShell(cacheDirectory) + " --manifest " + QuoteShell(manifest);
        if (assets)
        {
            const auto archiveManifest = cacheDirectory / "archives.txt";
            std::ofstream archives(archiveManifest, std::ios::trunc);
            for (const auto &path : assets->archives)
            {
                archives << path.string() << "\n";
            }
            archives.close();
            command += " --archives " + QuoteShell(archiveManifest);
        }
        if (std::system(command.c_str()) != 0)
        {
            std::cerr << "BSA model extraction failed; install the Python dependencies in tools/requirements.txt or "
                         "set NAVMESH_PYTHON.\n";
        }
    }
} // namespace

namespace navmesh::skyrim::offline
{
    GeometryExtraction ExtractGeometry(const std::filesystem::path &dataDirectory, const core::Cell &cell,
                                       const std::filesystem::path &cacheDirectory,
                                       const GeometryProgressCallback &progress,
                                       const GeometryCancellationCallback &cancelled, const ModelAssetSources *assets)
    {
        GeometryExtraction output;
        ExtractBsaModels(dataDirectory, cacheDirectory, cell, assets);
        const auto totalReferences = cell.references.size();
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
                .baseObject = {reference.basePlugin, reference.baseObjectId, reference.baseRecordType}};
            if (reference.deleted || reference.initiallyDisabled)
            {
                report.failure = reference.deleted ? "winning reference record is deleted"
                                                   : "winning reference record is initially disabled";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
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
                report.failure = "excluded by navigation policy (effect, furniture, animated, or actor reference)";
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
            const auto modelPath = assets && looseWinner != assets->looseModels.end() ? looseWinner->second
                                   : std::filesystem::exists(loosePath)               ? loosePath
                                                                                      : cachedPath;
            const auto nifGeometry = LoadNif(modelPath);
            const auto &mesh = !nifGeometry.collision.triangles.empty() ? nifGeometry.collision : nifGeometry.render;
            const bool hasCollision = !nifGeometry.collision.triangles.empty();
            if (!hasCollision && nifGeometry.nonSolidCollision)
            {
                report.failure = "excluded non-solid Havok collision; render fallback is forbidden";
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            if (nifGeometry.version.empty() || mesh.vertices.empty() || mesh.triangles.empty())
            {
                const auto exists = std::filesystem::exists(modelPath);
                const auto status = !exists                       ? core::GeometryCoverage::Missing
                                    : nifGeometry.version.empty() ? core::GeometryCoverage::Unreadable
                                                                  : core::GeometryCoverage::Unsupported;
                report.failure =
                    status == core::GeometryCoverage::Missing
                        ? (assets ? "model not found in MO2 loose assets or enabled BSAs" : "missing loose NIF")
                    : status == core::GeometryCoverage::Unreadable ? "NIF could not be read"
                                                                   : "NIF contains no supported triangle geometry";
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
            if (IsVisualEffectModel(reference.modelPath))
            {
                report.failure = "excluded visual effect from support geometry";
                report.vertices = mesh.vertices.size();
                report.triangles = mesh.triangles.size();
                ++output.modelsLoaded;
                output.scene.coverage.push_back({core::GeometryCoverage::Excluded, std::move(source), report.failure});
                ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            const auto transform =
                core::Transform::FromSkyrimReference(reference.position, reference.rotation, reference.scale);
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
                AppendGeometry(sourceMesh, transform, sourceIndex, destination, destinationProvenance,
                               output.invalidVertices);
                return sourceIndex;
            };
            source.sourceType =
                hasCollision ? core::GeometrySourceType::Collision : core::GeometrySourceType::RenderFallback;
            source.materialClass = hasCollision ? core::MaterialCollisionClass::HavokPackedTriangles
                                                : core::MaterialCollisionClass::RenderVisual;
            if (hasCollision)
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
            const auto &supportMesh = hasCollision ? nifGeometry.collision : nifGeometry.render;
            const auto supportSourceIndex =
                appendSource(supportMesh, source, output.mesh, output.scene.triangleProvenance);
            report.meshVertexOffset = output.mesh.vertices.size() - supportMesh.vertices.size();
            report.meshTriangleOffset = output.mesh.triangles.size() - supportMesh.triangles.size();
            output.invalidIndices += supportMesh.invalidIndices;
            report.vertices = supportMesh.vertices.size();
            report.triangles = supportMesh.triangles.size();
            report.invalidIndices = supportMesh.invalidIndices;
            report.degenerateTriangles = supportMesh.degenerateTriangles;
            report.shapes = supportMesh.shapes;
            report.nifVersion = nifGeometry.version;
            report.sourceType = SourceTypeName(source.sourceType);
            report.collisionType = source.collisionType;
            report.usedRenderFallback = !hasCollision;
            ++output.modelsLoaded;
            output.references.push_back(std::move(report));
            if (hasCollision)
            {
                ++output.collisionModelsLoaded;
                output.collisionTriangles += supportMesh.triangles.size();
                if (!nifGeometry.render.triangles.empty())
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
                 hasCollision ? "packed Havok collision loaded; render mesh retained as display-only fallback"
                              : "render NIF loaded as low-confidence fallback; no supported collision"});
        }
        if (progress)
        {
            progress(totalReferences, totalReferences);
        }
        output.scene.mesh = output.mesh;
        return output;
    }
    bool WriteGeometryObj(const std::filesystem::path &outputPath, const GeometryExtraction &geometry)
    {
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output)
        {
            return false;
        }
        for (const auto &vertex : geometry.mesh.vertices)
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
                const auto &triangle = geometry.mesh.triangles[index];
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
            geometry.renderFallbackTriangles, geometry.mesh.vertices.size(), geometry.mesh.triangles.size(),
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
                                  "}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}}}}{}\n",
                                  CoverageName(entry.status), EscapeJson(entry.detail), EscapeJson(source.modelPath),
                                  EscapeJson(source.reference.plugin), source.reference.formId,
                                  EscapeJson(source.reference.recordType), EscapeJson(source.baseObject.plugin),
                                  source.baseObject.formId, EscapeJson(source.baseObject.recordType),
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
                "{},\"model\":\"{}\",\"reference\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},"
                "\"baseObject\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}}{}}}{}\n",
                index, provenance.sourceTriangle, SourceTypeName(source.sourceType), EscapeJson(source.collisionType),
                source.confidence, EscapeJson(source.modelPath), EscapeJson(source.reference.plugin),
                source.reference.formId, EscapeJson(source.reference.recordType), EscapeJson(source.baseObject.plugin),
                source.baseObject.formId, EscapeJson(source.baseObject.recordType), terrain,
                index + 1 == geometry.scene.triangleProvenance.size() ? "" : ",");
        }
        output << "  ]\n}\n";
        return true;
    }
} // namespace navmesh::skyrim::offline
