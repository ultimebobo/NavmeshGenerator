#include "app/candidate_cache.h"
#include "core/navmesh/generator.h"

#include "core/io/content_hash.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstring>
#include <fstream>
#include <utility>
#include <type_traits>
#include <zlib.h>

namespace navmesh::app::detail
{
    namespace
    {
        // This identity versions both storage and successful generation semantics.
        // Change its pipeline revision when successful output becomes incompatible;
        // executable rebuilds alone must not discard completed generation work.
        constexpr std::string_view Schema = "navmesh-candidate-cache-6/recast-pipeline-18";
        constexpr std::size_t MaximumBytes = 512ULL * 1024 * 1024;

        template <class Archive> void Fields(Archive &a, core::Vec3 &v)
        {
            a(v.x, v.y, v.z);
        }
        template <class Archive> void Fields(Archive &a, core::Triangle &v)
        {
            a(v.vertices);
        }
        template <class Archive> void Fields(Archive &a, core::AABB &v)
        {
            a(v.min, v.max);
        }
        template <class Archive> void Fields(Archive &a, core::NavPolygon &v)
        {
            a(v.vertices, v.neighbors, v.flags);
        }
        template <class Archive> void Fields(Archive &a, core::NavMesh &v)
        {
            a(v.id, v.vertices, v.polygons);
        }
        template <class Archive> void Fields(Archive &a, core::RecordProvenance &v)
        {
            a(v.plugin, v.formId, v.recordType);
        }
        template <class Archive> void Fields(Archive &a, core::GeometrySource &v)
        {
            a(v.modelPath, v.materialClass, v.sourceType, v.collisionType, v.confidence, v.reference, v.baseObject,
              v.navigationObstacle);
        }
        template <class Archive> void Fields(Archive &a, core::TerrainTriangleProvenance &v)
        {
            a(v.cellX, v.cellY, v.landFormId, v.sampleX, v.sampleY);
        }
        template <class Archive> void Fields(Archive &a, core::TriangleProvenance &v)
        {
            a(v.geometrySource, v.sourceTriangle, v.terrain);
        }
        template <class Archive> void Fields(Archive &a, core::NavigationProfile &v)
        {
            a(v.name, v.agentRadius, v.agentHeight, v.maxSlopeDegrees, v.stepHeight, v.clearance, v.weldTolerance,
              v.minimumRegionArea, v.contourSimplificationTolerance, v.cellBorderPolicy);
        }
        template <class Archive> void Fields(Archive &a, core::RecastSettings &v)
        {
            a(v.cellSize, v.cellHeight, v.maxSimplificationError, v.maxEdgeLength, v.mergeRegionAreaMultiplier);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateRegion &v)
        {
            a(v.id, v.area, v.polygons, v.sourceTriangles, v.geometrySources, v.reachesBorder, v.exitFormIds);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateExit &v)
        {
            a(v.referenceId, v.position, v.region, v.polygon);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateBorderLink &v)
        {
            a(v.polygon, v.edge, v.neighborNavmeshId, v.neighborPolygon, v.neighborEdge, v.generatedNeighborCell);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateContour &v)
        {
            a(v.region, v.closed, v.vertices);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateTopology &v)
        {
            a(v.valid, v.findings);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateStatistics &v)
        {
            a(v.inputTriangles, v.eligibleTriangles, v.rejectedSlope, v.rejectedClearance, v.rejectedObstruction,
              v.rejectedSource, v.rejectedDegenerate, v.rejectedSmallRegion, v.rejectedUnreachable,
              v.polygonsBeforeSimplification, v.outputPolygons);
        }
        template <class Archive> void Fields(Archive &a, core::CandidateNavMesh &v)
        {
            a(v.profile, v.recastSettings, v.partitioningAlgorithm, v.triangleTagging, v.mesh, v.polygonSourceTriangles,
              v.polygonContributingTriangles, v.regions, v.exits, v.borderLinks, v.contours, v.topology, v.statistics,
              v.warnings);
        }

        /// One field visitor supplies deterministic hashing and private binary storage without copying whole candidates.
        class Archive
        {
          public:
            Archive(gzFile file, bool reading, std::size_t limit = MaximumBytes)
                : file_(file), reading_(reading), limit_(limit)
            {
            }
            explicit Archive(core::ContentHash &hash) : hash_(&hash) {}
            template <class... T> void operator()(T &...values)
            {
                (Value(values), ...);
            }
            bool good{true};

          private:
            void Bytes(void *data, std::size_t count)
            {
                if (!good || count > limit_ - transferred_)
                {
                    good = false;
                    return;
                }
                transferred_ += count;
                if (hash_)
                {
                    hash_->Add(std::span(static_cast<const std::uint8_t *>(data), count));
                }
                else if (reading_)
                {
                    good = gzread(file_, data, static_cast<unsigned int>(count)) == static_cast<int>(count);
                }
                else
                {
                    good = gzwrite(file_, data, static_cast<unsigned int>(count)) == static_cast<int>(count);
                }
            }
            template <class T> void Value(T &value)
            {
                if constexpr (std::is_same_v<T, bool>)
                {
                    std::uint8_t encoded = value ? 1 : 0;
                    Bytes(&encoded, 1);
                    good = good && encoded <= 1;
                    if (reading_ && good)
                    {
                        value = encoded != 0;
                    }
                }
                else if constexpr (std::is_arithmetic_v<T> || std::is_enum_v<T>)
                {
                    Bytes(&value, sizeof(value));
                }
                else
                {
                    Fields(*this, value);
                }
            }
            void Value(std::string &value)
            {
                std::uint64_t count = value.size();
                Value(count);
                if (!good || count > limit_ - transferred_)
                {
                    good = false;
                    return;
                }
                if (reading_)
                {
                    if (count > MaximumBytes - allocated_)
                    {
                        good = false;
                        return;
                    }
                    allocated_ += static_cast<std::size_t>(count);
                    value.resize(static_cast<std::size_t>(count));
                }
                Bytes(value.data(), value.size());
            }
            template <class T, std::size_t N> void Value(std::array<T, N> &value)
            {
                for (auto &item : value)
                {
                    Value(item);
                }
            }
            template <class T> void Value(std::optional<T> &value)
            {
                bool present = value.has_value();
                Value(present);
                if (reading_ && good)
                {
                    value = present ? std::optional<T>(T{}) : std::nullopt;
                }
                if (present && good)
                {
                    Value(*value);
                }
            }
            template <class T> void Value(std::vector<T> &value)
            {
                std::uint64_t count = value.size();
                Value(count);
                if (!good || count > (MaximumBytes - allocated_) / sizeof(T) || count > limit_ - transferred_ ||
                    count > 16777216)
                {
                    good = false;
                    return;
                }
                if (reading_)
                {
                    allocated_ += static_cast<std::size_t>(count) * sizeof(T);
                    value.resize(static_cast<std::size_t>(count));
                }
                for (auto &item : value)
                {
                    Value(item);
                    if (!good)
                    {
                        break;
                    }
                }
            }
            gzFile file_{};
            core::ContentHash *hash_{};
            bool reading_{};
            std::size_t transferred_{}, allocated_{};
            std::size_t limit_{MaximumBytes};
        };

        gzFile Open(const std::filesystem::path &path, const char *mode)
        {
#ifdef _WIN32
            return gzopen_w(path.c_str(), mode);
#else
            return gzopen(path.c_str(), mode);
#endif
        }
        // Writes and hashes never mutate fields. Reading alone changes the supplied object.
        template <class T> T &Writable(const T &value)
        {
            return const_cast<T &>(value);
        }
    } // namespace

    std::string CandidateFingerprint(const core::Scene &scene, const core::NavigationProfile &profile,
                                     std::optional<core::AABB> bounds, const std::vector<core::CandidateExit> &exits,
                                     const std::vector<core::NavMesh> &neighbors, std::string_view partitioning,
                                     const core::RecastSettings &settings, const std::vector<core::NavMesh> &authored,
                                     std::optional<float> waterHeight, bool tagTriangles)
    {
        core::ContentHash hash;
        hash.Add(Schema);
        hash.Add(partitioning);
        Archive archive(hash);
        archive(Writable(scene.mesh.vertices), Writable(scene.mesh.triangles), Writable(scene.geometrySources),
                Writable(scene.triangleProvenance), Writable(profile), Writable(settings), bounds, Writable(exits),
                Writable(neighbors), Writable(authored), waterHeight, tagTriangles);
        return archive.good ? hash.Hex() : std::string{};
    }

    bool StoreCandidate(const std::filesystem::path &path, const core::CandidateNavMesh &candidate,
                        const core::Scene &evidence)
    {
        static std::atomic_size_t sequence{};
        std::filesystem::create_directories(path.parent_path());
        auto temporary = path;
        temporary += "." + std::to_string(++sequence) + ".tmp";
        auto file = Open(temporary, "wb");
        if (!file)
        {
            return false;
        }
        Archive archive(file, false);
        std::string schema(Schema);
        archive(schema, Writable(candidate), Writable(evidence.geometrySources), Writable(evidence.triangleProvenance));
        const auto closed = gzclose(file) == Z_OK;
        std::error_code error;
        if (archive.good && closed)
        {
            std::filesystem::rename(temporary, path, error);
            if (!error)
            {
                return true;
            }
        }
        std::filesystem::remove(temporary, error);
        return false;
    }

    bool LoadCandidate(const std::filesystem::path &path, core::CandidateNavMesh &candidate, core::Scene &evidence)
    {
        std::ifstream compressed(path, std::ios::binary | std::ios::ate);
        if (compressed.tellg() < 18)
        {
            return false;
        }
        std::array<std::uint8_t, 4> encodedSize{};
        compressed.seekg(-4, std::ios::end);
        compressed.read(reinterpret_cast<char *>(encodedSize.data()), 4);
        const std::uint32_t declaredSize = encodedSize[0] | static_cast<std::uint32_t>(encodedSize[1]) << 8 |
                                           static_cast<std::uint32_t>(encodedSize[2]) << 16 |
                                           static_cast<std::uint32_t>(encodedSize[3]) << 24;
        if (!compressed || declaredSize < Schema.size() + 8 || declaredSize > MaximumBytes)
        {
            return false;
        }
        auto file = Open(path, "rb");
        if (!file)
        {
            return false;
        }
        struct FileOwner
        {
            gzFile file;
            ~FileOwner()
            {
                if (file)
                {
                    gzclose(file);
                }
            }
            bool Close()
            {
                return gzclose(std::exchange(file, nullptr)) == Z_OK;
            }
        } owner{file};
        try
        {
            Archive archive(file, true, declaredSize);
            std::string schema;
            core::CandidateNavMesh loaded;
            core::Scene sources;
            archive(schema);
            if (schema != Schema)
            {
                return false;
            }
            archive(loaded, sources.geometrySources, sources.triangleProvenance);
            core::ValidateRecastSettings(loaded.profile, loaded.recastSettings);
            std::uint8_t extra{};
            const bool ended = gzread(file, &extra, 1) == 0 && gzeof(file);
            const bool closed = owner.Close();
            const auto finite = [](const core::Vec3 &point)
            { return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z); };
            if (!archive.good || !ended || !closed || !loaded.topology.valid ||
                !std::isfinite(loaded.profile.weldTolerance) || loaded.profile.weldTolerance <= 0 ||
                !std::isfinite(loaded.profile.stepHeight) || loaded.profile.stepHeight < 0 ||
                !std::all_of(loaded.mesh.vertices.begin(), loaded.mesh.vertices.end(), finite) ||
                !core::ValidateCandidateTopology(loaded).valid ||
                loaded.mesh.polygons.size() != loaded.polygonSourceTriangles.size() ||
                loaded.mesh.polygons.size() != loaded.polygonContributingTriangles.size())
            {
                return false;
            }
            for (const auto &source : sources.triangleProvenance)
            {
                if (source.geometrySource >= sources.geometrySources.size())
                {
                    return false;
                }
            }
            const auto validTriangles = [&](const auto &triangles)
            {
                return std::all_of(triangles.begin(), triangles.end(),
                                   [&](auto index) { return index < sources.triangleProvenance.size(); });
            };
            if (!validTriangles(loaded.polygonSourceTriangles))
            {
                return false;
            }
            for (const auto &triangles : loaded.polygonContributingTriangles)
            {
                if (!validTriangles(triangles))
                {
                    return false;
                }
            }
            const auto validPolygons = [&](const auto &indices)
            {
                return std::all_of(indices.begin(), indices.end(),
                                   [&](auto index) { return index < loaded.mesh.polygons.size(); });
            };
            for (std::size_t index{}; index < loaded.regions.size(); ++index)
            {
                const auto &region = loaded.regions[index];
                if (region.id != index || !validPolygons(region.polygons) || !validTriangles(region.sourceTriangles) ||
                    !std::all_of(region.geometrySources.begin(), region.geometrySources.end(),
                                 [&](auto source) { return source < sources.geometrySources.size(); }))
                {
                    return false;
                }
            }
            for (const auto &contour : loaded.contours)
            {
                if (contour.region >= loaded.regions.size() ||
                    !std::all_of(contour.vertices.begin(), contour.vertices.end(),
                                 [&](auto vertex) { return vertex < loaded.mesh.vertices.size(); }))
                {
                    return false;
                }
            }
            for (const auto &exit : loaded.exits)
            {
                if (!finite(exit.position) || (exit.region && *exit.region >= loaded.regions.size()) ||
                    (exit.polygon && *exit.polygon >= loaded.mesh.polygons.size()))
                {
                    return false;
                }
            }
            for (const auto &link : loaded.borderLinks)
            {
                if (link.polygon >= loaded.mesh.polygons.size() || link.edge >= 3 || link.neighborEdge >= 3)
                {
                    return false;
                }
            }
            std::error_code error;
            std::filesystem::last_write_time(path, std::filesystem::file_time_type::clock::now(), error);
            candidate = std::move(loaded);
            evidence = std::move(sources);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    void ReleaseCandidateAudit(core::CandidateNavMesh &candidate)
    {
        decltype(candidate.polygonSourceTriangles){}.swap(candidate.polygonSourceTriangles);
        decltype(candidate.polygonContributingTriangles){}.swap(candidate.polygonContributingTriangles);
        decltype(candidate.contours){}.swap(candidate.contours);
        for (auto &region : candidate.regions)
        {
            decltype(region.sourceTriangles){}.swap(region.sourceTriangles);
            decltype(region.geometrySources){}.swap(region.geometrySources);
        }
    }
} // namespace navmesh::app::detail
