#pragma once

#include "app/options.h"
#include "skyrim/extraction/geometry_extractor.h"

namespace navmesh::app::detail
{
    /// Completed target, including compact writer inputs and an optional pinned inspection audit.
    struct BatchCellResult
    {
        /// Borrowed target metadata; the resolved load order must outlive all tasks and final writing.
        const core::Cell *cell{};
        /// Writer geometry in Skyrim world coordinates; audit-only arrays are released after spooling.
        core::CandidateNavMesh candidate;
        /// Public export metadata, report status, and captured failure message (empty on success).
        std::string metadata, status, error;
        /// Private gzip candidate/evidence path, optionally pinned until inspection export completes.
        std::filesystem::path auditPath;
        /// Steady-clock stage times in seconds; reuse performs no Recast generation.
        double extractionSeconds{}, generationSeconds{};
        /// Supplier CELL, retained placement, and unique logical model counts before extraction policy exclusions.
        std::size_t supplierCells{}, references{}, models{};
        /// True when the complete candidate and audit evidence were loaded from a compatible cache entry.
        bool reused{};
    };

    /// Isolated worker inputs; geometry is owned by one task, while cell metadata remains in the resolved snapshot.
    struct BatchGenerationInput
    {
        BatchCellResult result;
        skyrim::GeometryExtraction geometry;
        /// Optional target exterior bounds in Skyrim world coordinates; interior extent comes from support geometry.
        std::optional<core::AABB> bounds;
        /// Enabled target doors in Skyrim world coordinates.
        std::vector<core::CandidateExit> exits;
        /// Authored neighboring NAVMs used for border stitching and dependency invalidation.
        std::vector<core::NavMesh> adjacent;
        /// Shared asset snapshot for candidate retention and run-owned directory for pinned inspection audits.
        std::filesystem::path cacheDirectory, stagingDirectory;
    };

    /// Generate/reuse, validate, compact and spool one candidate; catches worker failures in result.error.
    [[nodiscard]] BatchCellResult BuildBatchCandidate(BatchGenerationInput input, const Options &options);

    /// Estimate scene and raster working storage for admission, in bytes; one oversized task must run alone.
    /// This is an admission estimate, not a limit on Recast vertical spans or resolved-plugin memory.
    [[nodiscard]] std::size_t EstimateGenerationBytes(const core::Scene &scene, std::optional<core::AABB> bounds);
} // namespace navmesh::app::detail
