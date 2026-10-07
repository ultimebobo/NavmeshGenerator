#pragma once

#include "app/options.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::app::detail
{
    /// Completed target, including compact writer inputs and an optional pinned inspection audit.
    struct BatchCellResult
    {
        /// Borrowed target metadata; the resolved load order must outlive all tasks and final writing.
        const core::Cell *cell{};
        /// Generated geometry and compact source joins retained through batch seam refinement.
        core::CandidateNavMesh candidate;
        /// Public export metadata, report status, and captured failure message (empty on success).
        std::string metadata, status, error;
        /// Topology findings captured when candidate generation fails.
        std::vector<std::string> diagnostics;
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
        /// Authored NAVMs from untouched neighbors only; regenerated cells are linked after all tasks finish.
        std::vector<core::NavMesh> adjacent;
        /// Shared asset snapshot for candidate retention and run-owned directory for pinned inspection audits.
        std::filesystem::path cacheDirectory, stagingDirectory;
    };

    /** Generate or reuse one complete walkable candidate for subsequent batch linking.
     * @param input Isolated world-space geometry, untouched-neighbor constraints and borrowed target metadata.
     * @param options Validated shared batch settings, including the normalized artifact policy.
     * @return Candidate with compact evidence and status generated, including valid empty floor coverage.
     * No component is removed for lacking an authored portal or a door. Worker failures,
     * including invalid topology, are captured in result.error with status failed.
     */
    [[nodiscard]] BatchCellResult BuildBatchCandidate(BatchGenerationInput input, const Options &options);

    /** Join the complete generated exterior set without consulting replaced authored geometry.
     * @param results Completed targets with full generated geometry and compact source joins.
     * Candidates remain present when an adjacent target has no walkable floor.
     * @param resolved Winning CELL worldspace and coordinate ownership.
     * @return Empty on success, or a fatal refinement/ownership error. Generated links
     * identify destination CELLs until the writer allocates every primary NAVM identity.
     * Refinement preserves existing portals into untouched cells and adds exact reciprocal
     * generated seams; failures prevent publishing the combined plugin.
     */
    [[nodiscard]] std::string ReconcileBatchBorders(std::vector<BatchCellResult> &results,
                                                    const skyrim::ResolvedLoadOrder &resolved);

    /// Estimate scene and raster working storage for admission, in bytes; one oversized task must run alone.
    /// This is an admission estimate, not a limit on Recast vertical spans or resolved-plugin memory.
    [[nodiscard]] std::size_t EstimateGenerationBytes(const core::Scene &scene, std::optional<core::AABB> bounds);
} // namespace navmesh::app::detail
