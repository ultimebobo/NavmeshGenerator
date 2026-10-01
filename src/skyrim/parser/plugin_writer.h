#pragma once

#include "core/navmesh/candidate.h"
#include "skyrim/parser/plugin_parser.h"

#include <filesystem>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
    /// One cell and its generated Skyrim world-space NAVM, borrowed for a batch write.
    struct NavmeshReplacement { const core::Cell* cell{}; const core::CandidateNavMesh* candidate{}; };
    /** Serialize all replacements into one verified override ESP.
     * @param outputDirectory Writable output folder; existing plugins are refused.
     * @param inputPlugins Active physical plugin paths in load order.
     * @param resolved Winning records from the same input snapshot.
     * @param replacements Unique cells with nonempty, valid candidates. Border links
     * between rebuilt cells must target their generated primary NAVM triangles.
     * @param writtenPath Receives the finalized ESP path on success.
     * @param error Receives a failure reason; no final ESP is published on failure.
     * @return True after every NAVM, door, and reciprocal border passes read-back.
     * @warning NAVI, XNDP, cover and unmatched authored links are not rebuilt.
     */
    [[nodiscard]] bool WriteNavmeshOverrides(const std::filesystem::path& outputDirectory,
        const std::vector<std::filesystem::path>& inputPlugins, const ResolvedLoadOrder& resolved,
        const std::vector<NavmeshReplacement>& replacements,
        std::filesystem::path& writtenPath, std::string& error);
    /** Write generated navigation as overrides of the selected cell's NAVMs.
     * @param outputDirectory Directory for a new ESP; existing files are never replaced.
     * @param inputPlugins Physical active plugin paths in resolved load order.
     * @param resolved Winning records from that same load order.
     * @param cell Selected cell in Skyrim world coordinates.
     * @param candidate Valid generated mesh in Skyrim world coordinates.
     * @param writtenPath Receives the final plugin path after read-back verification.
     * @param error Human-readable reason when the source cannot be safely serialized.
     * @return True only after a successful write and independent reader round trip;
     * false with an error when a required border portal cannot be serialized.
     * @note The ESP is ESL-flagged when its override-only record set fits a light plugin.
     * @warning Matched door triangles and reciprocal border portals are serialized.
     * Other authored links, cover data, NAVI, and REFR XNDP references are not rebuilt.
     */
    [[nodiscard]] bool WriteNavmeshOverride(const std::filesystem::path& outputDirectory,
        const std::vector<std::filesystem::path>& inputPlugins, const ResolvedLoadOrder& resolved,
        const core::Cell& cell, const core::CandidateNavMesh& candidate,
        std::filesystem::path& writtenPath, std::string& error);
}
