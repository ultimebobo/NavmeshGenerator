#pragma once

#include "core/navmesh/candidate.h"
#include "skyrim/parser/plugin_parser.h"

#include <filesystem>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
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
