#pragma once

#include "core/navmesh/candidate.h"
#include "skyrim/parser/plugin_parser.h"

#include <filesystem>
#include <string>
#include <vector>

namespace navmesh::skyrim
{
    /// One cell and its generated Skyrim world-space NAVM, borrowed for a batch write.
    struct NavmeshReplacement
    {
        const core::Cell *cell{};
        const core::CandidateNavMesh *candidate{};
    };
    /** Serialize generated navigation into a verified patch or source copy, allocating NAVMs for uncovered cells.
     * @param outputDirectory Writable output folder; existing plugins are refused.
     * Patch names receive a numeric suffix when their default filename is already
     * an active input, preventing a self dependency on an earlier generated plugin.
     * @param inputPlugins Active physical plugin paths in load order.
     * @param resolved Winning records from the same input snapshot.
     * @param replacements Unique cells with valid candidates, including empty meshes
     * when no walkable floor survives. Generated border links identify destination CELLs
     * and generated triangles; all primary NAVM identities are allocated before writing.
     * Each generated portal must have exact reversed endpoints and a unique return link.
     * Authored crossings require a generated replacement or explicit excluded-obstacle evidence.
     * Matched authored portal endpoints may extend beyond nominal CELL bounds by
     * core::AuthoredBorderTolerance, or the profile's border halo for explicit authored crossings;
     * other candidate vertices must stay inside.
     * Cells without any winning NAVM receive a plugin-owned identity and temporary
     * CELL child placement with a serialized PathingCell type tag for Creation Kit loading;
     * unsupported existing records cannot be treated as uncovered.
     * Every live NAVM in a selected CELL must have supported geometry. Unknown live NAVM
     * layouts in files able to reference replaced identities reject export because incoming
     * triangle dependencies cannot be ruled out, even without a matched border.
     * Authored incoming links to replaced geometry are removed from all source NAVMs;
     * matched borders receive fresh reciprocal links. Unrelated portals are retained.
     * Bounded authored neighbor vertex edits are checked against the input snapshot,
     * combined across replacements, and applied without renumbering triangles or vertices.
     * Conflicting edits, altered unrelated portals and unsupported spatial grid tails fail export.
     * Edited neighbors receive a rebuilt grid while retaining authored cover and door indices.
     * NAVI entries are rebuilt from every emitted NAVM, including reciprocal neighbors and
     * empty secondary meshes. Source copies retain unrelated navigation-map entries.
     * Cover is built from retained physical collision and coincident authored edges. Connector
     * constraints and base-object arrays are remapped to the primary generated record.
     * Winning door references receive reciprocal XNDP associations while preserving their
     * other fields, scripts, locks, flags and group placement. Patch exports place partial WRLD/CELL
     * parent declarations immediately before persistent door groups to establish their loading context.
     * Changed input snapshots fail export.
     * @param writtenPath Receives the finalized plugin path on success.
     * @param error Receives a failure reason; no final plugin is published on failure.
     * @param copyPlugin Optional active filename to copy with generated NAVMs. Preserves
     * unrelated records, the filename, TES4 flags, and existing master indices. Generated
     * references must belong to that plugin or its existing masters; new IDs must fit
     * its existing full/light format. The copy replaces the source when installed.
     * @return True after read-back and validation against the effective input navigation graph.
     * @warning Unrepresentable connector constraints, localized reference resources, unknown
     * reference fields and traversal portals without a safe replacement route reject export instead of being discarded.
     */
    [[nodiscard]] bool WriteNavmeshOverrides(const std::filesystem::path &outputDirectory,
                                             const std::vector<std::filesystem::path> &inputPlugins,
                                             const ResolvedLoadOrder &resolved,
                                             const std::vector<NavmeshReplacement> &replacements,
                                             std::filesystem::path &writtenPath, std::string &error,
                                             const std::string &copyPlugin = {});
    /** Write generated navigation as overrides, or a new NAVM when the selected cell has none.
     * @param outputDirectory Directory for a new ESP; existing files are never replaced.
     * @param inputPlugins Physical active plugin paths in resolved load order.
     * @param resolved Winning records from that same load order.
     * @param cell Selected cell in Skyrim world coordinates.
     * @param candidate Valid generated mesh in Skyrim world coordinates.
     * Only matched authored portal endpoints may exceed nominal exterior CELL bounds,
     * within the authored matching tolerance or bounded halo for required crossings. Neighbor geometry retains its identities;
     * bounded border fan edits recorded in the candidate are verified and serialized.
     * Incoming portals into replaced triangle spaces are cleared before matched
     * reciprocal portals are written; unmatched edges remain open boundaries.
     * @param writtenPath Receives the final plugin path after read-back verification.
     * @param error Human-readable reason when the source cannot be safely serialized.
     * @return True only after a successful write and independent reader round trip;
     * false with an error when a required border portal cannot be serialized.
     * @note The ESP is ESL-flagged when its master table and new identities fit a light plugin.
     * @note NAVI, cover, matched portals and reciprocal REFR XNDP associations are rebuilt automatically.
     * Unsupported connection or reference remapping fails without publishing an output plugin.
     */
    [[nodiscard]] bool WriteNavmeshOverride(const std::filesystem::path &outputDirectory,
                                            const std::vector<std::filesystem::path> &inputPlugins,
                                            const ResolvedLoadOrder &resolved, const core::Cell &cell,
                                            const core::CandidateNavMesh &candidate, std::filesystem::path &writtenPath,
                                            std::string &error);
} // namespace navmesh::skyrim
