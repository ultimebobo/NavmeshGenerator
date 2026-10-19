#pragma once

#include "skyrim/parser/plugin_parser.h"

namespace navmesh::skyrim
{
    /// Winning loose BOS input; UTF-8 contents are retained so resolution uses one coherent snapshot.
    struct BaseObjectSwapConfig
    {
        /// Top-level effective Data filename, preserving BOS's case-sensitive ordering.
        std::string name;
        /// Physical winning provider file; read-only during a run.
        std::filesystem::path path;
        /// Complete retained text, including comments and original line numbers.
        std::string contents;
    };

    /// Immutable effective Data configuration snapshot, ordered by BOS filename priority.
    struct BaseObjectSwapInput
    {
        /// Data roots in increasing provider priority, including MO2 overwrite when applicable.
        std::vector<std::filesystem::path> roots;
        /// Same-name winners in BOS configuration order.
        std::vector<BaseObjectSwapConfig> configs;
        /// Content and winner identity hash; independent of timestamps.
        std::string fingerprint;
    };

    /** Discover top-level _SWAP.ini files without modifying game or manager inputs.
     * @param roots Effective Data providers in increasing priority; later same-name files win.
     * @return Filename-ordered configs and content identity. Missing roots contribute no files.
     * @throws std::runtime_error for unreadable configurations or directory enumeration failures.
     */
    [[nodiscard]] BaseObjectSwapInput DiscoverBaseObjectSwaps(const std::vector<std::filesystem::path> &roots);

    /** Apply BOS replacement and property precedence after resolving real plugin winners.
     * @param resolved Full/light plugin identities and winning cells; modified in place.
     * @param input Immutable effective configuration snapshot.
     * @return Human-readable diagnostics, including conservative random/unknown-condition handling.
     * Exact rules preserve ordinary support geometry. Uncertain outcomes retain every possible
     * collider as obstruction envelopes and never introduce speculative walkable surfaces.
     * Runtime changes remain separate from encoded plugin records and their override history.
     * INI enumeration follows BOS's first-spelling sections and unique case-insensitive keys.
     * Sectionless and custom unconditional sections use reference swaps; conditional non-Forms sections use properties.
     * Unknown runtime state and failing replacement lookups retain conservative alternatives.
     * @throws std::runtime_error for malformed or unrepresentable navigation inputs, with source path and line.
     */
    [[nodiscard]] std::vector<std::string> ApplyBaseObjectSwaps(ResolvedLoadOrder &resolved,
                                                                const BaseObjectSwapInput &input);

    /// Rediscover winner membership and contents; false means the run's BOS snapshot is stale.
    /// Propagates filesystem/read failures rather than treating them as a coherent snapshot.
    [[nodiscard]] bool BaseObjectSwapSnapshotMatches(const BaseObjectSwapInput &input);

    /** Write BOS input identity, winning paths and effective placement evidence as JSON.
     * @return False on output failure. Does not write to the original configuration files.
     */
    [[nodiscard]] bool WriteBaseObjectSwapReport(const std::filesystem::path &path, const BaseObjectSwapInput &input,
                                                 const ResolvedLoadOrder &resolved,
                                                 const std::vector<std::string> &diagnostics);
} // namespace navmesh::skyrim
