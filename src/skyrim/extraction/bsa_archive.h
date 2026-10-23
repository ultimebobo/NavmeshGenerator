#pragma once

#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace navmesh::skyrim
{
    /// One unreadable archive index or winning model payload, with the actual provider and logical requested model.
    struct BsaFailure
    {
        std::filesystem::path archive;
        std::string model, reason;
    };
    /** Run-scoped selective BSA extraction into an owned model-cache snapshot.
     * Directory indexes are loaded lazily and retained until destruction. Only winning
     * requested NIF payloads are read. Archives must remain unchanged during the run.
     * Calls on one instance must be serialized by the caller.
     */
    class BsaModelExtractor
    {
      public:
        /** Bind ordered providers to a writable cache snapshot.
         * @param archives Physical BSA paths in increasing provider priority.
         * @param snapshot Owned cache directory; existing extracted/missing models are reused.
         * Throws on cache-directory preparation failure.
         * @warning Creates the snapshot directory; never modifies input archives.
         */
        BsaModelExtractor(std::vector<std::filesystem::path> archives, std::filesystem::path snapshot);
        /// Release retained directory indexes.
        ~BsaModelExtractor();

        /** Extract uncached winning models, publishing complete files by rename.
         * @param requested UTF-8 logical meshes paths, ASCII case insensitive, with either slash convention.
         * @return False on an unsafe logical path, index or payload-decoding failure; successful
         * files remain usable. An unreadable winning entry never uses a lower provider.
         * Missing names are persisted only after a reliable search of every provider.
         * Updates the snapshot's cumulative archive statistics after each nonempty request.
         * Allocation and cache/storage failures throw and must stop publishing.
         * Cache publication errors identify the physical path and operating-system reason.
         */
        [[nodiscard]] bool Extract(const std::set<std::string> &requested);
        /// Asset failures from the latest Extract call; borrowed until the next extraction or destruction.
        [[nodiscard]] const std::vector<BsaFailure> &Failures() const;

        /** Identify names whose winning archive belongs to a changed provider set.
         * @param changed Physical archive paths; path comparisons ignore case on Windows.
         * @param models Replaced with normalized forward-slash logical paths on full success.
         * @return False on an index failure, leaving models unchanged. No payloads are read.
         * Allocation and cache/storage failures throw.
         * Updates cumulative archive statistics; the caller excludes loose winners.
         */
        [[nodiscard]] bool ChangedModels(const std::set<std::filesystem::path> &changed, std::set<std::string> &models);

      private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace navmesh::skyrim
