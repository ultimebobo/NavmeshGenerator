#pragma once

#include "skyrim/extraction/geometry_extractor.h"

#include <map>
#include <ostream>
#include <set>
#include <tuple>

namespace navmesh::app::detail
{
    /// Run-owned deduplicated asset diagnostics, shared by selection and target extraction.
    class AssetFailureCatalog
    {
      public:
        /** Retain an unavailable asset and associate its reference, processing stage and affected CELL.
         * @param failure Asset-level evidence; normalized model/provider/category identify one entry.
         * @param stage Selection or extraction stage name.
         * @param cell Resolved target CELL identity; zero records an issue with unresolved ownership.
         * @return Stable one-based issue ID for per-cell reports. No input files are modified.
         */
        std::size_t Add(const skyrim::GeometryFailure &failure, std::string_view stage, std::uint32_t cell);
        /// Return the retained asset-level diagnostic for a valid one-based issue ID; throws on an invalid ID.
        [[nodiscard]] const skyrim::GeometryFailure &Failure(std::size_t id) const;
        /// Append a JSON array of unique assets, sources, stages and affected CELL identities; caller checks stream errors.
        void WriteJson(std::ostream &output) const;

      private:
        struct Entry
        {
            skyrim::GeometryFailure failure;
            std::set<std::string> stages;
            std::map<std::tuple<std::string, std::uint32_t, std::string, std::uint32_t>, skyrim::GeometryFailure>
                references;
            std::set<std::uint32_t> cells;
        };
        std::map<std::tuple<std::string, std::string, std::string>, std::size_t> identities_;
        std::vector<Entry> entries_;
    };
} // namespace navmesh::app::detail
