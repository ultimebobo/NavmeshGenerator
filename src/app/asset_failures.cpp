#include "app/asset_failures.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <cctype>
#include <format>

namespace navmesh::app::detail
{
    namespace
    {
        std::string Normalize(std::string value)
        {
            std::replace(value.begin(), value.end(), '\\', '/');
            std::transform(value.begin(), value.end(), value.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return value;
        }
    } // namespace

    std::size_t AssetFailureCatalog::Add(const skyrim::GeometryFailure &failure, std::string_view stage,
                                         std::uint32_t cell)
    {
        auto model = Normalize(failure.model);
        if (!model.empty() && !model.starts_with("meshes/"))
        {
            model.insert(0, "meshes/");
        }
        const auto key = std::tuple{model, Normalize(failure.provider), failure.category};
        const auto [position, inserted] = identities_.try_emplace(key, entries_.size() + 1);
        if (inserted)
        {
            entries_.push_back({.failure = failure});
        }
        auto &entry = entries_.at(position->second - 1);
        entry.stages.emplace(stage);
        if (failure.reference.formId)
        {
            entry.references.try_emplace(std::tuple{failure.reference.plugin, failure.reference.formId,
                                                    failure.baseObject.plugin, failure.baseObject.formId},
                                         failure);
        }
        if (cell)
        {
            entry.cells.insert(cell);
        }
        return position->second;
    }

    const skyrim::GeometryFailure &AssetFailureCatalog::Failure(std::size_t id) const
    {
        return entries_.at(id - 1).failure;
    }

    void AssetFailureCatalog::WriteJson(std::ostream &output) const
    {
        using reproducibility::EscapeJson;
        output << '[';
        for (std::size_t i{}; i < entries_.size(); ++i)
        {
            const auto &entry = entries_[i];
            output << std::format("{}{{\"id\":{},\"category\":\"{}\",\"model\":\"{}\",\"provider\":\"{}\","
                                  "\"reason\":\"{}\",\"stages\":[",
                                  i ? "," : "", i + 1, EscapeJson(entry.failure.category),
                                  EscapeJson(entry.failure.model), EscapeJson(entry.failure.provider),
                                  EscapeJson(entry.failure.reason));
            bool first = true;
            for (const auto &stage : entry.stages)
            {
                output << (first ? "" : ",") << '"' << EscapeJson(stage) << '"';
                first = false;
            }
            output << "],\"references\":[";
            first = true;
            for (const auto &[identity, source] : entry.references)
            {
                output << std::format("{}{{\"form_id\":\"{:08X}\",\"plugin\":\"{}\",\"base_form_id\":\"{:08X}\","
                                      "\"base_plugin\":\"{}\"}}",
                                      first ? "" : ",", source.reference.formId, EscapeJson(source.reference.plugin),
                                      source.baseObject.formId, EscapeJson(source.baseObject.plugin));
                first = false;
            }
            output << "],\"cells\":[";
            first = true;
            for (const auto cell : entry.cells)
            {
                output << std::format("{}\"{:08X}\"", first ? "" : ",", cell);
                first = false;
            }
            output << "]}";
        }
        output << ']';
    }
} // namespace navmesh::app::detail
