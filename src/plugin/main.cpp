#include "cli/json_report.h"
#include "skyrim/extraction/cell_extractor.h"
#include "validation/validation.h"

namespace
{
    void WriteCurrentCellReport()
    {
        const auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* cell = player ? player->GetParentCell() : nullptr;
        if (!cell) { REX::WARN("NavmeshGenerator: no current player cell is available."); return; }
        const auto extracted = navmesh::skyrim::ExtractCell(*cell);
        const auto report = navmesh::cli::ToJson(extracted, navmesh::validation::Validate(extracted));
        const auto reportPath = std::filesystem::path("Data/SKSE/Plugins/NavmeshGenerator/report.json");
        std::filesystem::create_directories(reportPath.parent_path());
        std::ofstream output(reportPath, std::ios::binary | std::ios::trunc);
        output << report;
        REX::INFO("NavmeshGenerator: wrote diagnostic for cell {:08X} to {}", extracted.id, reportPath.string());
    }
    void OnSKSEMessage(SKSE::MessagingInterface::Message* message)
    {
        if (message && message->type == SKSE::MessagingInterface::kPostLoadGame) WriteCurrentCellReport();
    }
}

SKSE_PLUGIN_LOAD(const SKSE::LoadInterface* skse)
{
    SKSE::Init(skse);
    SKSE::GetMessagingInterface()->RegisterListener(OnSKSEMessage);
    REX::INFO("NavmeshGenerator read-only diagnostic loaded.");
    return true;
}
