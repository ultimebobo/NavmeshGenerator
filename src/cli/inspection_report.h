#pragma once

#include "analysis/navmesh_analysis.h"
#include "core/reproducibility/export_metadata.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::cli
{
    /// Overwrite an OBJ in Skyrim world coordinates, skipping invalid polygons; return silently if opening fails.
    void WriteObj(const std::filesystem::path &outputPath, const navmesh::core::NavMesh &mesh, const std::string &name);

    /// Overwrite a world-space OBJ of NAVM and selected support triangles; return silently if opening fails.
    void WriteAnalysisObj(const std::filesystem::path &outputPath, const navmesh::core::NavMesh &mesh,
                          const navmesh::core::Mesh &geometry, const navmesh::analysis::AnalysisReport &analysisReport);

    /// Overwrite support, topology, and repair evidence as JSON; return silently if opening fails.
    void WriteAnalysisJson(const std::filesystem::path &outputPath,
                           const navmesh::analysis::AnalysisReport &analysisReport,
                           const navmesh::analysis::GeometrySummary &geometrySummary,
                           const navmesh::analysis::NavMeshSummary &meshSummary,
                           const navmesh::reproducibility::ExportMetadata &metadata);

    /// Overwrite an HTML support report with world-space projections; return silently if opening fails.
    void WriteDiagnosticHtml(const std::filesystem::path &outputPath, const navmesh::core::Cell &cell,
                             const navmesh::core::NavMesh &mesh, const navmesh::core::Mesh &geometry,
                             const navmesh::skyrim::GeometryExtraction &extraction,
                             const navmesh::analysis::AnalysisReport &report);

    /// Print support classifications and evidence statistics to standard output.
    void PrintAnalysisSummary(const navmesh::analysis::AnalysisReport &analysis);

    /// Overwrite winning-record and origin-chain JSON; return silently if opening fails.
    void WriteLoadOrderJson(const std::filesystem::path &path, const navmesh::skyrim::ResolvedLoadOrder &loadOrder);

    /// Overwrite cell-list JSON, preserving input order; return silently if opening fails.
    void WriteCellsJson(const std::filesystem::path &path, const std::vector<navmesh::core::Cell> &cells);
} // namespace navmesh::cli
