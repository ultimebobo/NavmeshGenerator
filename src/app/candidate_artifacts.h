#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::app::detail
{
    /// Stream candidate JSON into gzip without allocating an uncompressed file; false on serialization/write failure.
    [[nodiscard]] bool WriteCompressedCandidateJson(const std::filesystem::path &path,
                                                    const core::CandidateNavMesh &candidate, const core::Scene &scene,
                                                    const std::string &metadata);
} // namespace navmesh::app::detail
