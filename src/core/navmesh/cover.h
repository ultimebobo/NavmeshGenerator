#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /** Retain physical wall evidence needed to classify final navigation boundaries.
     * @param candidate World-space candidate; replaces its compact coverGeometry evidence.
     * @param scene Generation geometry and provenance. Visual fallback and navigation exclusions supply no cover.
     * The retained geometry is independent of triangle indices changed during later seam refinement.
     */
    void CaptureCoverGeometry(CandidateNavMesh &candidate, const Scene &scene);

    /** Build cover/traversal metadata on final linked geometry without renumbering triangles.
     * @param mesh Finite world-space geometry with valid vertex indices, portal edges and door triangles already marked;
     * updated in place.
     * @param candidate Valid finite agent settings and retained physical wall evidence from generation.
     * @param authored Original selected-cell meshes, used to retain coincident cover and creature restrictions.
     * Only closed boundary edges receive cover. A triangle may be cyclically rotated to encode its
     * covered edges in Skyrim's two cover slots; vertices, neighbors and edge flags rotate together.
     * Unsupported third-edge cover is conservatively limited to the two strongest edges.
     */
    void BuildNavmeshCover(NavMesh &mesh, const CandidateNavMesh &candidate, std::span<const NavMesh> authored);
} // namespace navmesh::core
