#pragma once

#include "core/geometry/types.h"
#include "core/navmesh/types.h"

#include <optional>
#include <string>

namespace navmesh::core
{
    /** Continuous placement uncertainty, conservatively represented by a solid collision envelope.
     * Positions are absolute Skyrim world units. Scale is a nonnegative maximum
     * factor; arbitraryRotation requires a model-local bounding sphere about the origin.
     * The envelope supplies obstruction only, never a walkable floor.
     */
    struct PlacementEnvelope
    {
        /// Inclusive component bounds on absolute reference positions, in world units.
        Vec3 minimumPosition, maximumPosition;
        /// Largest possible nonnegative model-to-world scale factor.
        float maximumScale{1.0F};
        /// Enclose all orientations about the model origin rather than the reference's fixed angles.
        bool arbitraryRotation{};
    };

    struct Reference
    {
        std::uint32_t id{};
        std::uint32_t baseObjectId{};
        std::string name;
        std::string recordType;
        std::string editorId;
        std::string modelPath;
        Vec3 position;
        Vec3 rotation;
        float scale{1.0F};
        /// Winning record origins from resolved load order assembly.
        std::string sourcePlugin;
        std::string basePlugin;
        std::string baseRecordType;
        /// True when the winning placed-record header marks this reference initially disabled.
        bool initiallyDisabled{};
        /// True when the winning placed-record header marks this reference deleted.
        bool deleted{};
        /// Winning placement has a teleport destination (XTEL), including non-physical cave exits.
        bool teleportExit{};
        /// Reciprocal XTEL arrival position in this reference's Skyrim world space; absent without a valid return link.
        /// Exit matching uses this standing position to select the accessible side of a teleport threshold.
        std::optional<Vec3> teleportArrival;
        /// Base-object OBND in model-local Skyrim units; absent when missing or degenerate.
        std::optional<AABB> localBounds;
        /// Runtime-input file and line provenance; empty for plugin-only placements.
        std::string inputConfig;
        /// Possible collision occupancy; absence means the placement is deterministic.
        std::optional<PlacementEnvelope> envelope;
    };
    struct Cell
    {
        std::uint32_t id{};
        std::string editorId;
        std::string name;
        bool isInterior{};
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates;
        std::vector<Reference> references;
        std::vector<NavMesh> navMeshes;
        /// Effective exterior water surface Z in Skyrim world units, when supported and finite.
        /// Absent for dry cells, interiors, or unresolved/default water data.
        std::optional<float> waterHeight;
    };
} // namespace navmesh::core
