#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/extraction/geometry_extractor.h"
#include "core/navmesh/generator.h"

#include <NifFile.hpp>
#include <Animation.hpp>
#include <bhk.hpp>

#include <cassert>
#include <filesystem>

namespace
{
    void AddBox(nifly::NifFile &nif, nifly::NiNode *target, nifly::Vector3 dimensions)
    {
        constexpr float havokUnits = 69.99125F;
        auto shape = std::make_unique<nifly::bhkBoxShape>();
        shape->dimensions = dimensions / havokUnits;
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
        body->collisionResponse = nifly::RESPONSE_SIMPLE_CONTACT;
        auto collision = std::make_unique<nifly::bhkCollisionObject>();
        collision->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        collision->targetRef.index = nif.GetBlockID(target);
        target->collisionRef.index = nif.GetHeader().AddBlock(std::move(collision));
    }

    std::uint32_t AddMotion(nifly::NifFile &nif, bool constant = false, int rotationKind = 0)
    {
        auto data = std::make_unique<nifly::NiTransformData>();
        data->translations.SetInterpolationType(nifly::LINEAR_KEY);
        data->translations.AddKey({.time = 0, .value = {0, 0, 100}});
        data->translations.AddKey({.time = 1, .value = {0, 0, constant || rotationKind != 0 ? 100.0F : 400.0F}});
        data->translations.AddKey({.time = 2, .value = {0, 0, 100}});
        if (rotationKind == 1)
        {
            data->rotationType = nifly::XYZ_ROTATION_KEY;
            data->zRotations.SetInterpolationType(nifly::LINEAR_KEY);
            data->zRotations.AddKey({.time = 0, .value = 0});
            data->zRotations.AddKey({.time = 1, .value = 1.5707963F});
            data->zRotations.AddKey({.time = 2, .value = 0});
        }
        else if (rotationKind == 2 || constant)
        {
            data->rotationType = nifly::LINEAR_KEY;
            data->quaternionKeys = {
                {.time = 0, .value = {}},
                {.time = 1,
                 .value = constant ? nifly::Quaternion{-1, 0, 0, 0} : nifly::Quaternion{0.7071068F, 0, 0, 0.7071068F}},
                {.time = 2, .value = {}}};
        }
        auto interpolator = std::make_unique<nifly::NiTransformInterpolator>();
        interpolator->dataRef.index = nif.GetHeader().AddBlock(std::move(data));
        return nif.GetHeader().AddBlock(std::move(interpolator));
    }

    void WritePassage(const std::filesystem::path &path, bool behaviorGraph, bool fixedGeometry = true,
                      bool closeSequence = true, int rotationKind = 0)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        nifly::MatTransform transform;
        transform.Clear();
        auto *frame = nif.AddNode("FixedFloor", transform, nif.GetRootNode());
        if (fixedGeometry)
        {
            // A fixed floor joins both sides of the moving panel. A constant controller on it must not
            // remove this support just because it belongs to the same behavior graph as the panel.
            frame->transform.translation = {0, 0, -10};
            AddBox(nif, frame, {512, 128, 10});
        }
        transform.translation = {0, 0, 100};
        auto *panel = nif.AddNode("MovingPanel", transform, nif.GetRootNode());
        transform.Clear();
        auto *panelCollision = nif.AddNode("PanelCollisionChild", transform, panel);
        AddBox(nif, panelCollision, {10, 128, 100});
        const std::vector<nifly::Vector3> vertices{{0, -128, 0}, {0, 128, 0}, {0, 128, 200}};
        const std::vector<nifly::Triangle> triangles{{0, 1, 2}};
        auto *render = nif.CreateShapeFromData("PanelDisplay", &vertices, &triangles, nullptr);
        assert(render);
        nif.SetParentNode(render, panel);

        const auto motion = AddMotion(nif, false, rotationKind);
        const auto constant = AddMotion(nif, true);
        if (behaviorGraph)
        {
            auto graph = std::make_unique<nifly::BSBehaviorGraphExtraData>();
            graph->behaviorGraphFile.get() = "synthetic/TwoState.hkx";
            nif.GetRootNode()->extraDataRefs.AddBlockRef(nif.GetHeader().AddBlock(std::move(graph)));
            for (const auto &[target, interpolator] : {std::pair{panel, motion}, std::pair{frame, constant}})
            {
                auto controller = std::make_unique<nifly::NiTransformController>();
                controller->targetRef.index = nif.GetBlockID(target);
                controller->interpolatorRef.index = interpolator;
                controller->startTime = 0;
                controller->stopTime = 2;
                target->controllerRef.index = nif.GetHeader().AddBlock(std::move(controller));
            }
        }
        else
        {
            auto manager = std::make_unique<nifly::NiControllerManager>();
            manager->targetRef.index = nif.GetBlockID(nif.GetRootNode());
            for (const auto *state : {" Open ", "cLoSe"})
            {
                if (!closeSequence && std::string_view(state) == "cLoSe")
                {
                    continue;
                }
                auto sequence = std::make_unique<nifly::NiControllerSequence>();
                sequence->name.get() = state;
                sequence->controlledBlocks.resize(2);
                sequence->controlledBlocks[0].nodeName.get() = "MovingPanel";
                sequence->controlledBlocks[0].interpolatorRef.index = motion;
                sequence->controlledBlocks[1].nodeName.get() = "FixedFloor";
                sequence->controlledBlocks[1].interpolatorRef.index = constant;
                manager->controllerSequenceRefs.AddBlockRef(nif.GetHeader().AddBlock(std::move(sequence)));
            }
            nif.GetRootNode()->controllerRef.index = nif.GetHeader().AddBlock(std::move(manager));
        }
        std::filesystem::create_directories(path.parent_path());
        assert(nif.Save(path) == 0);
    }
} // namespace

void TestOperablePassages()
{
    using namespace navmesh;
    const auto root = std::filesystem::temp_directory_path() / "navmesh-operable-passage-test";
    std::filesystem::create_directories(root);
    for (const int layout : {0, 1, 2})
    {
        const bool behaviorGraph = layout == 1;
        const auto logical = behaviorGraph ? "Architecture/Animated/StatefulGate.nif"
                             : layout == 2 ? "Architecture/Animated/StatefulPassage.nif"
                                           : "StatefulPassage.nif";
        WritePassage(root / "meshes" / logical, behaviorGraph, true, true, layout);
        core::Cell cell{.isInterior = true};
        cell.references.push_back({.id = 1, .recordType = "REFR", .modelPath = logical, .baseRecordType = "ACTI"});
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 1 && geometry.collisionModelsLoaded == 1);
            assert(geometry.scene.mesh.triangles.size() == 12);
            assert(geometry.scene.renderFallbackMesh.triangles.empty());
            assert(geometry.scene.coverage.front().detail.contains("operable panels excluded"));
            for (const auto &vertex : geometry.scene.mesh.vertices)
            {
                assert(vertex.z <= 0);
            }
            const auto candidate = core::RecastCandidateGenerator{}.Generate(
                geometry.scene, {}, std::nullopt, {}, core::RegionPartitioningAlgorithm::Watershed, {},
                core::CandidateRetention::AllWalkable);
            assert(candidate.topology.valid && candidate.regions.size() == 1);
            core::AABB candidateBounds;
            for (const auto &vertex : candidate.mesh.vertices)
            {
                candidateBounds.Expand(vertex);
            }
            assert(candidateBounds.min.x < -256 && candidateBounds.max.x > 256);
            const auto repeated = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(repeated.scene.mesh.triangles.size() == geometry.scene.mesh.triangles.size());
        }

        if (layout == 0)
        {
            // Sharing a NIF and model cache cannot transfer the passage policy to a container or static.
            for (const auto *baseType : {"CONT", "STAT", "FURN"})
            {
                cell.references.front().baseRecordType = baseType;
                const auto fixed = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache);
                assert(fixed.scene.mesh.triangles.size() == 24);
                assert(fixed.scene.renderFallbackMesh.triangles.size() == 1);
            }
        }
        cell.references.front().baseRecordType = "ACTI";
        WritePassage(root / "meshes" / logical, behaviorGraph, false, true, layout);
        const auto panelsOnly = skyrim::ExtractGeometry(root, cell, root);
        assert(panelsOnly.modelsExcluded == 1 && panelsOnly.modelsUnsupported == 0);
        assert(panelsOnly.scene.mesh.triangles.empty() && panelsOnly.scene.renderFallbackMesh.triangles.empty());
        assert(panelsOnly.scene.coverage.front().status == core::GeometryCoverage::Excluded);
    }
    // An unpaired animation name does not establish an operable passage.
    WritePassage(root / "meshes/Unpaired.nif", false, true, false);
    const core::Cell unpaired{
        .references = {{.id = 1, .recordType = "REFR", .modelPath = "Unpaired.nif", .baseRecordType = "ACTI"}}};
    const auto geometry = skyrim::ExtractGeometry(root, unpaired, root);
    assert(geometry.scene.mesh.triangles.size() == 24);
    WritePassage(root / "meshes/Architecture/Animated/Unpaired.nif", false, true, false);
    auto animated = unpaired;
    animated.references.front().modelPath = "Architecture/Animated/Unpaired.nif";
    const auto excluded = skyrim::ExtractGeometry(root, animated, root);
    assert(excluded.modelsExcluded == 1 && excluded.scene.mesh.triangles.empty());
}
