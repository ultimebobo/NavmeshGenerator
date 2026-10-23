#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/candidate_cache.h"
#include "app/geometry_pipeline.h"
#include "core/navmesh/generator.h"
#include "skyrim/extraction/collision_impact.h"
#include "skyrim/parser/base_object_swapper.h"

#include <NifFile.hpp>
#include <bhk.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numbers>
#include <set>

namespace
{
    using namespace navmesh;

    void WriteModel(const std::filesystem::path &path)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        const std::vector<nifly::Vector3> vertices{{0, 0, 0}, {100, 0, 0}, {0, 100, 100}};
        const std::vector<nifly::Triangle> triangles{{0, 1, 2}};
        assert(nif.CreateShapeFromData("SyntheticSolid", &vertices, &triangles, nullptr));
        constexpr auto units = 69.99125F;
        auto data = std::make_unique<nifly::hkPackedNiTriStripsData>();
        data->numVerts = 3;
        data->compressedVertData = {{0, 0, 0}, {100 / units, 0, 0}, {0, 100 / units, 100 / units}};
        data->keyCount = 1;
        data->triData.resize(1);
        data->triData.front().tri = {0, 1, 2};
        auto shape = std::make_unique<nifly::bhkPackedNiTriStripsShape>();
        shape->dataRef.index = nif.GetHeader().AddBlock(std::move(data));
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        std::filesystem::create_directories(path.parent_path());
        assert(nif.Save(path) == 0);
    }

    skyrim::ResolvedLoadOrder Fixture()
    {
        skyrim::ResolvedLoadOrder result;
        result.plugins = {"Base.esm", "Light.esl", "Override.esp"};
        result.pluginFormPrefixes = {{"base.esm", 0}, {"light.esl", 0xFE000000}, {"override.esp", 0x01000000}};
        for (int x = 0; x < 3; ++x)
        {
            const auto id = static_cast<std::uint32_t>(0x100 + x);
            result.cells.push_back({.id = id,
                                    .editorId = "Cell" + std::to_string(x),
                                    .exteriorCoordinates = std::array<std::int32_t, 2>{x, 0}});
            skyrim::ResolvedRecord cell{
                .type = "CELL", .formId = id, .editorId = "Cell" + std::to_string(x), .worldspaceFormId = 1};
            cell.winning = {.plugin = "Base.esm", .formId = id, .worldspaceFormId = 1};
            cell.origins = {cell.winning};
            result.records.push_back(cell);
        }
        for (const auto id : {10U, 20U, 30U, 0xFE000801U})
        {
            skyrim::ResolvedRecord base{
                .type = "STAT", .formId = id, .editorId = "Base" + std::to_string(id), .modelPath = "Solid.nif"};
            base.winning = {.plugin = id == 0xFE000801 ? "Light.esl" : "Base.esm",
                            .formId = id,
                            .modelRadius = 200.0F,
                            .hasModel = true,
                            .modelPath = "Solid.nif"};
            base.origins = {base.winning};
            result.records.push_back(base);
        }
        core::Reference reference{.id = 0x200,
                                  .baseObjectId = 10,
                                  .recordType = "REFR",
                                  .modelPath = "Solid.nif",
                                  .position = {100, 100, 0},
                                  .sourcePlugin = "Base.esm",
                                  .basePlugin = "Base.esm",
                                  .baseRecordType = "STAT"};
        result.cells.front().references.push_back(reference);
        skyrim::ResolvedRecord placed{.type = "REFR",
                                      .formId = reference.id,
                                      .cellFormId = 0x100,
                                      .worldspaceFormId = 1,
                                      .referencedFormIds = {10}};
        placed.winning = {.plugin = "Base.esm",
                          .formId = reference.id,
                          .cellFormId = 0x100,
                          .worldspaceFormId = 1,
                          .baseFormId = 10,
                          .position = reference.position,
                          .modelRadius = 200.0F,
                          .hasModel = true,
                          .modelPath = "Solid.nif"};
        placed.origins = {placed.winning};
        result.records.push_back(placed);
        return result;
    }

    skyrim::BaseObjectSwapInput Input(const std::string &text)
    {
        return {.configs = {{"test_swap.ini", {}, text}}};
    }

    const core::Reference &First(const skyrim::ResolvedLoadOrder &order)
    {
        return order.cells.front().references.front();
    }

    void TestConfigSyntax(const std::filesystem::path &root)
    {
        auto order = Fixture();
        const auto diagnostics =
            skyrim::ApplyBaseObjectSwaps(order, Input("\xEF\xBB\xBF; comment\r\n0x200~Base.esm|Base20\r\n"
                                                      "[ Forms ] ; base swaps\r\nBase10|Base30\r\n"
                                                      "[ Transforms ] # properties\r\n0x200~Base.esm|scale(2)\r\n"));
        assert(diagnostics.empty());
        assert(First(order).baseObjectId == 20);
        assert(First(order).scale == 2);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms] ; first\nBase10|Base20\n[Transforms]\nBase10|scale(2)\n"
                         "[ Forms ] # repeat\nBase10|Base30\n[Transforms|Cell0] ; conditional\nBase10|scale(3)\n"));
        assert(First(order).baseObjectId == 30 && First(order).scale == 3);

        for (const auto &text : {"; comment\n[Forms ; missing bracket\n", "; comment\ninvalid entry\n"})
        {
            auto input = Input(text);
            input.configs.front().path = root / "Data" / "Broken_SWAP.ini";
            order = Fixture();
            const auto warnings = skyrim::ApplyBaseObjectSwaps(order, input);
            assert(warnings.size() == 1);
            assert(warnings.front().starts_with(input.configs.front().path.generic_string() + ":2:"));
            assert(order.runtimePlacements.empty());
        }

        {
            auto input = Input("[Forms]\nBase10|Base20|posA(1,2)\n");
            input.configs.front().path = root / "Data" / "Broken_SWAP.ini";
            order = Fixture();
            const auto warnings = skyrim::ApplyBaseObjectSwaps(order, input);
            assert(warnings.empty());
            assert(First(order).baseObjectId == 20);
            assert(First(order).position.x == 0 && First(order).position.y == 0 && First(order).position.z == 0);
            assert(!First(order).envelope);
        }
    }

    void TestRuntimeCompatibility()
    {
        // Unrecognized chance fields keep BOS's always-pass default, including legacy numeric values.
        for (const auto *chance : {"100", "50", "chanceS(100", "unknown", "chanceS(100)", "chance(100) suffix"})
        {
            auto order = Fixture();
            const auto diagnostics =
                skyrim::ApplyBaseObjectSwaps(order, Input(std::string("[Forms]\nBase10|Base20|NONE|") + chance + "\n"));
            assert(diagnostics.empty());
            assert(First(order).baseObjectId == 20 && !First(order).envelope);
        }

        auto order = Fixture();
        const auto diagnostics =
            skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nAbsentSource|Base20|posA(broken)|chance(broken)\n"
                                                      "Base10|AbsentReplacement|posA(broken)|chance(broken)\n"
                                                      "[Properties]\nBase10,Base20|posA(broken)\n"
                                                      "[Forms]\nBase10|Base20|NONE|100|ignored\n"
                                                      "[Properties|Cell0|ignored]\nBase10|posR(50,0,0)|100|ignored\n"));
        assert(diagnostics.size() == 3);
        assert(First(order).baseObjectId == 20 && First(order).position.x == 150);
        assert(!First(order).envelope);

        // Runtime lookup adds rather than masks or ORs the ID. Carries can name a record
        // owned by another plugin; both matching and failed lookups must follow that identity.
        for (const auto local : {"0x4390", "0x390"})
        {
            order = Fixture();
            order.pluginFormPrefixes["light.esl"] = 0xFE005000;
            order.cells.front().references.front().id = 0xFE009390;
            order.records.back().formId = 0xFE009390;
            order.records.back().winning.formId = 0xFE009390;
            (void)skyrim::ApplyBaseObjectSwaps(
                order, Input(std::string("[Transforms]\n") + local + "~Light.esl|posA(300,400,500)\n"));
            assert(First(order).position.x == (std::string(local) == "0x4390" ? 300 : 100));
        }

        order = Fixture();
        order.pluginFormPrefixes["light.esl"] = 0xFE005000;
        auto replacement = *order.FindWinning(20);
        replacement.formId = 0xFE009390;
        order.records.push_back(replacement);
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|0x4390~Light.esl\n"));
        assert(First(order).baseObjectId == 0xFE009390);
        assert(!First(order).envelope);
    }

    void TestPermissivePropertyGrammar()
    {
        struct PointCase
        {
            const char *text;
            core::Vec3 expected;
        };
        // Expected values follow BOS v3.5.0 ObjectProperties and its pinned REX conversions.
        for (const auto &test : std::vector<PointCase>{{"posA(1,2)", {0, 0, 0}},
                                                       {"posR(1,2)", {100, 100, 0}},
                                                       {"posA", {0, 0, 0}},
                                                       {"unknown(pos)", {0, 0, 0}},
                                                       {"prefixposSuffix(10units,20units,30units)", {10, 20, 30}},
                                                       {"prefixRposSuffix(10,20,30)", {110, 120, 30}},
                                                       {"rotpos(10,20,30)", {10, 20, 30}},
                                                       {"pos(1,2,3,4)", {1, 2, 3}},
                                                       {"pos(1,2,3)rot(4,5,6)", {1, 2, 3}},
                                                       {"pos(9,9,9),posR(1,2,3)", {101, 102, 3}},
                                                       {",pos(1,2,3), unknown text", {1, 2, 3}},
                                                       {"pos(1,2,3),postext", {0, 0, 0}}})
        {
            auto order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(order, Input(std::string("[Properties]\nBase10|") + test.text + "\n"));
            const auto &reference = First(order);
            assert(reference.position.x == test.expected.x && reference.position.y == test.expected.y &&
                   reference.position.z == test.expected.z);
            assert(reference.rotation.x == 0 && reference.rotation.y == 0 && reference.rotation.z == 0);
            assert(!reference.envelope);
        }
        for (const auto &[text, expected] : std::vector<std::pair<std::string, float>>{{"scale", 0.0F},
                                                                                       {"scale(2,4)", 2.0F},
                                                                                       {"prefixAscale(150units)", 1.5F},
                                                                                       {"scaleA(inf)", 10.0F},
                                                                                       {"scaleA(-inf)", 0.0F}})
        {
            auto order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\nBase10|" + text + "\n"));
            assert(First(order).scale == expected && !First(order).envelope);
        }
        for (const auto *text : {"pos(10/20/999,0,0)", "pos(20/10,0,0)"})
        {
            auto order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(order, Input(std::string("[Properties]\nBase10|") + text + "\n"));
            assert(First(order).envelope);
            assert(First(order).envelope->minimumPosition.x == 10);
            assert(First(order).envelope->maximumPosition.x == 20);
        }
        auto order = Fixture();
        order.cells.front().references.front().rotation.z = 1;
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\nBase10|Rrot(0,0,90units)\n"));
        assert(std::abs(First(order).rotation.z - (1 + std::numbers::pi_v<float> / 2)) < 0.0001F);

        for (const auto *text : {"flags(0xC00)", "prefixCflags(0x800suffix)", "flagsC(-1)"})
        {
            order = Fixture();
            order.cells.front().references.front().initiallyDisabled = true;
            (void)skyrim::ApplyBaseObjectSwaps(order, Input(std::string("[Properties]\nBase10|") + text + "\n"));
            assert(!First(order).initiallyDisabled);
        }
        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\nBase10|flags(0x800suffix),flagsC(0x800)\n"));
        assert(First(order).initiallyDisabled);
        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\nBase10|unknown text,flags()\n"));
        assert(order.runtimePlacements.empty());
    }

    void TestPermissiveIdentityAndChanceGrammar()
    {
        for (const auto *source : {"0xAtrail~Base.esm", "0xA,0x1E~Base.esm", "-0xFFFFFFF6~Base.esm"})
        {
            auto order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(
                order, Input(std::string("[Properties]\n") + source + "|posR(1tail,2tail,3tail)\n"));
            assert(First(order).position.x == 101 && First(order).position.y == 102 && First(order).position.z == 3);
        }
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order,
                                           Input("[Forms]\n0xA,0x1E~Base.esm|0x14tail~Base.esm|NONE|chance(-1)\n"));
        assert(order.runtimePlacements.empty());
        order = Fixture();
        order.records[3].editorId = "Odd~Editor~Name";
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nOdd~Editor~Name|Base20\n"));
        assert(First(order).baseObjectId == 20);

        for (const auto *chance : {"chance(100tail,123tail,ignored)", "chance(nan,-1)", "chance(inf,0)",
                                   ")prefixchance(100)", " chance(100) suffix"})
        {
            order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(order,
                                               Input(std::string("[Forms]\nBase10|Base20|NONE|") + chance + "\n"));
            assert(First(order).baseObjectId == 20 && !First(order).envelope);
        }
        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\nBase10|Base30|NONE|chance(-inf)\n"
                                                        "Base10|Base10|NONE|chance(broken,broken)\n"));
        assert(First(order).baseObjectId == 20 && !First(order).envelope);
        for (const auto *text : {"[Forms]\nBase10|Base20|NONE|chance(100,invalid)\n",
                                 "[Forms]\nBase10,Base30|Base20,Base30|NONE|chance(100,invalid)\n",
                                 "[Properties]\nBase10|pos(1,invalid,3)\n", "[Properties]\nBase10|pos(inf,0,0)\n"})
        {
            order = Fixture();
            bool failed{};
            try
            {
                (void)skyrim::ApplyBaseObjectSwaps(order, Input(text));
            }
            catch (const std::runtime_error &error)
            {
                failed = true;
                assert(std::string(error.what()).starts_with("test_swap.ini:2:"));
            }
            assert(failed && order.runtimePlacements.empty());
        }
    }

    void TestIniEdgeGrammar()
    {
        for (const auto *text : {"[Forms|]\nBase10|Base20\n[Forms]\nBase10|Base30\n", "[ \n Forms ]\nBase10|Base20\n",
                                 "[Forms]\nBase10|Base20\n[Broken\nBase10|Base30\n"})
        {
            auto order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(order, Input(text));
            assert(First(order).baseObjectId == 20 && !First(order).envelope);
        }
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties|]\nBase10|scale(2)\n"
                                                        "[Properties]\nBase10|scale(3)\n"));
        assert(First(order).scale == 2 && !First(order).envelope);
        std::string text = "[Forms]\nBase10|Base20\n";
        text.push_back('\0');
        text += "Base10|Base30\n";
        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input(text));
        assert(First(order).baseObjectId == 20);
    }

    void TestIniEnumeration()
    {
        for (const auto &text : {"[Forms]\nBase10|Base20\nBase10|Base30\nbase10|base20\n",
                                 "[Forms]\nBase10|Base20\n[forms]\nBase10|Base30\n",
                                 "[Forms]\rBase10|Base20 = ignored value\rBase10|Base30 = another value\r"})
        {
            auto order = Fixture();
            const auto diagnostics = skyrim::ApplyBaseObjectSwaps(order, Input(text));
            assert(diagnostics.empty());
            assert(First(order).baseObjectId == 30);
            assert(!First(order).envelope);
        }
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Custom]\n0x200~Base.esm|Base20\n"
                                                        "[Custom|Cell0]\nBase10|posR(50,0,0)\n"));
        assert(First(order).baseObjectId == 20 && First(order).position.x == 150);

        order = Fixture();
        auto &base = order.records[3];
        assert(base.formId == 10);
        base.editorId = "ABCD";
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nABCD|Base20\n"));
        assert(First(order).baseObjectId == 20);

        order = Fixture();
        const auto diagnostics = skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10 |Base20\n"));
        assert(!diagnostics.empty());
        assert(order.runtimePlacements.empty());
    }

    void TestLookupFallthroughAndConditions()
    {
        auto order = Fixture();
        const auto diagnostics = skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base30\nBase10|Base20,0xFFFF~Base.esm|posR(50,0,0)\n"));
        assert(!diagnostics.empty());
        std::set<std::uint32_t> alternatives;
        for (const auto &reference : order.cells.front().references)
        {
            alternatives.insert(reference.baseObjectId);
            assert(reference.envelope);
            assert(reference.position.x == (reference.baseObjectId == 20 ? 150 : 100));
        }
        assert(alternatives == std::set<std::uint32_t>({20, 30}));

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base30\nBase10|Base20,Cell0\n"));
        alternatives.clear();
        for (const auto &reference : order.cells.front().references)
        {
            alternatives.insert(reference.baseObjectId);
        }
        assert(alternatives == std::set<std::uint32_t>({20, 30}));

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\n[Forms|Base20]\nBase10|Base30\n"));
        assert(First(order).baseObjectId == 20 && !First(order).envelope);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\n"
                                                        "Base10,0xFFFF~Base.esm,Base30|Base20,Base30\n"));
        assert(First(order).baseObjectId == 20 && !First(order).envelope);
    }

    void TestPropertyAliasesAndChanceBounds()
    {
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order,
                                           Input("[Transforms]\nBase10|pos(200 , 100, 0),rot(0,0,90 ),scaleR(2 )\n"));
        assert(First(order).position.x == 200 && First(order).scale == 2);
        assert(std::abs(First(order).rotation.z - std::numbers::pi_v<float> / 2) < 0.0001F);
        assert(!First(order).envelope);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20|NONE|chance(150)\n"
                                                        "Base10|Base30|NONE|chance(-1)\n"));
        assert(First(order).baseObjectId == 20 && !First(order).envelope);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\nBase10|Base10|flags(0)\n"));
        assert(First(order).baseObjectId == 10 && !First(order).envelope);
    }

    void TestUnknownProperties()
    {
        for (const auto *section : {"Properties", "Transforms"})
        {
            auto order = Fixture();
            const auto diagnostics = skyrim::ApplyBaseObjectSwaps(
                order, Input(std::string("[") + section + "]\nBase10|PosA(9000,0,0),unknown(not-a-number)\n"));
            assert(diagnostics.empty());
            assert(order.runtimePlacements.empty());
            assert(First(order).position.x == 100);

            order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(
                order, Input(std::string("[") + section +
                             "]\nBase10|PosA(9000,0,0),posR(50,0,0),unknown(a,b),scale(2),Unknown()\n"));
            assert(First(order).position.x == 150);
            assert(First(order).scale == 2);
            assert(!First(order).envelope);

            order = Fixture();
            (void)skyrim::ApplyBaseObjectSwaps(
                order, Input(std::string("[") + section + "]\nBase10|posR(50,0,0)\nBase10|PosA(9000,0,0)\n"));
            assert(First(order).position.x == 150);
        }

        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20|PosA(9000,0,0)\n"
                                                        "[Transforms]\nBase10|posR(50,0,0)\n"));
        assert(First(order).baseObjectId == 20);
        assert(First(order).position.x == 150);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20|PosA(9000,0,0),scale(2)\n"
                                                        "[Transforms]\nBase10|posR(50,0,0)\n"));
        assert(First(order).baseObjectId == 20);
        assert(First(order).position.x == 100);
        assert(First(order).scale == 2);
    }

    void TestPrecedenceAndUnits()
    {
        auto order = Fixture();
        const auto diagnostics = skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base20|posA(9000,100,0)\n"
                         "[References]\n0x200~Base.esm|0x801~Light.esl|rotA(0,0,90),scaleA(150)\n"
                         "[Transforms]\n0x200~Base.esm|posA(5000,0,0)\n"));
        assert(diagnostics.empty());
        assert(First(order).baseObjectId == 0xFE000801);
        assert(First(order).position.x == 100); // The winning swap's whole property bundle takes priority.
        assert(std::abs(First(order).rotation.z - std::numbers::pi_v<float> / 2) < 0.0001F);
        assert(First(order).scale == 1.5F);
        assert(!First(order).envelope);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Transforms]\nBase10|posA(1000,100,0)\n[Properties]\nBase10|posA(2000,100,0)\n"
                         "[Transforms]\nBase10|posA(3000,100,0)\n"));
        assert(First(order).position.x == 2000); // Repeated sections are grouped before precedence resolution.

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base20|none|none\n[Transforms]\nBase10|rotA(0,0,720)\n"));
        assert(std::abs(First(order).rotation.z - std::numbers::pi_v<float> * 4) < 0.0001F);
        assert(order.runtimePlacements.size() == 1);
        assert(order.FindWinning(0x200)->winning.position->x == 100); // Encoded/plugin history is untouched.

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base20\n[Transforms]\nBase10|posR(4000,0,0),scale(1.25)\n"
                         "[Properties]\n0x200~Base.esm|posR(4100,0,0),scale(1.5)\n"));
        assert(First(order).baseObjectId == 20);
        assert(First(order).position.x == 4200);
        assert(First(order).scale == 1.5F);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base20\nBase10|Base30\n[Forms|Cell0]\nBase10|Base20\n"));
        assert(First(order).baseObjectId == 20);
        assert(!First(order).envelope);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms|-Cell0]\nBase10|Base30\n[Forms]\nBase10|Base20\n"));
        assert(First(order).baseObjectId == 20);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\n0x200~Base.esm|flags(0x800)\n"));
        assert(First(order).initiallyDisabled);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(
            order, Input("[Forms]\nBase10|Base20\nBase10|Base10|NONE\n[References|Cell0]\nBase10|posR(50,0,0)\n"));
        assert(First(order).baseObjectId == 20);
        assert(First(order).position.x == 150);

        order = Fixture();
        const auto invalidSet =
            skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10,Base20,Base30|Base20,Base30\n"));
        assert(!invalidSet.empty());
        assert(order.runtimePlacements.empty());
    }

    void TestAdditionalBasesRequireRuntimePlacement(const std::filesystem::path &root)
    {
        auto order = Fixture();
        auto &cell = order.cells.front();
        const auto staticReference = cell.references.front();
        for (const auto *type : {"LVLI", "MISC", "WEAP", "ARMO", "AMMO", "ALCH", "BOOK", "SLGM", "KEYM", "INGR", "SCRL",
                                 "LIGH", "HAZD", "IDLM"})
        {
            auto reference = staticReference;
            reference.id += static_cast<std::uint32_t>(cell.references.size());
            reference.baseObjectId = reference.id + 0x1000;
            reference.baseRecordType = type;
            auto base = *order.FindWinning(10);
            base.type = type;
            base.formId = reference.baseObjectId;
            base.winning.formId = base.formId;
            base.origins = {base.winning};
            if (base.type == "ARMO")
            {
                base.raw = skyrim::PluginRecord{};
                base.raw->subrecords.push_back(
                    {.type = "MOD4",
                     .data = std::vector<std::uint8_t>{'F', 'e', 'm', 'a', 'l', 'e', '.', 'n', 'i', 'f', 0}});
            }
            order.records.push_back(base);
            auto placed = *order.FindWinning(staticReference.id);
            placed.formId = reference.id;
            placed.referencedFormIds = {reference.baseObjectId};
            placed.winning.formId = reference.id;
            placed.winning.baseFormId = reference.baseObjectId;
            placed.origins = {placed.winning};
            order.records.push_back(placed);
            cell.references.push_back(reference);
        }
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry =
                skyrim::ExtractGeometry(root, cell, root / "cache", {}, {}, nullptr, nullptr, navigationOnly);
            assert(geometry.collisionTriangles == 1);
            assert(geometry.modelsLoaded == 1);
            assert(geometry.modelsExcluded == cell.references.size() - 1);
            assert(geometry.scene.mesh.triangles.size() == 1);
        }

        // An unrelated configuration cannot enable the indexed inventory models.
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\nBase20|scale(2)\n"));
        assert(order.runtimePlacements.empty());
        const auto untouched = skyrim::ExtractGeometry(root, cell, root / "cache");
        assert(untouched.collisionTriangles == 1);

        // An ineffective property must not admit armor through its alternate ground model.
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\n0x204~Base.esm|Unknown(value)\n"));
        assert(order.runtimePlacements.empty());
        const auto ineffective = skyrim::ExtractGeometry(root, cell, root / "cache");
        assert(ineffective.collisionTriangles == 1);

        // A targeted transform admits its additional base without admitting other clutter.
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\n0x202~Base.esm|posR(50,0,0)\n"));
        assert(order.runtimePlacements.size() == 1);
        const auto changed = skyrim::ExtractGeometry(root, cell, root / "cache");
        assert(changed.collisionTriangles == 2);
        assert(changed.modelsLoaded == 2);
        assert(changed.modelsExcluded == cell.references.size() - 2);
    }

    void TestRandomOutcomes(const std::filesystem::path &root)
    {
        auto order = Fixture();
        auto diagnostics =
            skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20,Base30|NONE|chanceR(50)\n"));
        assert(!diagnostics.empty());
        assert(order.cells.front().references.size() == 3);
        std::set<std::uint32_t> choices;
        for (const auto &reference : order.cells.front().references)
        {
            assert(reference.envelope);
            choices.insert(reference.baseObjectId);
        }
        assert(choices == std::set<std::uint32_t>({10, 20, 30}));

        order = Fixture();
        diagnostics = skyrim::ApplyBaseObjectSwaps(
            order, Input("[Transforms]\n0x200~Base.esm|posA(100/8300,100,50),rotR(0,0,0/360),scaleA(100/200)\n"));
        assert(!diagnostics.empty());
        const auto geometry =
            skyrim::ExtractGeometry(root, order.cells.front(), root / "cache", {}, {}, nullptr, nullptr, true);
        assert(geometry.modelsLoaded == 1);
        const auto &source = geometry.scene.geometrySources.front();
        assert(source.navigationObstacle && source.solidBounds);
        assert(source.solidBounds->min.x < 100);
        assert(source.solidBounds->max.x > 8300);
        assert(source.solidBounds->Contains({4000, 100, 50}));
        const skyrim::CellImpactIndex index(order);
        skyrim::ModelGeometryCache cache;
        skyrim::ImpactSelectionStatistics statistics;
        const auto affected = skyrim::SelectCollisionAffectedCells(
            order, index, {.dataDirectory = root, .cacheDirectory = root / "cache"}, cache, statistics);
        assert(affected.size() == 3);
        assert(statistics.changedRecords.at("BOS") == 1);
        assert(index.GeometryNeighbors(order.cells.back(), 0).size() >= 2);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms|RuntimeKeyword]\nBase10|Base20\n"));
        assert(order.cells.front().references.size() == 2);
        assert(First(order).envelope);

        order = Fixture();
        order.records.push_back({.type = "LVLI", .formId = 60, .editorId = "LeveledBase", .leveledBaseIds = {20, 30}});
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\nBase10|LeveledBase|posR(100,0,0)\n"));
        bool unchangedBaseWithProperties{};
        for (const auto &reference : order.cells.front().references)
        {
            unchangedBaseWithProperties =
                unchangedBaseWithProperties ||
                (reference.baseObjectId == 10 && reference.position.x == 200 && reference.envelope);
        }
        assert(unchangedBaseWithProperties);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\nBase10|scale(1/1000)\n"));
        assert(First(order).envelope);
        assert(First(order).envelope->maximumScale == UINT16_MAX / 100.0F);
    }

    void TestMovementAndRemoval(const std::filesystem::path &root)
    {
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\n0x200~Base.esm|posA(5000,100,0)\n"));
        const skyrim::CellImpactIndex index(order);
        assert(index.GeometryCell(order.cells.front()).references.empty());
        assert(index.GeometryCell(order.cells[1]).references.size() == 1);
        skyrim::ModelGeometryCache cache;
        skyrim::ImpactSelectionStatistics statistics;
        const auto affected = skyrim::SelectCollisionAffectedCells(
            order, index, {.dataDirectory = root, .cacheDirectory = root / "cache"}, cache, statistics);
        assert(affected.size() == 2);
        assert(affected[0]->id == 0x100 && affected[1]->id == 0x101);

        order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Properties]\n0x200~Base.esm|flags(0x800)\n"));
        const skyrim::CellImpactIndex disabledIndex(order);
        const auto disabledAffected = skyrim::SelectCollisionAffectedCells(
            order, disabledIndex, {.dataDirectory = root, .cacheDirectory = root / "cache"}, cache, statistics);
        assert(disabledAffected.size() == 1);
    }

    void TestFilledVolumeAndCache(const std::filesystem::path &root)
    {
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Transforms]\n0x200~Base.esm|posA(200/500,200,50)\n"));
        auto geometry =
            skyrim::ExtractGeometry(root, order.cells.front(), root / "cache", {}, {}, nullptr, nullptr, true);
        auto &scene = geometry.scene;
        const core::NavMesh roof{.vertices = scene.mesh.vertices, .polygons = {{.vertices = {4, 5, 6}}}};
        const auto roofAnalysis =
            analysis::AnalyzeNavMeshPolygons(roof, scene.mesh, app::detail::BuildTriangleSources(geometry),
                                             {.surfaceSearchRadius = 64, .maxSupportDistance = 2, .maxSlope = 45});
        assert(roofAnalysis.summary.supportFound == 0);
        const auto source = scene.geometrySources.size();
        scene.geometrySources.push_back({.sourceType = core::GeometrySourceType::Terrain});
        const auto vertex = static_cast<std::uint32_t>(scene.mesh.vertices.size());
        scene.mesh.vertices.insert(scene.mesh.vertices.end(),
                                   {{0, 0, 100}, {1000, 0, 100}, {1000, 1000, 100}, {0, 1000, 100}});
        scene.mesh.triangles.push_back({{vertex, vertex + 1, vertex + 2}});
        scene.mesh.triangles.push_back({{vertex, vertex + 2, vertex + 3}});
        scene.triangleProvenance.push_back({source, 0});
        scene.triangleProvenance.push_back({source, 1});
        core::NavigationProfile profile;
        profile.agentRadius = 8;
        profile.agentHeight = 32;
        profile.stepHeight = 8;
        profile.minimumRegionArea = 1;
        const core::RecastSettings settings{.cellSize = 4, .cellHeight = 2};
        const auto candidate = core::RecastCandidateGenerator{}.Generate(
            scene, profile, {}, {}, core::RegionPartitioningAlgorithm::Watershed, settings,
            core::CandidateRetention::AllWalkable);
        assert(!candidate.mesh.polygons.empty());
        for (const auto &polygon : candidate.mesh.polygons)
        {
            const auto center =
                (candidate.mesh.vertices[polygon.vertices[0]] + candidate.mesh.vertices[polygon.vertices[1]] +
                 candidate.mesh.vertices[polygon.vertices[2]]) /
                3.0F;
            assert(!(center.x > 200 && center.x < 600 && center.y > 200 && center.y < 300));
            assert(center.z < 120); // The uncertainty envelope roof cannot become a floor.
        }
        const auto fingerprint = app::detail::CandidateFingerprint(scene, profile, {}, {}, {}, "watershed", settings);
        scene.geometrySources.front().solidBounds->max.x += 10;
        assert(fingerprint != app::detail::CandidateFingerprint(scene, profile, {}, {}, {}, "watershed", settings));
    }

    void TestDiscovery(const std::filesystem::path &root)
    {
        const auto data = root / "Data";
        const auto mod = root / "Mod";
        const auto overwrite = root / "Overwrite";
        for (const auto &directory : {data, mod, overwrite})
        {
            std::filesystem::create_directories(directory);
        }
        std::ofstream(data / "A_SWAP.ini") << "[Transforms]\nBase10|scale(2)\n";
        std::ofstream(mod / "A_SWAP.ini") << "[Transforms]\nBase10|scale(3)\n";
        std::ofstream(overwrite / "A_SWAP.ini") << "[Transforms]\nBase10|scale(4)\n";
        std::ofstream(mod / "Z_SWAP.ini") << "[Transforms]\nBase10|scale(5)\n";
        std::ofstream(mod / "ignored.ini") << "invalid";
        const auto input = skyrim::DiscoverBaseObjectSwaps({data, mod, overwrite});
        assert(input.configs.size() == 2);
        assert(input.configs.front().path.parent_path() == overwrite);
        auto order = Fixture();
        (void)skyrim::ApplyBaseObjectSwaps(order, input);
        assert(First(order).scale == 5);
        assert(skyrim::BaseObjectSwapSnapshotMatches(input));
        std::ofstream(mod / "Z_SWAP.ini") << "[Transforms]\nBase10|scale(6)\n";
        assert(!skyrim::BaseObjectSwapSnapshotMatches(input));
        assert(skyrim::WriteBaseObjectSwapReport(root / "bos-input.json", input, order, {}));
    }
} // namespace

void ExportBaseObjectSwapModel(const std::filesystem::path &path)
{
    WriteModel(path);
}

void TestBaseObjectSwapper()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-bos-tests-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    WriteModel(root / "meshes" / "Solid.nif");
    TestPrecedenceAndUnits();
    TestConfigSyntax(root);
    {
        auto order = Fixture();
        order.cells.front().references.front().editorMarker = true;
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\n"));
        assert(!First(order).editorMarker);
        order = Fixture();
        for (auto &record : order.records)
        {
            if (record.formId == 20)
            {
                record.winning.editorMarker = true;
            }
        }
        (void)skyrim::ApplyBaseObjectSwaps(order, Input("[Forms]\nBase10|Base20\n"));
        assert(First(order).editorMarker);
    }
    TestRuntimeCompatibility();
    TestPermissivePropertyGrammar();
    TestPermissiveIdentityAndChanceGrammar();
    TestIniEdgeGrammar();
    TestIniEnumeration();
    TestLookupFallthroughAndConditions();
    TestPropertyAliasesAndChanceBounds();
    TestUnknownProperties();
    TestAdditionalBasesRequireRuntimePlacement(root);
    TestRandomOutcomes(root);
    TestMovementAndRemoval(root);
    TestFilledVolumeAndCache(root);
    TestDiscovery(root);
    std::filesystem::remove_all(root);
}
