#include "skyrim/parser/navigation_audit.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace
{
    using namespace navmesh;

    core::NavMesh Square(std::uint32_t id)
    {
        return {.id = id,
                .vertices = {{0, 0, 0}, {100, 0, 0}, {100, 100, 0}, {0, 100, 0}},
                .polygons = {{{0, 1, 2}, {0xffff, 0xffff, 1}}, {{0, 2, 3}, {0, 0xffff, 0xffff}}}};
    }

    bool Has(const skyrim::NavigationAudit &report, std::string_view kind)
    {
        return std::any_of(report.findings.begin(), report.findings.end(),
                           [&](const auto &finding) { return finding.kind == kind; });
    }
} // namespace

void TestNavigationAudit()
{
    using namespace navmesh;
    skyrim::ResolvedLoadOrder input;
    input.cells = {{.id = 0x100, .isInterior = true, .navMeshes = {Square(0x200)}},
                   {.id = 0x101, .isInterior = true, .navMeshes = {{.id = 0x201}}},
                   {.id = 0x102, .isInterior = true}};
    auto audit = [&] { return skyrim::AuditNavigation(input, 32); };
    assert(audit().findings.empty());
    assert(audit().scannedNavmeshes == 2);

    // One-sided indexed shared edges retain their direction without selecting an automatic rebuild.
    auto &oneSided = input.cells.front().navMeshes.front();
    oneSided.polygons[1].neighbors[0] = 0xffff;
    assert(audit().findings.empty());
    oneSided.vertices.push_back(oneSided.vertices[0]);
    oneSided.polygons[1].vertices[0] = 4;
    assert(Has(audit(), "invalid_adjacency"));
    oneSided = Square(0x200);

    // A defect in one winning mesh selects only its owning CELL, with bounded evidence per kind.
    auto &mesh = input.cells.front().navMeshes.front();
    mesh.polygons.front().neighbors[0] = 99;
    mesh.polygons.back().neighbors[2] = 88;
    const auto broken = audit();
    assert(broken.findings.size() == 1 && broken.findings.front().kind == "invalid_adjacency");
    assert(broken.findings.front().cellId == 0x100 && broken.findings.front().navmeshId == 0x200);
    mesh = Square(0x200);
    mesh.polygons.front().vertices[0] = 99;
    assert(Has(audit(), "invalid_triangle_vertices"));
    mesh = Square(0x200);
    mesh.vertices.front().x = std::numeric_limits<float>::quiet_NaN();
    assert(Has(audit(), "non_finite_vertex"));
    mesh = Square(0x200);
    mesh.polygons.front().vertices = {0, 0, 0};
    assert(Has(audit(), "degenerate_triangle"));
    mesh = Square(0x200);
    mesh.polygons.front().vertices = {2, 1, 0};
    assert(Has(audit(), "inverted_triangle"));
    mesh = Square(0x200);
    mesh.polygons.push_back(mesh.polygons.front());
    assert(Has(audit(), "duplicate_triangle"));
    assert(Has(audit(), "non_manifold_edge"));

    // Disconnected islands, unused vertices, disabled triangles and vertical authored faces are permissible.
    mesh = Square(0x200);
    mesh.polygons.back().flags |= 1U << 3;
    mesh.polygons.front().neighbors[2] = 0xffff;
    mesh.polygons.back().vertices[0] = 99;
    assert(audit().findings.empty());
    mesh.vertices[3].x = std::numeric_limits<float>::quiet_NaN();
    assert(audit().findings.empty());
    mesh = Square(0x200);
    mesh.polygons.front().neighbors[2] = 0xffff;
    mesh.polygons.back().neighbors[0] = 0xffff;
    assert(audit().findings.empty());
    mesh = Square(0x200);
    mesh.polygons.resize(1);
    mesh.polygons.front().neighbors.fill(0xffff);
    mesh.vertices = {{0, 0, 0}, {0, 0, 100}, {0, 100, 100}};
    assert(audit().findings.empty());

    // Portal validation uses resolved identities, allows authored offsets and understands ledge reverse types.
    mesh = Square(0x200);
    auto &other = input.cells[1].navMeshes.front();
    other = Square(0x201);
    mesh.polygons.front().flags = 1;
    mesh.polygons.front().neighbors[0] = 0;
    mesh.externalLinks = {{0, 0, 0x201, 0, 1}};
    other.polygons.front().flags = 1;
    other.polygons.front().neighbors[0] = 0;
    other.externalLinks = {{0, 0, 0x200, 0, 2}};
    assert(audit().findings.empty());
    other.externalLinks.clear();
    assert(Has(audit(), "invalid_external_link"));
    other.externalLinks = {{0, 0, 0x200, 0, 2}};
    mesh.externalLinks.front().targetPolygon = 99;
    assert(Has(audit(), "invalid_external_link"));
    mesh = Square(0x200);
    other = {};
    mesh.doorLinks = {{99, 0x300}};
    assert(Has(audit(), "invalid_door_link"));

    // Truncated tables and null door destinations must remain visible after scene decoding drops them.
    mesh = Square(0x200);
    input.records = {{.type = "NAVM",
                      .formId = 0x200,
                      .raw = skyrim::PluginRecord{.decodedPayload = std::vector<std::uint8_t>(4)},
                      .navm = skyrim::NavmLayout{.trailingData = {0, 4}, .supported = true}}};
    assert(Has(audit(), "malformed_connections"));
    std::vector<std::uint8_t> doorTable(18);
    doorTable[4] = 1;
    input.records.front().raw->decodedPayload = std::move(doorTable);
    input.records.front().navm->trailingData.size = 18;
    assert(Has(audit(), "invalid_door_link"));

    // Deleted winning records and unsupported layouts do not become automatic repair targets.
    input.records = {{.type = "NAVM", .formId = 0x200, .winning = {.deleted = true}},
                     {.type = "NAVM", .formId = 0x202, .navm = skyrim::NavmLayout{.supported = false}}};
    const auto excluded = audit();
    assert(excluded.findings.empty() && excluded.unsupportedNavmeshes == 1);
    assert(skyrim::AuditNavigation(input, 32, [] { return true; }).scannedNavmeshes == 0);
}
