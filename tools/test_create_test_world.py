"""Binary-layout, CLI stitching and door-link regressions for the playable world fixture."""

import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import create_test_world as fixture


EXE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None


def read_records(data):
    """Inspect exact record/group envelopes independently of the fixture serializer."""
    records = []
    groups = []

    def visit(start, end, ancestors):
        while start < end:
            if end - start < 24:
                raise ValueError("Truncated record header")
            kind, size, flags, form_id = struct.unpack_from("<4sIII", data, start)
            stop = start + (size if kind == b"GRUP" else size + 24)
            if stop > end or stop < start + 24:
                raise ValueError("Invalid record size")
            if kind == b"GRUP":
                placement = (form_id, flags)  # group type, label
                groups.append(placement)
                visit(start + 24, stop, ancestors + [placement])
            else:
                records.append((kind, form_id, flags, data[start + 24:stop], ancestors))
            start = stop

    visit(0, len(data), [])
    return records, groups


def subrecords(payload):
    result = []
    offset = 0
    while offset < len(payload):
        kind, size = struct.unpack_from("<4sH", payload, offset)
        offset += 6
        if offset + size > len(payload):
            raise ValueError("Truncated subrecord")
        result.append((kind, payload[offset:offset + size]))
        offset += size
    return result


def decode_heights(payload):
    """Decode LAND delta heights to world units without using the builder's height function."""
    height_map = dict(subrecords(payload))[b"VHGT"]
    base = struct.unpack_from("<f", height_map)[0]
    heights = []
    for index, delta in enumerate(struct.unpack_from("<1089b", height_map, 4)):
        previous = base if not heights else heights[index - 33 if index % 33 == 0 else index - 1]
        heights.append(previous + delta)
    return [height * 8 for height in heights]


class TestWorldFixture(unittest.TestCase):
    def test_plugin_structure_terrain_and_teleport_markers(self):
        for size in (2, 3, 16):
            with self.subTest(size=size):
                plugin, manifest = fixture.build_plugin("SyntheticCell", size)
                records, groups = read_records(plugin)
                self.assertEqual({entry[0] for entry in records}, {b"TES4", b"WRLD", b"CELL", b"LAND", b"REFR"})
                header = dict(subrecords(records[0][3]))
                self.assertEqual(header[b"MAST"], b"Skyrim.esm\0")
                _, count, next_id = struct.unpack("<fII", header[b"HEDR"])
                self.assertEqual(count, len(records) - 1 + len(groups))
                ids = [entry[1] for entry in records[1:]]
                self.assertEqual(len(ids), len(set(ids)))
                self.assertTrue(all(form_id >> 24 == 1 for form_id in ids))
                self.assertEqual(next_id, max(ids) % 0x1000000 + 1)
                self.assertEqual(records[0][2], 0)  # Full ESP; never ESL-remapped.

                cells = {}
                terrain = {}
                markers = {}
                for kind, form_id, flags, payload, ancestors in records[1:]:
                    fields = dict(subrecords(payload))
                    if kind == b"CELL":
                        if flags & 0x400:
                            self.assertEqual(struct.unpack("<iiI", fields[b"XCLC"]), (0, 0, 0))
                            self.assertEqual([item[0] for item in ancestors], [0, 1])
                            continue
                        x, y, _ = struct.unpack("<iiI", fields[b"XCLC"])
                        cells[form_id] = (ancestors[1][1], x, y)
                        self.assertEqual([item[0] for item in ancestors], [0, 1, 4, 5])
                        self.assertEqual(struct.unpack("<hh", struct.pack("<I", ancestors[2][1])), (y // 32, x // 32))
                        self.assertEqual(struct.unpack("<hh", struct.pack("<I", ancestors[3][1])), (y // 8, x // 8))
                        self.assertFalse(struct.unpack("<H", fields[b"DATA"])[0] & 3)
                    elif kind in (b"LAND", b"REFR"):
                        if kind == b"REFR" and struct.unpack("<I", fields[b"NAME"])[0] == fixture.TELEPORT_DOOR:
                            continue
                        self.assertEqual([item[0] for item in ancestors], [0, 1, 4, 5, 6, 9])
                        owner = ancestors[-1][1]
                        self.assertEqual(owner, ancestors[-2][1])
                        if kind == b"LAND":
                            terrain[owner] = decode_heights(payload)
                            self.assertEqual(len(fields[b"VNML"]), 33 * 33 * 3)
                            self.assertEqual(len(fields[b"VCLR"]), 33 * 33 * 3)
                            textures = [struct.unpack("<IBBh", value) for sig, value in subrecords(payload) if sig == b"BTXT"]
                            self.assertEqual(textures, [(fixture.LAND_TEXTURE, quadrant, 0, -1) for quadrant in range(4)])
                        else:
                            base = struct.unpack("<I", fields[b"NAME"])[0]
                            self.assertIn(base, (fixture.COC_MARKER, fixture.DOOR_SURROUND))
                            if base == fixture.COC_MARKER:
                                markers[owner] = struct.unpack("<6f", fields[b"DATA"])
                self.assertEqual(len(cells), size * size + 1)
                self.assertEqual(cells.keys(), terrain.keys())
                self.assertEqual(cells.keys(), markers.keys())
                by_coordinates = {coordinates: terrain[form_id] for form_id, coordinates in cells.items()}
                for form_id, (world, x, y) in cells.items():
                    heights = terrain[form_id]
                    spawn = markers[form_id]
                    self.assertEqual(spawn[:2], (x * 4096 + 2048, y * 4096 + 2048))
                    self.assertGreater(spawn[2], heights[16 * 33 + 16])
                    if (world, x + 1, y) in by_coordinates:
                        self.assertEqual(heights[32::33], by_coordinates[(world, x + 1, y)][::33])
                    if (world, x, y + 1) in by_coordinates:
                        self.assertEqual(heights[-33:], by_coordinates[(world, x, y + 1)][:33])
                self.assertEqual(len(manifest["seams"]), 2 * size * (size - 1))
                self.assertEqual(manifest["coc"], "coc SyntheticCell")

    def test_door_route_is_persistent_reciprocal_and_grounded(self):
        for size in (2, 3, 16):
            with self.subTest(size=size):
                plugin, manifest = fixture.build_plugin("DoorTestCell", size)
                records, _ = read_records(plugin)
                by_id = {form_id: (kind, flags, dict(subrecords(payload)), ancestors)
                         for kind, form_id, flags, payload, ancestors in records}
                worlds = {form_id: fields[b"EDID"].rstrip(b"\0").decode()
                          for form_id, (kind, _, fields, _) in by_id.items() if kind == b"WRLD"}
                cells = {cell["editor_id"]: cell for cell in manifest["cells"]}
                endpoints = manifest["door_route"]
                self.assertEqual([point["point"] for point in endpoints], ["A", "B"])
                self.assertEqual(endpoints[0]["cell"], "DoorTestCell")
                self.assertNotEqual(endpoints[0]["worldspace"], endpoints[1]["worldspace"])
                self.assertEqual(sum(cell["worldspace"] == endpoints[1]["worldspace"]
                                     for cell in manifest["cells"]), 1)
                for endpoint, partner in ((endpoints[0], endpoints[1]), (endpoints[1], endpoints[0])):
                    kind, flags, fields, ancestors = by_id[int(endpoint["reference_id"], 16)]
                    self.assertEqual(kind, b"REFR")
                    self.assertEqual(flags, 0x400)  # Persistent, enabled and unlocked.
                    self.assertEqual([item[0] for item in ancestors], [0, 1, 6, 8])
                    self.assertEqual(ancestors[-1][1], ancestors[-2][1])
                    owner = by_id[ancestors[-1][1]]
                    self.assertEqual(owner[0], b"CELL")
                    self.assertTrue(owner[1] & 0x400)
                    self.assertEqual(struct.unpack("<iiI", owner[2][b"XCLC"]), (0, 0, 0))
                    self.assertEqual(worlds[ancestors[1][1]], endpoint["worldspace"])
                    self.assertEqual(struct.unpack("<I", fields[b"NAME"])[0], fixture.TELEPORT_DOOR)
                    self.assertNotIn(b"XNDP", fields)
                    self.assertNotIn(b"XLOC", fields)
                    self.assertNotIn(b"VMAD", fields)
                    self.assertEqual(struct.unpack("<6f", fields[b"DATA"]),
                                     struct.unpack("<6f", struct.pack("<6f", *endpoint["position"], *endpoint["rotation"])))
                    destination, *transform, teleport_flags = struct.unpack("<I6fI", fields[b"XTEL"])
                    self.assertEqual(destination, int(partner["reference_id"], 16))
                    self.assertEqual(teleport_flags, 0)
                    for actual, expected in zip(transform, partner["arrival"]):
                        self.assertAlmostEqual(actual, expected, places=6)
                    cell = cells[endpoint["cell"]]
                    self.assertEqual(cell["worldspace"], endpoint["worldspace"])
                    frame_kind, frame_flags, frame_fields, frame_placement = by_id[int(endpoint["surround_reference_id"], 16)]
                    self.assertEqual(frame_kind, b"REFR")
                    self.assertEqual(frame_flags, 0)
                    self.assertEqual(struct.unpack("<I", frame_fields[b"NAME"])[0], fixture.DOOR_SURROUND)
                    self.assertEqual(frame_fields[b"DATA"], fields[b"DATA"])
                    self.assertEqual([item[0] for item in frame_placement], [0, 1, 4, 5, 6, 9])
                    self.assertEqual(frame_placement[-1][1], int(cell["form_id"], 16))
                    land = next(payload for kind, _, _, payload, placement in records
                                if kind == b"LAND" and placement[-1][1] == int(cell["form_id"], 16))
                    heights = decode_heights(land)
                    for position in (endpoint["position"], endpoint["arrival"]):
                        x, y, z = position[:3]
                        self.assertEqual([int(x // 4096), int(y // 4096)], cell["coordinates"])
                        sample = int(y % 4096 // 128) * 33 + int(x % 4096 // 128)
                        self.assertEqual(z, heights[sample])
                    offset = [endpoint["arrival"][i] - endpoint["position"][i] for i in range(2)]
                    self.assertGreater(math.hypot(*offset), 128)
                    heading = endpoint["arrival"][5]
                    self.assertGreater(offset[0] * math.sin(heading) + offset[1] * math.cos(heading), 0)

    def test_cli_output_validation_and_overwrite_refusal(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "fixture"
            script = Path(fixture.__file__)
            for arguments in (("--size", "1"), ("--size", "17"), ("--cell-name", "bad name")):
                result = subprocess.run([sys.executable, str(script), "--output", str(output), *arguments],
                                        capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())
            command = [sys.executable, str(script), "--output", str(output)]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = json.loads((output / "fixture.json").read_text())
            self.assertIn(manifest["coc"], result.stdout)
            self.assertIn("A -> B", result.stdout)
            self.assertIn("B -> A", result.stdout)
            self.assertEqual((output / "load-order.txt").read_text().splitlines(),
                             ["Skyrim.esm", str(output.resolve() / fixture.PLUGIN_NAME)])
            original = {path.name: path.read_bytes() for path in output.iterdir()}
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(original, {path.name: path.read_bytes() for path in output.iterdir()})

    @unittest.skipUnless(EXE, "Pass the desktop executable to run the CLI navigation regression")
    def test_generation_links_every_shared_border_and_door(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = fixture.create_test_world(root / "fixture")
            # A project-authored DOOR base lets this terrain-only run resolve the
            # endpoint type and bounds without requiring any installed game assets.
            door = fixture.text_subrecord("EDID", "SyntheticTeleportDoor")
            door += fixture.subrecord("OBND", struct.pack("<6h", -48, -8, 0, 48, 8, 192))
            door += fixture.subrecord("FNAM", bytes(1))
            header = fixture.subrecord("HEDR", struct.pack("<fII", 1.7, 2, fixture.TELEPORT_DOOR + 1))
            master = fixture.record("TES4", 0, header, 1)
            master += fixture.group(int.from_bytes(b"DOOR", "little"), 0,
                                    fixture.record("DOOR", fixture.TELEPORT_DOOR, door))
            (root / "Skyrim.esm").write_bytes(master)
            source = (root / "fixture" / fixture.PLUGIN_NAME).read_bytes()
            output = root / "generated"
            result = subprocess.run([str(EXE), "--data", str(root), "--load-order",
                                     str(root / "fixture/load-order.txt"), "--rebuild-plugin", fixture.PLUGIN_NAME,
                                     "--terrain-only", "--generate-plugin", "--batch-output", "full",
                                     "--output", str(output), "--asset-cache", str(root / "cache")],
                                    capture_output=True, text=True, timeout=180)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = json.loads((output / "batch-report.json").read_text())
            self.assertEqual(report["status"], "complete")
            generated = [cell for cell in report["cells"] if cell["status"] == "generated"]
            self.assertEqual(len(generated), len(manifest["cells"]))
            candidates = {cell["form_id"]: json.loads((output / "cells" / cell["form_id"] / "candidate-navm.json").read_text())
                          for cell in generated}
            names = {cell["form_id"]: cell["editor_id"] for cell in manifest["cells"]}
            actual_seams = set()
            for cell_id, candidate in candidates.items():
                self.assertTrue(candidate["topology"]["valid"])
                self.assertGreater(len(candidate["polygons"]), 0)
                for link in candidate["border_links"]:
                    neighbor_id = link["generated_neighbor_cell"]
                    neighbor = candidates[neighbor_id]
                    self.assertTrue(any(back["generated_neighbor_cell"] == cell_id
                                        and back["polygon"] == link["neighbor_polygon"]
                                        and back["edge"] == link["neighbor_edge"]
                                        and back["neighbor_polygon"] == link["polygon"]
                                        and back["neighbor_edge"] == link["edge"]
                                        for back in neighbor["border_links"]))
                    face = candidate["polygons"][link["polygon"]]["vertices"]
                    target = neighbor["polygons"][link["neighbor_polygon"]]["vertices"]
                    self.assertEqual(candidate["vertices"][face[link["edge"]]],
                                     neighbor["vertices"][target[(link["neighbor_edge"] + 1) % 3]])
                    self.assertEqual(candidate["vertices"][face[(link["edge"] + 1) % 3]],
                                     neighbor["vertices"][target[link["neighbor_edge"]]])
                    actual_seams.add(tuple(sorted((names[cell_id], names[neighbor_id]))))
            self.assertEqual(actual_seams, {tuple(sorted(pair)) for pair in manifest["seams"]})
            for endpoint in manifest["door_route"]:
                cell_id = next(cell["form_id"] for cell in manifest["cells"]
                               if cell["editor_id"] == endpoint["cell"])
                exits = candidates[cell_id]["exits"]
                self.assertEqual([exit["reference_id"] for exit in exits], [endpoint["reference_id"]])
                self.assertIsNotNone(exits[0]["polygon"])
            emitted, _ = read_records((output / "generated-navmesh.esp").read_bytes())
            parents = {(kind, form_id): (flags, payload) for kind, form_id, flags, payload, _ in emitted
                       if kind in (b"WRLD", b"CELL")}
            self.assertEqual(sum(kind == b"WRLD" for kind, _ in parents), 2)
            self.assertEqual(sum(kind == b"CELL" for kind, _ in parents), 2)
            for (kind, _), (flags, payload) in parents.items():
                self.assertEqual(flags, 0x4400 if kind == b"CELL" else 0x4000)
                self.assertEqual(payload, b"")
            self.assertEqual(sum(entry[0] == b"NAVM" for entry in emitted), len(manifest["cells"]))
            self.assertIn(b"NAVI", {entry[0] for entry in emitted})
            # Decode the actual ESP door tables and reciprocal XNDP associations,
            # independently of candidate JSON and the production plugin reader.
            references = {form_id: (flags, dict(subrecords(payload)), placement)
                          for kind, form_id, flags, payload, placement in emitted if kind == b"REFR"}
            expected_doors = {int(endpoint["reference_id"], 16) for endpoint in manifest["door_route"]}
            self.assertEqual(set(references), expected_doors)
            portals = {}
            for kind, form_id, _, payload, placement in emitted:
                if kind != b"NAVM":
                    continue
                mesh = dict(subrecords(payload))[b"NVNM"]
                self.assertEqual(struct.unpack_from("<II", mesh), (12, 0xA5E9A03C))
                vertices = struct.unpack_from("<I", mesh, 16)[0]
                triangle_at = 20 + vertices * 12
                triangles = struct.unpack_from("<I", mesh, triangle_at)[0]
                links_at = triangle_at + 4 + triangles * 16
                links = struct.unpack_from("<I", mesh, links_at)[0]
                door_at = links_at + 4 + links * 10
                doors = struct.unpack_from("<I", mesh, door_at)[0]
                for index in range(doors):
                    triangle, crc, reference = struct.unpack_from("<HII", mesh, door_at + 4 + index * 10)
                    self.assertEqual(crc, 0xE48B73F3)
                    self.assertLess(triangle, triangles)
                    self.assertTrue(struct.unpack_from("<H", mesh, triangle_at + 4 + triangle * 16 + 12)[0] & 0x400)
                    self.assertNotIn(reference, portals)
                    portals[reference] = (form_id, triangle, placement[-1][1])
                    self.assertEqual(struct.unpack("<IHH", references[reference][1][b"XNDP"]),
                                     (form_id, triangle, 0))
            self.assertEqual(set(portals), expected_doors)
            original_records, _ = read_records(source)
            for kind, form_id, flags, payload, placement in original_records:
                if form_id not in expected_doors:
                    continue
                actual_flags, actual_fields, actual_placement = references[form_id]
                emitted_payload = next(data for record_kind, identity, _, data, _ in emitted
                                       if record_kind == b"REFR" and identity == form_id)
                self.assertEqual(subrecords(emitted_payload)[-1][0], b"DATA")
                self.assertEqual(actual_flags, flags)
                self.assertEqual(actual_placement, placement)
                self.assertEqual({key: value for key, value in actual_fields.items() if key != b"XNDP"},
                                 dict(subrecords(payload)))
                endpoint = next(point for point in manifest["door_route"] if int(point["reference_id"], 16) == form_id)
                owner = next(cell["form_id"] for cell in manifest["cells"] if cell["editor_id"] == endpoint["cell"])
                self.assertEqual(portals[form_id][2], int(owner, 16))
            self.assertEqual(source, (root / "fixture" / fixture.PLUGIN_NAME).read_bytes())
            # Loading the generated patch must retain the full source parent data.
            order = root / "with-patch.txt"
            order.write_text((root / "fixture/load-order.txt").read_text()
                             + str(output / "generated-navmesh.esp") + "\n")
            reloaded = subprocess.run([str(EXE), "--data", str(root), "--load-order", str(order),
                                       "--list-cells", "--output", str(root / "catalog")],
                                      capture_output=True, text=True, timeout=90)
            self.assertEqual(reloaded.returncode, 0, reloaded.stdout + reloaded.stderr)
            catalog = json.loads((root / "catalog/cells.json").read_text())
            catalog_cells = catalog["cells"]
            for endpoint in manifest["door_route"]:
                persistent = next(cell for cell in catalog_cells
                                  if cell["editor_id"] == endpoint["worldspace"] + "Persistent")
                self.assertFalse(persistent["interior"])
                self.assertEqual(persistent["coordinates"], [0, 0])


if __name__ == "__main__":
    unittest.main()
