"""Binary-layout and CLI stitching regressions for the playable world fixture."""

import json
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
                            self.assertNotIn(b"XCLC", fields)
                            self.assertEqual([item[0] for item in ancestors], [0, 1])
                            continue
                        x, y, _ = struct.unpack("<iiI", fields[b"XCLC"])
                        cells[form_id] = (x, y)
                        self.assertEqual([item[0] for item in ancestors], [0, 1, 4, 5])
                        self.assertEqual(struct.unpack("<hh", struct.pack("<I", ancestors[2][1])), (y // 32, x // 32))
                        self.assertEqual(struct.unpack("<hh", struct.pack("<I", ancestors[3][1])), (y // 8, x // 8))
                        self.assertFalse(struct.unpack("<H", fields[b"DATA"])[0] & 3)
                    elif kind in (b"LAND", b"REFR"):
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
                            self.assertEqual(struct.unpack("<I", fields[b"NAME"])[0], fixture.COC_MARKER)
                            markers[owner] = struct.unpack("<6f", fields[b"DATA"])
                self.assertEqual(len(cells), size * size)
                self.assertEqual(cells.keys(), terrain.keys())
                self.assertEqual(cells.keys(), markers.keys())
                by_coordinates = {coordinates: terrain[form_id] for form_id, coordinates in cells.items()}
                for form_id, (x, y) in cells.items():
                    heights = terrain[form_id]
                    spawn = markers[form_id]
                    self.assertEqual(spawn[:2], (x * 4096 + 2048, y * 4096 + 2048))
                    self.assertGreater(spawn[2], heights[16 * 33 + 16])
                    if (x + 1, y) in by_coordinates:
                        self.assertEqual(heights[32::33], by_coordinates[(x + 1, y)][::33])
                    if (x, y + 1) in by_coordinates:
                        self.assertEqual(heights[-33:], by_coordinates[(x, y + 1)][:33])
                self.assertEqual(len(manifest["seams"]), 2 * size * (size - 1))
                self.assertEqual(manifest["coc"], "coc SyntheticCell")

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
            self.assertEqual((output / "load-order.txt").read_text().splitlines(),
                             ["Skyrim.esm", str(output.resolve() / fixture.PLUGIN_NAME)])
            original = {path.name: path.read_bytes() for path in output.iterdir()}
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(original, {path.name: path.read_bytes() for path in output.iterdir()})

    @unittest.skipUnless(EXE, "Pass the desktop executable to run the CLI stitching regression")
    def test_generation_links_every_shared_border(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = fixture.create_test_world(root / "fixture")
            # Only the master identity is needed for this terrain-only run. The
            # real game supplies the referenced texture, climate, and marker.
            master = b"TES4" + struct.pack("<IIIIHH", 18, 1, 0, 0, 44, 0)
            master += b"HEDR" + struct.pack("<HfII", 12, 1.7, 0, 0x800)
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
            emitted, _ = read_records((output / "generated-navmesh.esp").read_bytes())
            self.assertEqual(sum(entry[0] == b"NAVM" for entry in emitted), len(manifest["cells"]))
            self.assertIn(b"NAVI", {entry[0] for entry in emitted})
            self.assertEqual(source, (root / "fixture" / fixture.PLUGIN_NAME).read_bytes())


if __name__ == "__main__":
    unittest.main()
