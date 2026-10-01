"""CLI regressions using redistributable synthetic plugins, without opening the UI."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

EXE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else Path("build/windows/x64/releasedbg/navmesh-offline.exe").resolve()


def sub(kind, data):
    return kind.encode() + struct.pack("<H", len(data)) + data


def record(kind, form, data, flags=0):
    return kind.encode() + struct.pack("<IIIII", len(data), flags, form, 0, 44) + data


def group(label, kind, data):
    return b"GRUP" + struct.pack("<IIIII", len(data) + 24, label, kind, 0, 0) + data


def land():
    return sub("VHGT", struct.pack("<f", 0) + bytes(33 * 33 + 3))


def navm(x, y=0):
    vertices = [(x * 4096, y * 4096, 0), ((x + 1) * 4096, y * 4096, 0),
                ((x + 1) * 4096, (y + 1) * 4096, 0), (x * 4096, (y + 1) * 4096, 0)]
    body = struct.pack("<IIIhhI", 12, 0, 0x400, y, x, 4)
    body += b"".join(struct.pack("<fff", *vertex) for vertex in vertices)
    body += struct.pack("<I", 2)
    body += struct.pack("<8H", 0, 1, 2, 65535, 65535, 1, 0, 0)
    body += struct.pack("<8H", 0, 2, 3, 0, 65535, 65535, 0, 0)
    body += struct.pack("<4I8fI2H", 0, 0, 0, 1, *([0] * 8), 2, 0, 1)
    return sub("NVNM", body)


class BatchRebuild(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="navmesh-batch-cli-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        cells = b""
        for x in range(3):
            cell = 0x100 + x
            payload = sub("EDID", f"Exterior{x}\0".encode()) + sub("XCLC", struct.pack("<ii", x, 0))
            children = record("LAND", 0x300 + x, land()) + record("NAVM", 0x200 + x, navm(x))
            cells += record("CELL", cell, payload) + group(cell, 6, group(cell, 9, children))
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        world = record("WRLD", 0x400, sub("EDID", b"FixtureWorld\0")) + group(0x400, 1, cells)
        (self.root / "Baseline.esm").write_bytes(record("TES4", 0, header, 1) + group(int.from_bytes(b"WRLD", "little"), 0, world))
        # Only LAND changes; the winning CELL still belongs to the baseline.
        patch = group(0x400, 1, group(0x100, 6, group(0x100, 9, record("LAND", 0x300, land()))))
        patch_header = header + sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, patch_header) + group(int.from_bytes(b"WRLD", "little"), 0, patch))
        (self.root / "plugins.txt").write_text("Baseline.esm\nPatch.esp\n")

    def run_cli(self, *args, output="output", code=0, terrain_only=True):
        completed = subprocess.run([str(EXE), "--data", str(self.root), "--load-order", str(self.root / "plugins.txt"),
                                    "--output", str(self.root / output),
                                    *(["--terrain-only"] if terrain_only else []), *args],
                                   capture_output=True, text=True, timeout=90)
        self.assertEqual(completed.returncode, code, completed.stdout + completed.stderr)
        return self.root / output

    def test_plugin_scope_and_single_patch(self):
        output = self.run_cli("--rebuild-plugin", "pAtCh.EsP", "--generate-plugin")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertEqual(report["status"], "complete")
        self.assertEqual(report["selected_cells"], 2)
        self.assertEqual([cell["status"] for cell in report["cells"]], ["generated", "generated"])
        self.assertGreater(report["geometry_cache_hits"], 0)
        self.assertEqual(report["geometry_cells_extracted"], 3)
        self.assertTrue((output / "generated-navmesh.esp").exists())
        for cell in report["cells"]:
            candidate = json.loads((output / "cells" / cell["form_id"] / "candidate-navm.json").read_text())
            self.assertTrue(candidate["topology"]["valid"])
            self.assertGreater(len(candidate["polygons"]), 0)
        # A written patch can be read through the normal CLI and supplies all NAVMs.
        with (self.root / "plugins.txt").open("a") as manifest:
            manifest.write(str(output / "generated-navmesh.esp") + "\n")
        self.run_cli("--list-cells", output="read-back")

    def test_load_order_scope(self):
        output = self.run_cli("--rebuild-load-order")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertEqual(report["selected_cells"], 2)
        self.assertEqual(report["scope"], "load_order")
        self.assertIn("metadata", report)

    def test_baseline_only_has_no_changes(self):
        (self.root / "plugins.txt").write_text("Baseline.esm\n")
        output = self.run_cli("--rebuild-load-order", "--generate-plugin")
        self.assertEqual(json.loads((output / "batch-report.json").read_text())["selected_cells"], 0)
        self.assertFalse((output / "generated-navmesh.esp").exists())

    def test_existing_patch_is_preserved(self):
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin")
        plugin = (output / "generated-navmesh.esp").read_bytes()
        self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", code=2)
        self.assertEqual((output / "generated-navmesh.esp").read_bytes(), plugin)
        self.assertEqual(json.loads((output / "batch-report.json").read_text())["status"], "failed")

    def test_invalid_selection(self):
        self.run_cli("--rebuild-plugin", "Inactive.esp", code=1)
        self.run_cli("--rebuild-load-order", "--cell-formid", "100", code=1)
        self.run_cli("--rebuild-plugin", "Patch.esp", "--rebuild-load-order", code=1)
        self.run_cli("--rebuild-load-order", "--neighboring-cell-radius", "-1", code=1)

    def test_single_cell_keeps_geometry_halo(self):
        output = self.run_cli("--cell-formid", "100", "--generate-plugin", "--neighboring-cell-radius", "1")
        candidate = json.loads((output / "candidate-navm.json").read_text())
        self.assertGreater(len(candidate["polygons"]), 0)
        self.assertTrue(all(0 <= vertex[0] <= 4096 for vertex in candidate["vertices"]))

    def test_scene_neighborhood_excludes_distant_geometry_supplier_cells(self):
        # Distant references can supply intersecting models without adding their
        # source CELL's terrain or authored navigation to the inspection scene.
        coordinates = [(x, y) for x in range(-1, 2) for y in range(-1, 2)] + [(5, 0), (10, 0)]
        cells = b""
        for index, (x, y) in enumerate(coordinates):
            cell = 0x100 + index
            payload = sub("EDID", f"SceneCell{index}\0".encode()) + sub("XCLC", struct.pack("<ii", x, y))
            children = record("LAND", 0x300 + index, land()) + record("NAVM", 0x200 + index, navm(x, y))
            if index >= 9:
                reference = sub("NAME", struct.pack("<I", 0x500 + index - 9))
                reference += sub("DATA", struct.pack("<6f", x * 4096 + 100, 100, 0, 0, 0, 0))
                children += record("REFR", 0x600 + index, reference)
            cells += record("CELL", cell, payload) + group(cell, 6, group(cell, 9, children))
        bounded = sub("MODL", b"Oversized.nif\0") + sub("OBND", struct.pack("<6h", -30000, -100, -100, 30000, 100, 100))
        unbounded = sub("MODL", b"UnknownBounds.nif\0")
        models = group(int.from_bytes(b"STAT", "little"), 0,
                       record("STAT", 0x500, bounded) + record("STAT", 0x501, unbounded))
        world = record("WRLD", 0x400, sub("EDID", b"SceneWorld\0")) + group(0x400, 1, cells)
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        (self.root / "Baseline.esm").write_bytes(record("TES4", 0, header, 1) + models
                                               + group(int.from_bytes(b"WRLD", "little"), 0, world))
        (self.root / "plugins.txt").write_text("Baseline.esm\n")
        for terrain_only, radius, generating in [(False, 1, False), (True, 1, True),
                                                (False, 0, False), (False, 0, True)]:
            with self.subTest(terrain_only=terrain_only, radius=radius, generating=generating):
                output = self.run_cli("--cell-formid", "104", "--neighboring-cell-radius", str(radius),
                                      *(["--generate-candidate"] if generating else []),
                                      output=f"scene-{terrain_only}-{radius}-{generating}", terrain_only=terrain_only)
                report = json.loads((output / "report.json").read_text())
                expected = set(range(9)) if radius or generating else {4}
                self.assertEqual(report["metadata"]["source_coverage"]["terrain_land_decoded"], len(expected))
                glb = (output / "scene.glb").read_bytes()
                chunk_size = struct.unpack_from("<I", glb, 12)[0]
                scene = json.loads(glb[20:20 + chunk_size])
                names = [node["name"] for node in scene["nodes"]]
                navmesh_ids = {name.split()[2].rstrip(":") for name in names if name.startswith("Existing NAVM ")}
                self.assertEqual(navmesh_ids, {f"{0x200 + index:08X}" for index in expected})


if __name__ == "__main__":
    unittest.main()
