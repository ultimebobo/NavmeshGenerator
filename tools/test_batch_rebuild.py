"""CLI regressions using redistributable synthetic plugins, without opening the UI."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

EXE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else Path("build/windows/x64/releasedbg/NavmeshGenerator.exe").resolve()


def sub(kind, data):
    return kind.encode() + struct.pack("<H", len(data)) + data


def record(kind, form, data, flags=0):
    return kind.encode() + struct.pack("<IIIII", len(data), flags, form, 0, 44) + data


def group(label, kind, data):
    return b"GRUP" + struct.pack("<IIIII", len(data) + 24, label, kind, 0, 0) + data


def land(offset=0):
    return sub("VHGT", struct.pack("<f", offset) + bytes(33 * 33 + 3))


def navm(x, y=0, flags=(0, 0)):
    vertices = [(x * 4096, y * 4096, 0), ((x + 1) * 4096, y * 4096, 0),
                ((x + 1) * 4096, (y + 1) * 4096, 0), (x * 4096, (y + 1) * 4096, 0)]
    body = struct.pack("<IIIhhI", 12, 0, 0x400, y, x, 4)
    body += b"".join(struct.pack("<fff", *vertex) for vertex in vertices)
    body += struct.pack("<I", 2)
    body += struct.pack("<8H", 0, 1, 2, 65535, 65535, 1, flags[0], 0)
    body += struct.pack("<8H", 0, 2, 3, 0, 65535, 65535, flags[1], 0)
    body += struct.pack("<4I8fI2H", 0, 0, 0, 1, *([0] * 8), 2, 0, 1)
    return sub("NVNM", body)


def read_records(path):
    """Inspect emitted record identities and payloads independently of the C++ reader."""
    data = path.read_bytes()
    records = []

    def visit(start, end):
        while start < end:
            kind = data[start:start + 4].decode()
            size = struct.unpack_from("<I", data, start + 4)[0]
            if kind == "GRUP":
                visit(start + 24, start + size)
                start += size
            else:
                flags, form = struct.unpack_from("<II", data, start + 8)
                records.append((kind, form, flags, data[start + 24:start + 24 + size]))
                start += 24 + size
        assert start == end

    visit(0, len(data))
    return records


def navm_geometry(payload):
    """Read the versioned NVNM geometry prefix, allowing appended portal sections."""
    assert payload[:4] == b"NVNM"
    body = payload[6:]
    vertices = struct.unpack_from("<I", body, 16)[0]
    triangle_at = 20 + vertices * 12
    triangles = struct.unpack_from("<I", body, triangle_at)[0]
    positions = body[20:triangle_at]
    # Portal flags and neighbor indices may change; vertex indices must stay authored.
    faces = [body[triangle_at + 4 + index * 16:triangle_at + 10 + index * 16]
             for index in range(triangles)]
    return positions, faces


class BatchRebuild(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="navmesh-batch-cli-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.write_baseline()
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        # Only LAND changes; the winning CELL still belongs to the baseline.
        patch = group(0x400, 1, group(0x100, 6, group(0x100, 9, record("LAND", 0x300, land(0.001)))))
        patch_header = header + sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, patch_header) + group(int.from_bytes(b"WRLD", "little"), 0, patch))
        (self.root / "plugins.txt").write_text("Baseline.esm\nPatch.esp\n")

    def write_baseline(self, navmeshes=None, references=None, water=None, world_data=b""):
        if navmeshes is None:
            navmeshes = {x: navm(x) for x in range(3)}
        cells = b""
        for x in range(3):
            cell = 0x100 + x
            payload = sub("EDID", f"Exterior{x}\0".encode()) + sub("XCLC", struct.pack("<ii", x, 0))
            payload += (water or {}).get(x, b"")
            children = record("LAND", 0x300 + x, land())
            if x in navmeshes:
                children += record("NAVM", 0x200 + x, navmeshes[x])
            children += (references or {}).get(x, b"")
            cells += record("CELL", cell, payload) + group(cell, 6, group(cell, 9, children))
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        world = record("WRLD", 0x400, sub("EDID", b"FixtureWorld\0") + world_data) + group(0x400, 1, cells)
        (self.root / "Baseline.esm").write_bytes(record("TES4", 0, header, 1) + group(int.from_bytes(b"WRLD", "little"), 0, world))

    def run_cli(self, *args, output="output", code=0, terrain_only=True):
        completed = subprocess.run([str(EXE), "--data", str(self.root), "--load-order", str(self.root / "plugins.txt"),
                                    "--output", str(self.root / output), "--batch-output", "full",
                                    "--asset-cache", str(self.root / "cache"),
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

    def test_triangle_tagging_and_disable(self):
        self.write_baseline(navmeshes={x: navm(x, flags=(0x40, 0)) for x in range(3)},
                            water={0: sub("DATA", b"\x02\x00") + sub("XCLW", struct.pack("<f", 10))})
        def check(candidate, enabled):
            self.assertEqual(candidate["triangle_tagging"], enabled)
            polygons = candidate["polygons"]
            self.assertTrue(polygons)
            self.assertEqual(any(p["water"] for p in polygons), enabled)
            self.assertEqual(any(p["preferred_path"] for p in polygons), enabled)
            for p in polygons:
                self.assertEqual(p["water"], bool(p["flags"] & 0x200))
                self.assertEqual(p["preferred_path"], bool(p["flags"] & 0x40))
        single = self.run_cli("--cell-formid", "100", "--generate-plugin", output="tagged-cell")
        tagged_candidate = json.loads((single / "candidate-navm.json").read_text())
        check(tagged_candidate, True)
        glb = (single / "scene.glb").read_bytes()
        glb_json_size = struct.unpack_from("<I", glb, 12)[0]
        scene = json.loads(glb[20:20 + glb_json_size])
        self.assertTrue(any(mesh["name"] == "Candidate NAVM: water" for mesh in scene["meshes"]))
        self.assertTrue(any(mesh["name"] == "Candidate NAVM: water_preferred_path" for mesh in scene["meshes"]))
        # Independent NVNM decoding proves actual plugin flags, beyond inspection JSON.
        for kind, form, _, payload in read_records(single / "generated-navmesh.esp"):
            if kind == "NAVM" and form & 0xffffff == 0x200:
                body = payload[6:]
                triangle_at = 20 + struct.unpack_from("<I", body, 16)[0] * 12
                count = struct.unpack_from("<I", body, triangle_at)[0]
                flags = [struct.unpack_from("<H", body, triangle_at + 4 + i * 16 + 12)[0] for i in range(count)]
                self.assertTrue(any(f & 0x200 for f in flags))
                self.assertTrue(any(f & 0x40 for f in flags))
                break
        else:
            self.fail("Expected generated primary NAVM")
        disabled = self.run_cli("--cell-formid", "100", "--generate-plugin", "--no-triangle-tagging",
                                output="untagged-cell")
        untagged_candidate = json.loads((disabled / "candidate-navm.json").read_text())
        check(untagged_candidate, False)
        self.assertEqual(tagged_candidate["vertices"], untagged_candidate["vertices"])
        for tagged, untagged in zip(tagged_candidate["polygons"], untagged_candidate["polygons"]):
            self.assertEqual(tagged["vertices"], untagged["vertices"])
            self.assertEqual(tagged["neighbors"], untagged["neighbors"])
        for kind, _, _, payload in read_records(disabled / "generated-navmesh.esp"):
            if kind == "NAVM":
                body = payload[6:]
                triangle_at = 20 + struct.unpack_from("<I", body, 16)[0] * 12
                count = struct.unpack_from("<I", body, triangle_at)[0]
                self.assertFalse(any(struct.unpack_from("<H", body, triangle_at + 4 + i * 16 + 12)[0] & 0x240
                                     for i in range(count)))
        batch = self.run_cli("--rebuild-plugin", "Patch.esp", output="tagged-batch")
        check(json.loads((batch / "cells/00000100/candidate-navm.json").read_text()), True)
        disabled_batch = self.run_cli("--rebuild-plugin", "Patch.esp", "--no-triangle-tagging",
                                     output="untagged-batch")
        check(json.loads((disabled_batch / "cells/00000100/candidate-navm.json").read_text()), False)
        report = json.loads((disabled_batch / "batch-report.json").read_text())
        self.assertFalse(any(cell["candidate_reused"] for cell in report["cells"]))

    def test_effective_water_height(self):
        variants = [(b"", True), (sub("XCLW", struct.pack("<f", -2147483648)), True),
                    (sub("XCLW", struct.pack("<f", -1)), False),
                    (sub("XCLW", struct.pack("<f", float("nan"))), False),
                    (sub("XCLW", b"\0"), False)]
        for index, (height, expected) in enumerate(variants):
            with self.subTest(index=index):
                self.write_baseline(water={0: sub("DATA", b"\x02\x00") + height},
                                    world_data=sub("DNAM", struct.pack("<ff", 0, 10)))
                output = self.run_cli("--cell-formid", "100", "--generate-candidate", output=f"water-{index}")
                candidate = json.loads((output / "candidate-navm.json").read_text())
                self.assertEqual(any(p["water"] for p in candidate["polygons"]), expected)
        self.write_baseline(water={0: sub("XCLW", struct.pack("<f", 10))})
        output = self.run_cli("--cell-formid", "100", "--generate-candidate", output="dry-cell")
        self.assertFalse(any(p["water"] for p in json.loads((output / "candidate-navm.json").read_text())["polygons"]))

    def test_parent_world_water(self):
        parent_link = sub("WNAM", struct.pack("<I", 0x401)) + sub("PNAM", struct.pack("<H", 8))
        self.write_baseline(water={0: sub("DATA", b"\x02\x00")},
                            world_data=parent_link + sub("DNAM", struct.pack("<ff", 0, -1)))
        parent = record("WRLD", 0x401, sub("EDID", b"ParentWorld\0") + sub("DNAM", struct.pack("<ff", 0, 10)))
        with (self.root / "Baseline.esm").open("ab") as plugin:
            plugin.write(group(int.from_bytes(b"WRLD", "little"), 0, parent))
        output = self.run_cli("--cell-formid", "100", "--generate-candidate", output="parent-water")
        self.assertTrue(any(p["water"] for p in json.loads((output / "candidate-navm.json").read_text())["polygons"]))

    def test_generated_polygons_reach_door_or_border(self):
        """Check actual adjacency after border stitching in both shared run paths."""
        single = self.run_cli("--cell-formid", "100", "--generate-candidate", output="reachable-cell")
        batch = self.run_cli("--rebuild-plugin", "Patch.esp", output="reachable-batch")
        candidates = [(single / "candidate-navm.json", 0)]
        candidates.extend((batch / "cells" / f"{0x100 + x:08X}" / "candidate-navm.json", x) for x in range(2))
        for path, cell_x in candidates:
            with self.subTest(path=path):
                candidate = json.loads(path.read_text())
                polygons = candidate["polygons"]
                self.assertGreater(len(polygons), 0)
                pending = [door["polygon"] for door in candidate["exits"] if door["polygon"] is not None]
                pending.extend(link["polygon"] for link in candidate["border_links"])
                for index, polygon in enumerate(polygons):
                    for edge, neighbor in enumerate(polygon["neighbors"]):
                        if neighbor is not None:
                            continue
                        a = candidate["vertices"][polygon["vertices"][edge]]
                        b = candidate["vertices"][polygon["vertices"][(edge + 1) % 3]]
                        if any(abs(a[axis] - limit) <= candidate["profile"]["weld_tolerance"] and
                               abs(b[axis] - limit) <= candidate["profile"]["weld_tolerance"]
                               for axis, limit in ((0, cell_x * 4096), (0, (cell_x + 1) * 4096), (1, 0), (1, 4096))):
                            pending.append(index)
                reachable = set()
                while pending:
                    index = pending.pop()
                    if index in reachable:
                        continue
                    self.assertLess(index, len(polygons))
                    reachable.add(index)
                    pending.extend(neighbor for neighbor in polygons[index]["neighbors"] if neighbor is not None)
                self.assertEqual(reachable, set(range(len(polygons))))

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

    def test_plugin_copy_preserves_records_header_and_source(self):
        selected = self.root / "Patch.esp"
        original = selected.read_bytes()
        header_size = struct.unpack_from("<I", original, 4)[0]
        header = original[24:24 + header_size] + sub("CNAM", b"Fixture author\0") + sub("ZZZZ", b"opaque header")
        opaque_payload = sub("XXXX", struct.pack("<I", 70000)) + b"DATA\0\0" + b"x" * 70000
        opaque = record("QUST", 0x01000850, struct.pack("<I", len(opaque_payload)) + zlib.compress(opaque_payload), 0x40000)
        # TES4 localization and master flags must survive even for an ESP filename.
        original = record("TES4", 0, header, 0x81) + original[24 + header_size:]
        original += group(int.from_bytes(b"QUST", "little"), 0, opaque)
        selected.write_bytes(original)
        output = self.run_cli("--rebuild-plugin", "pAtCh.EsP", "--copy-plugin")
        copied = output / "Patch.esp"
        self.assertTrue(copied.exists())
        self.assertFalse((output / "generated-navmesh.esp").exists())
        self.assertEqual(selected.read_bytes(), original)
        self.assertIn(opaque, copied.read_bytes())
        emitted = read_records(copied)
        self.assertEqual([item for item in emitted if item[0] not in ("TES4", "NAVM")],
                         [item for item in read_records(selected) if item[0] != "TES4"])
        old_header = read_records(selected)[0]
        new_header = emitted[0]
        self.assertEqual(new_header[:3], old_header[:3])
        # HEDR accounting changes and masters register generated overrides in ONAM.
        self.assertEqual(new_header[3][:10], old_header[3][:10])
        self.assertEqual(new_header[3][18:len(old_header[3])], old_header[3][18:])
        onam = new_header[3][len(old_header[3]):]
        self.assertEqual(onam[:4], b"ONAM")
        self.assertEqual(set(struct.unpack(f"<{(len(onam) - 6) // 4}I", onam[6:])),
                         {form for kind, form, flags, payload in emitted if kind == "NAVM" and form >> 24 == 0})
        self.assertEqual(struct.unpack_from("<I", new_header[3], 14)[0], 0x851)
        self.assertNotIn(sub("MAST", b"Patch.esp\0"), new_header[3])
        self.assertTrue(json.loads((output / "batch-report.json").read_text())["copy_plugin"])
        # Read the copy as a replacement under its original identity.
        (self.root / "plugins.txt").write_text(f"Baseline.esm\n{copied}\n")
        self.run_cli("--list-cells", output="copy-read-back")

    def test_plugin_copy_new_ids_avoid_unindexed_records(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        selected = self.root / "Patch.esp"
        unknown = record("QUST", 0x01000850, sub("EDID", b"OpaqueIdentity\0"))
        selected.write_bytes(selected.read_bytes() + group(int.from_bytes(b"QUST", "little"), 0, unknown))
        original = selected.read_bytes()
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", "--skip-existing-navmesh")
        copied = output / "Patch.esp"
        records = read_records(copied)
        self.assertIn(0x01000851, [form for kind, form, flags, payload in records if kind == "NAVM"])
        self.assertIn(unknown, copied.read_bytes())
        self.assertEqual(struct.unpack_from("<I", records[0][3], 14)[0], 0x852)
        self.assertEqual(selected.read_bytes(), original)
        self.assertFalse(records[0][2] & 0x200)

    def test_copy_preserves_light_format_and_extension(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        selected = self.root / "Patch.esp"
        source = bytearray(selected.read_bytes())
        struct.pack_into("<I", source, 8, 0x200)
        # HEDR's allocation cursor must also be honored when higher than existing IDs.
        struct.pack_into("<I", source, 38, 0x900)
        for filename, flags in [("Author.esl", 0x200), ("Author.esm", 1)]:
            with self.subTest(filename=filename):
                struct.pack_into("<I", source, 8, flags)
                (self.root / filename).write_bytes(source)
                (self.root / "plugins.txt").write_text(f"Baseline.esm\n{filename}\n")
                output = self.run_cli("--rebuild-plugin", filename, "--copy-plugin", "--skip-existing-navmesh",
                                      output=filename + "-output")
                records = read_records(output / filename)
                self.assertEqual(records[0][2], flags)
                self.assertIn(0x01000900, [form for kind, form, record_flags, payload in records if kind == "NAVM"])

    def test_copy_light_id_exhaustion_fails_without_output(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        selected = self.root / "Patch.esp"
        source = bytearray(selected.read_bytes())
        struct.pack_into("<I", source, 8, 0x200)
        struct.pack_into("<I", source, 38, 0x1000)
        selected.write_bytes(source)
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", "--skip-existing-navmesh", code=2)
        self.assertFalse((output / "Patch.esp").exists())
        self.assertFalse((output / "Patch.esp.tmp").exists())
        self.assertIn("full/light format", json.loads((output / "batch-report.json").read_text())["error"])
        self.assertEqual(selected.read_bytes(), source)

    def test_copy_replaces_owned_navm_once(self):
        self.write_baseline({1: navm(1), 2: navm(2)})
        selected = self.root / "Patch.esp"
        owned = group(int.from_bytes(b"WRLD", "little"), 0,
                      group(0x400, 1, group(0x100, 6, group(0x100, 9, record("NAVM", 0x01000800, navm(0))))))
        selected.write_bytes(selected.read_bytes() + owned)
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin")
        records = read_records(output / "Patch.esp")
        meshes = [item for item in records if item[0] == "NAVM" and item[1] == 0x01000800]
        self.assertEqual(len(meshes), 1)
        self.assertNotEqual(navm_geometry(meshes[0][3]), navm_geometry(navm(0)))

    def test_copy_rejects_dependencies_outside_source_master_table(self):
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        header += sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        children = group(0x100, 6, group(0x100, 9, record("NAVM", 0x01000800, navm(0))))
        (self.root / "Later.esp").write_bytes(record("TES4", 0, header)
                                             + group(int.from_bytes(b"WRLD", "little"), 0, group(0x400, 1, children)))
        (self.root / "plugins.txt").write_text("Baseline.esm\nPatch.esp\nLater.esp\n")
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", code=2)
        self.assertFalse((output / "Patch.esp").exists())
        self.assertFalse((output / "Patch.esp.tmp").exists())
        self.assertEqual(json.loads((output / "batch-report.json").read_text())["status"], "failed")

    def test_copy_refuses_overwriting_source_or_existing_output(self):
        selected = self.root / "Patch.esp"
        source = selected.read_bytes()
        self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", output=".", code=2)
        self.assertEqual(selected.read_bytes(), source)
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin")
        copied = (output / "Patch.esp").read_bytes()
        self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", code=2)
        self.assertEqual((output / "Patch.esp").read_bytes(), copied)
        self.assertEqual(selected.read_bytes(), source)

    def test_copy_requires_plugin_scope(self):
        self.run_cli("--rebuild-load-order", "--copy-plugin", code=1)
        self.run_cli("--cell-formid", "100", "--copy-plugin", code=1)
        self.run_cli("--list-cells", "--copy-plugin", code=1)

    def test_copy_preserves_unmodified_unsupported_navm(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        selected = self.root / "Patch.esp"
        cell = sub("EDID", b"RemoteAuthoredCell\0") + sub("XCLC", struct.pack("<ii", 10, 0))
        unsupported = bytearray(navm(10))
        struct.pack_into("<I", unsupported, 6, 99)
        authored = record("NAVM", 0x01000801, bytes(unsupported))
        remote = record("CELL", 0x01000800, cell) + group(0x01000800, 6, group(0x01000800, 9, authored))
        selected.write_bytes(selected.read_bytes() + group(int.from_bytes(b"WRLD", "little"), 0, group(0x400, 1, remote)))
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", "--skip-existing-navmesh")
        self.assertIn(authored, (output / "Patch.esp").read_bytes())
        records = read_records(output / "Patch.esp")
        self.assertIn(0x01000802, [form for kind, form, flags, payload in records if kind == "NAVM"])

    def test_copy_rejects_malformed_tail_and_duplicate_opaque_ids(self):
        selected = self.root / "Patch.esp"
        original = selected.read_bytes()
        opaque = record("QUST", 0x01000850, b"opaque data")
        for suffix in [b"truncated", group(int.from_bytes(b"QUST", "little"), 0, opaque + opaque),
                       record("TES4", 0, sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800)))]:
            with self.subTest(suffix=suffix[:12]):
                selected.write_bytes(original + suffix)
                output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", code=2)
                self.assertFalse((output / "Patch.esp").exists())
                self.assertFalse((output / "Patch.esp.tmp").exists())
                self.assertEqual(selected.read_bytes(), original + suffix)

    def test_copy_extends_existing_extended_onam_without_self_entries(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        selected = self.root / "Patch.esp"
        original = selected.read_bytes()
        header_size = struct.unpack_from("<I", original, 4)[0]
        forms = struct.pack("<I", 0x300) * 16384
        onam = sub("XXXX", struct.pack("<I", len(forms))) + b"ONAM\0\0" + forms
        header = original[24:24 + header_size] + onam
        selected.write_bytes(record("TES4", 0, header) + original[24 + header_size:])
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--copy-plugin", "--skip-existing-navmesh")
        records = read_records(output / "Patch.esp")
        copied_header = records[0][3]
        prefix_size = header_size
        self.assertEqual(copied_header[prefix_size:prefix_size + 4], b"XXXX")
        data_size = struct.unpack_from("<I", copied_header, prefix_size + 6)[0]
        payload = copied_header[prefix_size + 16:prefix_size + 16 + data_size]
        self.assertTrue(payload.startswith(forms))
        appended = struct.unpack(f"<{(len(payload) - len(forms)) // 4}I", payload[len(forms):])
        expected = {form for kind, form, flags, data in records if kind == "NAVM" and form >> 24 == 0}
        self.assertEqual(set(appended), expected)

    def test_copy_without_masters_retains_source_identity(self):
        output = self.run_cli("--rebuild-plugin", "Baseline.esm", "--copy-plugin")
        records = read_records(output / "Baseline.esm")
        self.assertEqual(records[0][2], 1)
        self.assertNotIn(b"MAST", records[0][3])
        self.assertNotIn(b"ONAM", records[0][3])
        self.assertEqual({form for kind, form, flags, data in records if kind == "NAVM"}, {0x200, 0x201, 0x202})

    def test_single_cell_keeps_geometry_halo(self):
        output = self.run_cli("--cell-formid", "100", "--generate-plugin", "--neighboring-cell-radius", "1")
        candidate = json.loads((output / "candidate-navm.json").read_text())
        self.assertGreater(len(candidate["polygons"]), 0)
        self.assertTrue(all(0 <= vertex[0] <= 4096 for vertex in candidate["vertices"]))

    def test_skip_existing_cells_before_extraction(self):
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--skip-existing-navmesh")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertTrue(report["skip_existing_navmesh"])
        self.assertEqual([cell["status"] for cell in report["cells"]], ["skipped_existing_navm"] * 2)
        self.assertEqual(report["geometry_cells_extracted"], 0)
        self.assertFalse((output / "generated-navmesh.esp").exists())
        single = self.run_cli("--cell-formid", "100", "--generate-plugin", "--skip-existing-navmesh", output="single-skip")
        self.assertEqual(json.loads((single / "generation-report.json").read_text())["status"], "skipped_existing_navm")
        self.assertIn("metadata", json.loads((single / "generation-report.json").read_text()))
        self.assertFalse((single / "candidate-navm.json").exists())

    def test_generate_uncovered_cell_and_preserve_authored_geometry(self):
        self.write_baseline({0: navm(0), 2: navm(2)})
        source = (self.root / "Baseline.esm").read_bytes()
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--skip-existing-navmesh")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertEqual([cell["status"] for cell in report["cells"]], ["skipped_existing_navm", "generated"])
        self.assertGreater(report["generated_polygons"], 0)
        emitted = read_records(output / "generated-navmesh.esp")
        meshes = {form: payload for kind, form, flags, payload in emitted if kind == "NAVM"}
        self.assertIn(0x01000800, meshes)
        self.assertEqual(struct.unpack_from("<Ihh", meshes[0x01000800], 14), (0x400, 0, 1))
        self.assertNotIn(0x201, meshes)
        for form, payload in meshes.items():
            if form in (0x200, 0x202):
                self.assertEqual(navm_geometry(payload), navm_geometry(navm(form - 0x200)))
        self.assertEqual((self.root / "Baseline.esm").read_bytes(), source)
        with (self.root / "plugins.txt").open("a") as manifest:
            manifest.write(str(output / "generated-navmesh.esp") + "\n")
        self.run_cli("--cell-formid", "101", "--generate-plugin", "--skip-existing-navmesh", output="skip-newly-covered")

    def test_single_uncovered_cell_generates_candidate_and_plugin(self):
        self.write_baseline({1: navm(1), 2: navm(2)})
        for operation in ("--generate-candidate", "--generate-plugin"):
            output = self.run_cli("--cell-formid", "100", operation, "--skip-existing-navmesh", output=operation[2:])
            candidate = json.loads((output / "candidate-navm.json").read_text())
            self.assertTrue(candidate["topology"]["valid"])
            self.assertGreater(len(candidate["polygons"]), 0)
            self.assertEqual((output / "generated-navmesh.esp").exists(), operation == "--generate-plugin")

    def test_later_plugin_navm_protects_cell(self):
        self.write_baseline({})
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        header += sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        children = group(0x100, 6, group(0x100, 9, record("NAVM", 0x01000800, navm(0))))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, header)
                                             + group(int.from_bytes(b"WRLD", "little"), 0, group(0x400, 1, children)))
        output = self.run_cli("--cell-formid", "100", "--generate-candidate", "--skip-existing-navmesh")
        self.assertEqual(json.loads((output / "generation-report.json").read_text())["status"], "skipped_existing_navm")

    def test_deleted_navm_identity_protects_cell(self):
        self.write_baseline({})
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        header += sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        children = group(0x100, 6, group(0x100, 9, record("NAVM", 0x01000800, b"", flags=0x20)))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, header)
                                             + group(int.from_bytes(b"WRLD", "little"), 0, group(0x400, 1, children)))
        output = self.run_cli("--cell-formid", "100", "--generate-plugin", "--skip-existing-navmesh")
        self.assertEqual(json.loads((output / "generation-report.json").read_text())["status"], "skipped_existing_navm")

    def test_multiple_uncovered_cells_get_unique_plugin_owned_records(self):
        self.write_baseline({})
        output = self.run_cli("--rebuild-load-order", "--neighboring-cell-radius", "1",
                              "--generate-plugin", "--skip-existing-navmesh")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertTrue(all(cell["status"] == "generated" for cell in report["cells"]))
        emitted = read_records(output / "generated-navmesh.esp")
        ids = [form for kind, form, flags, payload in emitted if kind == "NAVM"]
        self.assertEqual(len(ids), report["selected_cells"])
        self.assertEqual(len(set(ids)), len(ids))
        self.assertTrue(all(form >> 24 == 1 for form in ids))
        header = next(payload for kind, form, flags, payload in emitted if kind == "TES4")
        self.assertEqual(struct.unpack_from("<I", header, 14)[0], 0x800 + len(ids))

    def test_empty_and_unsupported_navm_records_protect_cells(self):
        unsupported = bytearray(navm(0))
        struct.pack_into("<I", unsupported, 6, 99)
        empty = sub("NVNM", struct.pack("<6I", 12, 0, 0x400, 1, 0, 0))
        self.write_baseline({0: bytes(unsupported), 1: empty})
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--skip-existing-navmesh")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertEqual([cell["status"] for cell in report["cells"]], ["skipped_existing_navm"] * 2)
        self.assertEqual(report["geometry_cells_extracted"], 0)

    def test_skip_option_requires_generation_and_resolved_input(self):
        self.run_cli("--cell-formid", "100", "--skip-existing-navmesh", code=1)
        self.run_cli("--list-cells", "--skip-existing-navmesh", code=1)
        completed = subprocess.run([str(EXE), "--plugin", str(self.root / "Baseline.esm"),
                                    "--generate-candidate", "--skip-existing-navmesh"], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 1)

    def test_scene_groups_doors_and_neighbor_connections(self):
        def connected_navm(x, polygon, edge, target, target_polygon, doors=()):
            payload = navm(x)
            body = bytearray(payload[6:])
            triangles_at = 20 + 4 * 12 + 4
            struct.pack_into("<H", body, triangles_at + polygon * 16 + 6 + edge * 2, 0)
            struct.pack_into("<H", body, triangles_at + polygon * 16 + 12, 1 << edge)
            trailing_at = triangles_at + 2 * 16
            connections = struct.pack("<IIIHI", 1, 0, target, target_polygon, len(doors))
            connections += b"".join(struct.pack("<HII", triangle, 0xE48B73F3, ref) for triangle, ref in doors)
            return sub("NVNM", bytes(body[:trailing_at]) + connections + body[trailing_at + 8:])

        def placed_exit(form, teleport=False, disabled=False):
            payload = sub("NAME", struct.pack("<I", 0x600))
            payload += sub("DATA", struct.pack("<6f", 64, 64, 0, 0, 0, 0))
            if teleport:
                payload += sub("XTEL", struct.pack("<I6f", 0x900, *([0] * 6)))
            return record("REFR", form, payload, (1 << 11) if disabled else 0)

        meshes = {0: connected_navm(0, 0, 1, 0x201, 1, [(0, 0x501)]),
                  1: connected_navm(1, 1, 2, 0x200, 0)}
        refs = b"".join([placed_exit(0x500, teleport=True), placed_exit(0x501),
                          placed_exit(0x502), placed_exit(0x503, teleport=True, disabled=True)])
        self.write_baseline(meshes, {0: refs})
        # Exit references can have no visible model, as for an invisible cave entrance.
        baseline = self.root / "Baseline.esm"
        baseline.write_bytes(baseline.read_bytes() + record("DOOR", 0x600, b""))
        unrelated = record("TES4", 0, sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800)))
        (self.root / "Unrelated.esm").write_bytes(unrelated)
        # The winning NAVM override has a different master table from its origin.
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800))
        header += sub("MAST", b"Unrelated.esm\0") + sub("DATA", bytes(8))
        header += sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        winning = connected_navm(0, 0, 1, 0x01000201, 1, [(0, 0x01000501)])
        patch = group(0x01000400, 1, group(0x01000100, 6,
                      group(0x01000100, 9, record("NAVM", 0x01000200, winning))))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, header) +
            group(int.from_bytes(b"WRLD", "little"), 0, patch))
        (self.root / "plugins.txt").write_text("Unrelated.esm\nBaseline.esm\nPatch.esp\n")

        def read_scene(output):
            data = (output / "scene.glb").read_bytes()
            json_size = struct.unpack_from("<I", data, 12)[0]
            return json.loads(data[20:20 + json_size]), data[28 + json_size:]

        for generating in (False, True):
            output = self.run_cli("--cell-formid", "01000100", "--neighboring-cell-radius", "1",
                                  *(["--generate-candidate"] if generating else []),
                                  output=f"scene-connections-{generating}")
            scene, binary = read_scene(output)
            roots = [scene["nodes"][index] for index in scene["scenes"][0]["nodes"]]
            expected = ["Original NAVM (current cell)", "Neighboring NAVM"]
            if generating:
                expected.append("Candidate NAVM")
            expected += ["NAVM links", "Doors", "Terrain", "Collision", "Render fallback"]
            self.assertEqual([node["name"] for node in roots], expected)
            self.assertEqual([scene["nodes"][i]["name"] for i in roots[0]["children"]],
                             ["Existing NAVM 01000200: door_linked", "Existing NAVM 01000200: supported"])
            self.assertTrue(all("01000201" in scene["nodes"][i]["name"] for i in roots[1]["children"]))
            names = [node["name"] for node in scene["nodes"]]
            self.assertIn("Door 01000500", names)
            self.assertIn("Door 01000501", names)
            self.assertNotIn("Door 01000502", names)
            self.assertNotIn("Door 01000503", names)
            self.assertFalse(any(name.startswith("Diagnostic:") for name in names))
            door_faces = next(mesh for mesh in scene["meshes"] if mesh["name"] ==
                              "Existing NAVM 01000200: door_linked")
            material = scene["materials"][door_faces["primitives"][0]["material"]]
            self.assertIn("door-linked", material["name"])
            self.assertGreater(material["pbrMetallicRoughness"]["baseColorFactor"][0],
                               material["pbrMetallicRoughness"]["baseColorFactor"][1])
            bars = [mesh for mesh in scene["meshes"] if mesh["name"].startswith("Authored link")]
            self.assertEqual(len(bars), 1)  # Reciprocal directions share one bar.
            bar = bars[0]["primitives"][0]
            color = scene["materials"][bar["material"]]["pbrMetallicRoughness"]["baseColorFactor"]
            self.assertGreater(color[1], color[0])
            self.assertGreater(color[1], color[2])
            positions = scene["accessors"][bar["attributes"]["POSITION"]]
            view = scene["bufferViews"][positions["bufferView"]]
            points = [struct.unpack_from("<3f", binary, view["byteOffset"] + i * 12)
                      for i in range(positions["count"])]
            self.assertEqual(len(points), 8)
            # Prism ends follow the connected edge at the cell boundary, lifted above the surface.
            centers = [tuple(sum(point[axis] for point in points[start:start + 4]) / 4
                             for axis in range(3)) for start in (0, 4)]
            self.assertEqual(centers, [(4096, 0, 8), (4096, 4096, 8)])
            self.assertEqual(scene["accessors"][bar["indices"]]["count"], 36)
            if generating:
                candidate = json.loads((output / "candidate-navm.json").read_text())
                self.assertEqual(sum(name.startswith("Candidate link") for name in names),
                                 len(candidate["border_links"]))
                self.assertIn("Candidate NAVM: door_linked", names)
            provenance = json.loads((output / "scene.glb.provenance.json").read_text())
            self.assertEqual(provenance["layers"], expected)

        simple_output = self.root / "scene-single-plugin"
        completed = subprocess.run([str(EXE), "--plugin", str(baseline), "--cell-formid", "100",
                                    "--terrain-only", "--output", str(simple_output)],
                                   capture_output=True, text=True, timeout=90)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        simple, _ = read_scene(simple_output)
        names = [node["name"] for node in simple["nodes"]]
        self.assertIn("Existing NAVM 00000200: door_linked", names)
        self.assertIn("Door 00000500", names)
        self.assertIn("Door 00000501", names)
        self.assertNotIn("Door 00000502", names)
        self.assertNotIn("Door 00000503", names)

        # Culling either triangle removes its connecting bar; layer filtering keeps stable order.
        output = self.run_cli("--cell-formid", "01000100", "--neighboring-cell-radius", "1",
                              "--scene-bounds", "1", "1", "100", "100", output="scene-bounded")
        bounded, _ = read_scene(output)
        self.assertFalse(any(mesh["name"].startswith("Authored link") for mesh in bounded["meshes"]))
        output = self.run_cli("--cell-formid", "01000100", "--neighboring-cell-radius", "1",
                              "--geometry-layers", "terrain,diagnostics,navmesh", output="scene-reordered")
        filtered, _ = read_scene(output)
        self.assertEqual([filtered["nodes"][i]["name"] for i in filtered["scenes"][0]["nodes"]],
                         ["Original NAVM (current cell)", "Neighboring NAVM", "NAVM links", "Doors", "Terrain"])

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


    def test_semantic_metadata_overrides_do_not_select_worldspace(self):
        header = sub("HEDR", struct.pack("<fII", 1.7, 0, 0x800)) + sub("MAST", b"Baseline.esm\0") + sub("DATA", bytes(8))
        world = record("WRLD", 0x400, sub("EDID", b"DisplayName\0") + sub("FULL", b"New label\0") + sub("MHDT", bytes(64)))
        cell = record("CELL", 0x100, sub("EDID", b"ChangedLabel\0") + sub("XCLC", struct.pack("<ii", 0, 0)))
        texture = group(0x100, 6, group(0x100, 9, record("LAND", 0x300, sub("VCLR", bytes(33 * 33 * 3)))))
        patch = world + group(0x400, 1, cell + texture)
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, header) + group(int.from_bytes(b"WRLD", "little"), 0, patch))
        output = self.run_cli("--rebuild-plugin", "Patch.esp")
        self.assertEqual(json.loads((output / "batch-report.json").read_text())["selected_cells"], 0)
        # Unknown fields stay conservative, even where display fields are ignored.
        patch = record("WRLD", 0x400, sub("EDID", b"DisplayName\0") + sub("ZZZZ", b"unknown dependency"))
        (self.root / "Patch.esp").write_bytes(record("TES4", 0, header) + group(int.from_bytes(b"WRLD", "little"), 0, patch))
        output = self.run_cli("--rebuild-plugin", "Patch.esp", output="unknown")
        self.assertEqual(json.loads((output / "batch-report.json").read_text())["selected_cells"], 3)

    def test_compact_and_minimal_output_preserve_plugin_bytes(self):
        import gzip
        full = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", output="full")
        compact = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--batch-output", "compact", output="compact")
        minimal = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--batch-output", "auto", output="minimal")
        self.assertEqual((full / "generated-navmesh.esp").read_bytes(), (compact / "generated-navmesh.esp").read_bytes())
        self.assertEqual((full / "generated-navmesh.esp").read_bytes(), (minimal / "generated-navmesh.esp").read_bytes())
        self.assertFalse((minimal / "load-order.json").exists())
        self.assertFalse((minimal / "cells").exists())
        for directory in (full / "cells").iterdir():
            self.assertEqual(json.loads((directory / "candidate-navm.json").read_text()),
                             json.loads(gzip.decompress((compact / "cells" / directory.name / "candidate-navm.json.gz").read_bytes())))
            self.assertFalse((compact / "cells" / directory.name / "candidate-navm.obj").exists())
        size = lambda directory: sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())
        self.assertLess(size(minimal), size(full) // 2)
        self.assertTrue(all(not list(directory.glob(".candidate-staging-*")) for directory in (full, compact, minimal)))

    def test_candidate_reuse_invalidates_on_authored_neighbor_change(self):
        self.run_cli("--rebuild-plugin", "Patch.esp", output="first")
        repeated = self.run_cli("--rebuild-plugin", "Patch.esp", output="repeated")
        self.assertEqual(json.loads((repeated / "batch-report.json").read_text())["candidate_cache_hits"], 2)
        changed = bytearray(navm(2))
        # Edit every authored vertex's height in the neighboring NAVM while leaving LAND unchanged.
        for index in range(4):
            struct.pack_into("<f", changed, 6 + 20 + index * 12 + 8, 1.0)
        self.write_baseline({0: navm(0), 1: navm(1), 2: bytes(changed)})
        revised = self.run_cli("--rebuild-plugin", "Patch.esp", output="revised")
        report = json.loads((revised / "batch-report.json").read_text())
        self.assertEqual(report["candidate_cache_hits"], 1)
        self.assertEqual([cell["candidate_reused"] for cell in report["cells"]], [True, False])

    def test_bounded_workers_preserve_order_and_plugin(self):
        single = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", output="single")
        # Separate cold cache ensures both cells exercise Recast tasks rather than reuse.
        parallel = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--workers", "2",
                                "--working-memory-mib", "1024", "--asset-cache", str(self.root / "parallel-cache"), output="parallel")
        report = json.loads((parallel / "batch-report.json").read_text())
        self.assertEqual(report["workers_peak"], 2)
        self.assertEqual((single / "generated-navmesh.esp").read_bytes(), (parallel / "generated-navmesh.esp").read_bytes())
        limited = self.run_cli("--rebuild-plugin", "Patch.esp", "--workers", "2", "--working-memory-mib", "64",
                               "--asset-cache", str(self.root / "limited-cache"), output="limited")
        self.assertEqual(json.loads((limited / "batch-report.json").read_text())["workers_peak"], 1)

    def test_zero_cache_budget_prunes_only_generated_files(self):
        cache = self.root / "cache"
        (cache / "operator-files").mkdir(parents=True)
        (cache / "operator-files" / "keep.nif").write_bytes(b"operator owned")
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--cache-budget-mib", "0")
        self.assertTrue((output / "generated-navmesh.esp").exists())
        self.assertEqual((cache / "operator-files" / "keep.nif").read_bytes(), b"operator owned")
        self.assertFalse(list(cache.glob("*/candidates/*.gz")))

    def test_preflight_samples_without_writing_plugin_and_reuses_work(self):
        output = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", "--estimate-only",
                              "--batch-output", "auto", output="estimate")
        report = json.loads((output / "batch-report.json").read_text())
        self.assertEqual(report["status"], "estimated")
        self.assertEqual(report["selected_cells"], 2)
        self.assertEqual(report["eligible_cells"], 2)
        self.assertEqual(report["sampled_cells"], 2)
        self.assertEqual(report["completed_cells"], 2)
        self.assertFalse((output / "generated-navmesh.esp").exists())
        self.assertFalse((output / "cells").exists())
        full = self.run_cli("--rebuild-plugin", "Patch.esp", "--generate-plugin", output="after-estimate")
        self.assertEqual(json.loads((full / "batch-report.json").read_text())["candidate_cache_hits"], 2)
        self.assertTrue((full / "generated-navmesh.esp").exists())
        self.run_cli("--cell-formid", "100", "--estimate-only", code=1)

    def test_performance_options_validate(self):
        for args in [("--workers", "0"), ("--workers", "-1"), ("--working-memory-mib", "0"),
                     ("--cache-budget-mib", "-1"), ("--batch-output", "unknown"), ("--batch-output", "plugin_only")]:
            self.run_cli("--rebuild-plugin", "Patch.esp", *args, code=1)


    def test_listing_writes_catalog_without_console_rows(self):
        output = self.root / "catalog"
        completed = subprocess.run([str(EXE), "--data", str(self.root), "--load-order", str(self.root / "plugins.txt"),
                                    "--list-cells", "--output", str(output)], capture_output=True, text=True, timeout=90)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertTrue((output / "cells.json").exists())
        self.assertNotIn("editor_id=", completed.stdout)
        self.assertNotIn("coords=", completed.stdout)
        self.assertFalse((output / "geometry.json").exists())
        self.assertFalse((output / "candidate-navm.json").exists())

    def test_recast_settings_reach_cell_and_batch_and_invalidate_cache(self):
        flags = ["--agent-radius", "8", "--agent-height", "120", "--agent-clearance", "120",
                 "--agent-step-height", "24", "--agent-max-slope", "40", "--minimum-region-area", "5000",
                 "--weld-tolerance", "0.1", "--recast-cell-size", "8", "--recast-cell-height", "4",
                 "--recast-simplification-error", "1.5", "--recast-max-edge-length", "256",
                 "--recast-merge-area-multiplier", "3"]
        single = self.run_cli("--cell-formid", "100", "--generate-candidate", *flags, output="custom-cell")
        batch = self.run_cli("--rebuild-plugin", "Patch.esp", *flags, output="custom-batch")
        reference = json.loads((single / "candidate-navm.json").read_text())
        self.assertEqual(reference["profile"]["agent_radius"], 8)
        self.assertEqual(reference["profile"]["step_height"], 24)
        self.assertEqual(reference["recast_settings"], {"cell_size": 8, "cell_height": 4,
            "max_simplification_error": 1.5, "max_edge_length": 256, "merge_region_area_multiplier": 3})
        for cell in (batch / "cells").iterdir():
            candidate = json.loads((cell / "candidate-navm.json").read_text())
            self.assertEqual(candidate["profile"], reference["profile"])
            self.assertEqual(candidate["recast_settings"], reference["recast_settings"])
            self.assertTrue(candidate["topology"]["valid"])
        repeated = self.run_cli("--rebuild-plugin", "Patch.esp", *flags, output="custom-repeated")
        self.assertEqual(json.loads((repeated / "batch-report.json").read_text())["candidate_cache_hits"], 2)
        revised = self.run_cli("--rebuild-plugin", "Patch.esp", *flags, "--recast-cell-height", "2", output="custom-revised")
        self.assertEqual(json.loads((revised / "batch-report.json").read_text())["candidate_cache_hits"], 0)

    def test_recast_settings_reject_invalid_values(self):
        invalid = [("--recast-cell-size", "0"), ("--recast-cell-height", "nan"), ("--agent-height", "inf"),
                   ("--agent-radius", "-1"), ("--agent-max-slope", "90"), ("--minimum-region-area", "-1"),
                   ("--recast-simplification-error", "-1"), ("--recast-max-edge-length", "1junk"),
                   ("--recast-merge-area-multiplier", "-1"), ("--weld-tolerance", "0"),
                   ("--recast-cell-height", "0.000001"), ("--agent-step-height", "-1")]
        for flag, value in invalid:
            with self.subTest(flag=flag, value=value):
                self.run_cli("--rebuild-plugin", "Patch.esp", flag, value, code=1)
        self.run_cli("--rebuild-plugin", "Patch.esp", "--recast-cell-size", code=1)


if __name__ == "__main__":
    unittest.main()
