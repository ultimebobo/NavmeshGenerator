"""Optional CLI export regressions against a locally installed vanilla master."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import unittest
import zlib


PROJECT = Path(__file__).resolve().parents[2]
SINGLE_CELLS = (
    "SteepfallBurrow01", "RobbersGorge01", "LostProspectMine01",
    "FortSnowhawk02", "ThalmorEmbassy05", "GoldenglowEstate04",
    "CrystaldriftCave01", "BoulderfallCave01", "GraywinterWatch01",
    "Icerunner01", "RiftenEsbernsVault", "ClearspringTarn01",
    "Mzinchaleft03", "WhiterunDragonsreachBasement", "RiftenRatway01",
    "ReachwaterRock01", "ValkyggExterior02", "LabyrinthianExterior02",
    "ElenwensResidenceExterior",
)
# This cross-cell descent needs its adjacent border suppliers regenerated together.
NEIGHBOR_CASE = ("BonechillPassageExterior01", "00009C78", "00009C9A")
OPTIONS = argparse.Namespace(
    exe=str(PROJECT / "build/windows/x64/release/NavmeshGenerator.exe"),
    data=os.environ.get("SKYRIM_DATA_DIR"), output=None, asset_cache=None, timeout=300,
)


def subrecords(payload):
    """Decode bounded subrecords, including extended lengths, independently of the app."""
    offset = 0
    extended = None
    while offset < len(payload):
        kind, size = struct.unpack_from("<4sH", payload, offset)
        offset += 6
        if kind == b"XXXX":
            if size != 4 or extended is not None:
                raise ValueError("Invalid extended subrecord")
            extended = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
            continue
        size = size if extended is None else extended
        if offset + size > len(payload):
            raise ValueError("Subrecord exceeds payload")
        yield kind, payload[offset:offset + size]
        offset += size
        extended = None
    if offset != len(payload) or extended is not None:
        raise ValueError("Incomplete subrecord")


def read_navigation(path):
    """Read CELL identities and consumed live NVNM connections from source or output."""
    data = path.read_bytes()
    cells, meshes = {}, {}

    def visit(start, end, cell=None):
        while start < end:
            kind = data[start:start + 4]
            size, flags, form = struct.unpack_from("<III", data, start + 4)
            following = start + (size if kind == b"GRUP" else 24 + size)
            if following > end or following <= start or end - start < 24:
                raise ValueError("Invalid record boundary")
            if kind == b"GRUP":
                visit(start + 24, following, flags if form in (6, 8, 9, 10) else cell)
            elif kind in (b"CELL", b"NAVM") and not flags & 0x20:
                payload = data[start + 24:following]
                if flags & 0x40000:
                    expected = struct.unpack_from("<I", payload)[0]
                    payload = zlib.decompress(payload[4:])
                    if len(payload) != expected:
                        raise ValueError("Invalid decompressed record size")
                subs = dict(subrecords(payload))
                if kind == b"CELL":
                    cells[form] = subs.get(b"EDID", b"").rstrip(b"\0").decode("utf-8")
                elif b"NVNM" in subs:
                    body = subs[b"NVNM"]
                    if struct.unpack_from("<I", body)[0] != 12:
                        raise ValueError("Unsupported NVNM version")
                    vertex_count = struct.unpack_from("<I", body, 16)[0]
                    at = 20 + vertex_count * 12
                    count = struct.unpack_from("<I", body, at)[0]
                    faces = list(struct.iter_unpack("<8H", body[at + 4:at + 4 + count * 16]))
                    at += 4 + count * 16
                    count = struct.unpack_from("<I", body, at)[0]
                    entries = list(struct.iter_unpack("<IIH", body[at + 4:at + 4 + count * 10]))
                    links = []
                    for triangle, face in enumerate(faces):
                        if face[6] & 8:
                            continue
                        for edge in range(3):
                            if face[6] & (1 << edge):
                                link_type, target, target_triangle = entries[face[3 + edge]]
                                links.append((triangle, edge, link_type, target, target_triangle))
                    meshes[form] = {"cell": cell, "faces": faces, "links": links}
            start = following
        if start != end:
            raise ValueError("Incomplete group")

    visit(0, len(data))
    return cells, meshes


class VanillaLedgeLinks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not OPTIONS.data:
            raise unittest.SkipTest("Set SKYRIM_DATA_DIR or pass --data for local-game checks")
        cls.data = Path(OPTIONS.data).resolve()
        cls.cells, cls.source = read_navigation(cls.data / "Skyrim.esm")
        cls.output = Path(OPTIONS.output).resolve() if OPTIONS.output else Path(
            tempfile.mkdtemp(prefix="vanilla-ledges-"))
        if OPTIONS.output:
            cls.output.mkdir(parents=True, exist_ok=False)
        cls.load_order = cls.output / "load-order.txt"
        cls.load_order.write_text("Skyrim.esm\n", encoding="utf-8")
        cls.results = []
        print(f"Vanilla ledge regression artifacts: {cls.output}", flush=True)

    def resolve_cell(self, selector):
        matches = [form for form, name in self.cells.items()
                   if name == selector or f"{form:08X}" == selector]
        self.assertEqual(len(matches), 1, f"Ambiguous or missing vanilla cell: {selector}")
        return matches[0]

    def check_candidate(self, folder, form, emitted):
        candidate = json.loads((folder / "candidate-navm.json").read_text(encoding="utf-8"))
        self.assertTrue(candidate["topology"]["valid"], candidate["topology"])
        self.assertTrue(candidate["polygons"], "Empty candidate must not count as a pass")
        primary = [(identity, mesh) for identity, mesh in emitted.items()
                   if mesh["cell"] == form and mesh["faces"]]
        self.assertEqual(len(primary), 1, "Expected one nonempty output NAVM per cell")
        identity, mesh = primary[0]
        self.assertEqual(len(mesh["faces"]), len(candidate["polygons"]))
        # Cover encoding may cyclically rotate a face and all its edge slots.
        # Compare oriented triangles, then match ledges to the emitted edge numbers.
        edge_mapping = {}
        for index, (face, polygon) in enumerate(zip(mesh["faces"], candidate["polygons"])):
            vertices = polygon["vertices"]
            self.assertIn(list(face[:3]), [vertices[shift:] + vertices[:shift] for shift in range(3)])
            for edge, vertex in enumerate(vertices):
                edge_mapping[index, edge] = face[:3].index(vertex)
        down = [link for link in mesh["links"] if link[2] == 2]
        up = [link for link in mesh["links"] if link[2] == 1]
        expected_down = Counter((link["upper_polygon"],
                                 edge_mapping[link["upper_polygon"], link["upper_edge"]], 2, identity,
                                 link["lower_polygon"]) for link in candidate["ledge_links"])
        expected_up = Counter((link["lower_polygon"],
                               edge_mapping[link["lower_polygon"], link["lower_edge"]], 1, identity,
                               link["upper_polygon"]) for link in candidate["ledge_links"])
        self.assertEqual(Counter(down), expected_down)
        self.assertEqual(Counter(up), expected_up)
        self.assertTrue(all(count == 1 for count in expected_down.values()))
        self.assertTrue(all(count == 1 for count in expected_up.values()))
        return len(down)

    def check_export(self, selectors, result):
        forms = [self.resolve_cell(selector) for selector in selectors]
        authored = [(identity, link) for identity, mesh in self.source.items()
                    if mesh["cell"] == forms[0] for link in mesh["links"] if link[2] == 2]
        self.assertTrue(authored, "Sample must contain live authored Ledge Down entries")
        for identity, (triangle, _, _, target, target_triangle) in authored:
            self.assertIn(target, self.source)
            reverse = [link for link in self.source[target]["links"]
                       if link[0] == target_triangle and link[2:] == (1, identity, triangle)]
            self.assertEqual(len(reverse), 1, "Expected a unique live authored Ledge Up")
        result["authored_ledge_pairs"] = len(authored)
        output = self.output / selectors[0]
        command = [str(Path(OPTIONS.exe).resolve()), "--data", str(self.data),
                   "--load-order", str(self.load_order), "--generate-plugin", "--output", str(output)]
        if len(forms) == 1:
            command += ["--cell-formid", f"{forms[0]:08X}"]
        else:
            command += ["--cells", ",".join(f"{form:08X}" for form in forms),
                        "--batch-output", "full", "--cell-error-policy", "stop"]
        if OPTIONS.asset_cache:
            command += ["--asset-cache", str(Path(OPTIONS.asset_cache).resolve())]
        result["command"] = command
        with (self.output / f"{selectors[0]}.log").open("w", encoding="utf-8") as log:
            process = subprocess.run(command, cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=OPTIONS.timeout)
        result["exit_code"] = process.returncode
        self.assertEqual(process.returncode, 0, f"CLI failure; see {log.name}")
        self.assertFalse((output / "bug-report.txt").exists(), f"Bug report in {output}")
        if len(forms) > 1:
            report = json.loads((output / "batch-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "complete")
            self.assertFalse(report["error"])
            self.assertEqual(report["generation_failed_cells"], 0)
            self.assertEqual(report["extraction_failed_cells"], 0)
            self.assertEqual({cell["form_id"] for cell in report["cells"]},
                             {f"{form:08X}" for form in forms})
            self.assertTrue(all(cell["status"] == "generated" and not cell["error"]
                                for cell in report["cells"]), report["cells"])
        plugin = output / "generated-navmesh.esp"
        self.assertTrue(plugin.is_file(), "Successful exit without a plugin must fail this regression")
        _, emitted = read_navigation(plugin)
        result["generated_ledge_pairs"] = sum(self.check_candidate(
            output if len(forms) == 1 else output / "cells" / f"{form:08X}", form, emitted)
            for form in forms)
        result["exported_cells"] = len(forms)

    def test_vanilla_exports(self):
        for selectors in [*((name,) for name in SINGLE_CELLS), NEIGHBOR_CASE]:
            with self.subTest(cell=selectors[0]):
                result = {"cell": selectors[0], "passed": False}
                started = time.monotonic()
                try:
                    self.check_export(selectors, result)
                    result["passed"] = True
                except Exception as error:
                    result["error"] = str(error)
                    raise
                finally:
                    result["seconds"] = round(time.monotonic() - started, 2)
                    self.results.append(result)
                    (self.output / "results.json").write_text(
                        json.dumps(self.results, indent=2) + "\n", encoding="utf-8")
                    print(f"{selectors[0]}: {'PASS' if result['passed'] else 'FAIL'} "
                          f"({result['seconds']}s)", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", default=OPTIONS.exe)
    parser.add_argument("--data", default=OPTIONS.data)
    parser.add_argument("--output", help="New directory for logs, plugins and results.json")
    parser.add_argument("--asset-cache", help="Optional shared asset cache directory")
    parser.add_argument("--timeout", type=float, default=300, help="Seconds allowed per CLI invocation")
    OPTIONS, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__, *remaining])
