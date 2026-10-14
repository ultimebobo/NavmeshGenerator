"""Occlusion regressions for the scene image renderer; no game assets required."""

import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


spec = importlib.util.spec_from_file_location(
    "render_media", Path(__file__).parents[1] / "tools/render_media.py"
)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def square(height, extent):
    return np.array([
        [[-extent, -extent, height], [extent, -extent, height], [extent, extent, height]],
        [[-extent, -extent, height], [extent, extent, height], [-extent, extent, height]],
    ], dtype=np.float64)


def write_scene(path, scene):
    """Build a minimal world-coordinate GLB using the export's layer hierarchy."""
    binary = bytearray()
    document = {"asset": {"version": "2.0"}, "nodes": [], "meshes": [],
                "bufferViews": [], "accessors": []}
    for layer, name, triangles in scene:
        mesh_index = len(document["meshes"])
        child_index = len(document["nodes"]) + 1
        document["nodes"].extend([{"name": layer, "children": [child_index]},
                                  {"name": name, "mesh": mesh_index}])
        for data, component, kind in (
            (triangles.astype("<f4").reshape(-1, 3), 5126, "VEC3"),
            (np.arange(triangles.size // 3, dtype="<u4"), 5125, "SCALAR"),
        ):
            view_index = len(document["bufferViews"])
            document["bufferViews"].append({"buffer": 0, "byteOffset": len(binary),
                                            "byteLength": data.nbytes})
            document["accessors"].append({"bufferView": view_index, "componentType": component,
                                          "count": len(data), "type": kind})
            binary.extend(data.tobytes())
        document["meshes"].append({"name": name, "primitives": [
            {"attributes": {"POSITION": len(document["accessors"]) - 2},
             "indices": len(document["accessors"]) - 1}]})
    document["buffers"] = [{"byteLength": len(binary)}]
    encoded = json.dumps(document).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    path.write_bytes(struct.pack("<III", 0x46546C67, 2, 28 + len(encoded) + len(binary))
                     + struct.pack("<II", len(encoded), 0x4E4F534A) + encoded
                     + struct.pack("<II", len(binary), 0x004E4942) + binary)


class NavigationOcclusionTests(unittest.TestCase):
    def render_road_and_deck(self, navigation_height, elevation=90):
        scene = [
            ("Collision", "bridge deck", square(100, 2)),
            ("Candidate NAVM", "road navigation", square(navigation_height, 1)),
        ]
        return np.array(renderer.render(scene, "Candidate NAVM", None, 160, 160, 0, elevation))

    def navigation_pixels(self, image):
        return np.all(image == renderer.GENERATED, axis=2).sum()

    def test_bridge_deck_hides_road_navigation(self):
        self.assertEqual(self.navigation_pixels(self.render_road_and_deck(0)), 0)

    def test_coplanar_deck_navigation_remains_visible(self):
        self.assertGreater(self.navigation_pixels(self.render_road_and_deck(100)), 100)

    def test_navigation_above_the_deck_is_visible(self):
        self.assertGreater(self.navigation_pixels(self.render_road_and_deck(101)), 100)

    def test_small_burial_is_not_treated_as_coplanar(self):
        self.assertEqual(self.navigation_pixels(self.render_road_and_deck(99.99)), 0)

    def test_oblique_view_uses_collision_depth(self):
        # A broad deck covers the road's projection even from an oblique view.
        scene = [
            ("Collision", "bridge deck", square(1, 4)),
            ("Candidate NAVM", "road navigation", square(0, 1)),
        ]
        image = np.array(renderer.render(scene, "Candidate NAVM", None, 160, 160, 0, 60))
        self.assertEqual(self.navigation_pixels(image), 0)


class ComparisonCountsTests(unittest.TestCase):
    def test_cell_metadata_deduplicates_identities_and_survives_view_override(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.glb"
            Path(str(path) + ".metadata.json").write_text(json.dumps({"metadata": {
                "selected_cell": {"form_id": "00000003", "is_interior": True},
                "selected_cells": [{"form_id": "0000000a", "is_interior": False},
                                   {"form_id": "0000000A", "is_interior": False},
                                   {"form_id": "0000000B", "is_interior": False}],
            }}))
            settings = renderer.presentation([], path, "exterior", None, None, 0, None)
            self.assertEqual(settings["cell_count"], 2)
            self.assertEqual(settings["cell_count_source"], "metadata")
            self.assertIsNotNone(settings["metadata_source"])
            cropped = renderer.presentation([], path, "exterior", None, (-1, -1, 1, 1), 0, None)
            self.assertTrue(cropped["cell_count_before_crop"])
            overridden = renderer.presentation([], path, "exterior", None, (-1, -1, 1, 1),
                                               0, None, cell_count=1)
            self.assertEqual(overridden["cell_count"], 1)
            self.assertEqual(overridden["cell_count_source"], "override")
            self.assertFalse(overridden["cell_count_before_crop"])

    def test_missing_metadata_does_not_invent_cell_count(self):
        settings = renderer.presentation([], Path("absent.glb"), "auto", None, None, 0, None)
        self.assertIsNone(settings["cell_count"])
        self.assertEqual(renderer.cell_caption(None), "Cell count unavailable")

    def test_cell_counts_are_drawn_on_scenes_and_comparisons(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            with patch.object(renderer, "render", return_value=renderer.Image.new("RGB", (1, 1))):
                with patch.object(renderer, "text") as draw_text:
                    renderer.comparison([], folder, cell_count=3)
                    self.assertIn("3 selected cells", [call.args[2] for call in draw_text.call_args_list])
                with patch.object(renderer, "text") as draw_text:
                    renderer.scene_image([], folder, "scene.png", "Scene", "Synthetic", cell_count=1)
                    self.assertIn("1 selected cell", [call.args[2] for call in draw_text.call_args_list])

    def test_counts_include_hidden_faces_and_all_navigation_materials(self):
        scene = [
            ("Collision", "covering deck", square(100, 4)),
            ("Original NAVM (current cell)", "supported", square(0, 1)),
            ("Original NAVM (current cell)", "door linked", square(1, 1)),
            ("Candidate NAVM", "water", square(0, 1)),
            ("Neighboring NAVM", "outside selection", square(0, 1)),
            ("NAVM links", "connection bar", square(0, 1)),
        ]
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(renderer, "render", return_value=renderer.Image.new("RGB", (1, 1))):
                counts = renderer.comparison(scene, Path(temporary), attribution="Synthetic fixture")
            self.assertEqual(counts, {"authored": 4, "generated": 2})
            self.assertTrue((Path(temporary) / "navmesh-comparison.png").exists())

    def test_counts_use_display_crop_and_allow_empty_generated_navigation(self):
        outside = square(0, 1) + np.array([10, 0, 0])
        scene = [
            ("Original NAVM (current cell)", "selected", square(0, 1)),
            ("Original NAVM (current cell)", "outside", outside),
        ]
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(renderer, "render", return_value=renderer.Image.new("RGB", (1, 1))):
                counts = renderer.comparison(scene, Path(temporary), bounds=(-2, -2, 2, 2))
            self.assertEqual(counts, {"authored": 2, "generated": 0})


class InteriorCutawayTests(unittest.TestCase):
    def setUp(self):
        self.scene = [
            ("Collision", "floor", square(0, 2)),
            ("Collision", "roof", square(200, 10)),
            ("Original NAVM (current cell)", "authored floor", square(0, 1)),
            ("Candidate NAVM", "generated floor", square(0, 1)),
        ]

    def test_cutaway_reveals_navigation_and_removes_roof_from_framing(self):
        cutaway = np.array(renderer.render(self.scene, "Candidate NAVM", None, 160, 160, 0, 90,
                                          renderer.INTERIOR_CUT_MARGIN))
        floor_only = np.array(renderer.render([item for item in self.scene if item[1] != "roof"],
                                             "Candidate NAVM", None, 160, 160, 0, 90))
        np.testing.assert_array_equal(cutaway, floor_only)
        self.assertGreater(np.all(cutaway == renderer.GENERATED, axis=2).sum(), 100)

    def test_retained_obstacle_still_occludes_navigation(self):
        scene = self.scene + [("Collision", "low obstacle", square(10, 1))]
        image = np.array(renderer.render(scene, "Candidate NAVM", None, 160, 160, 0, 90,
                                        renderer.INTERIOR_CUT_MARGIN))
        self.assertEqual(np.all(image == renderer.GENERATED, axis=2).sum(), 0)

    def test_crossing_surfaces_keep_lower_portions_and_winding(self):
        triangle = np.array([[[0, 0, 0], [2, 0, 0], [0, 0, 2]]], dtype=np.float64)
        clipped = renderer.clip_support(triangle, 1)
        self.assertEqual(len(clipped), 2)
        self.assertTrue(np.all(clipped[:, :, 2] <= 1))
        normals = np.cross(clipped[:, 1] - clipped[:, 0], clipped[:, 2] - clipped[:, 0])
        self.assertTrue(np.all(normals[:, 1] < 0))
        self.assertAlmostEqual(np.linalg.norm(normals, axis=1).sum() / 2, 1.5)
        reverse = renderer.clip_support(triangle[:, ::-1], 1)
        self.assertAlmostEqual(np.linalg.norm(np.cross(reverse[:, 1] - reverse[:, 0],
                                                     reverse[:, 2] - reverse[:, 0]), axis=1).sum() / 2, 1.5)
        one_below = renderer.clip_support(np.array([[[0, 0, 0], [2, 0, 2], [0, 0, 2]]]), 1)
        self.assertEqual(len(one_below), 1)
        np.testing.assert_array_equal(triangle, [[[0, 0, 0], [2, 0, 0], [0, 0, 2]]])

    def test_vertices_on_cut_do_not_create_degenerate_fragments(self):
        triangle = np.array([[[0, 0, 1], [2, 0, 0], [0, 0, 2]]], dtype=np.float64)
        self.assertEqual(len(renderer.clip_support(triangle, 1)), 1)
        np.testing.assert_array_equal(renderer.clip_support(square(1, 1), 1), square(1, 1))
        self.assertEqual(renderer.clip_support(square(2, 1), 1).shape, (0, 3, 3))

    def test_auto_cut_uses_both_layers_after_bounds_selection(self):
        scene = self.scene + [("Original NAVM (current cell)", "upper floor", square(30, 1)),
                              ("Candidate NAVM", "outside crop", square(500, 1) + [50, 0, 0])]
        settings = renderer.presentation(scene, Path("unused.glb"), "interior", None,
                                         (-2, -2, 2, 2), 0, None)
        self.assertEqual(settings["cut_height"], 30 + renderer.INTERIOR_CUT_MARGIN)
        self.assertEqual(settings["elevation"], 70)

    def test_empty_navigation_requires_explicit_cut(self):
        scene = [self.scene[0]]
        with self.assertRaisesRegex(ValueError, "--cut-height"):
            renderer.presentation(scene, Path("unused.glb"), "interior", None, None, 0, None)
        settings = renderer.presentation(scene, Path("unused.glb"), "auto", 20, None, 0, 60)
        self.assertEqual(settings["view"], "interior")
        self.assertEqual(settings["elevation"], 60)

    def test_metadata_detection_and_explicit_view_override(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scene.glb"
            sidecar = Path(str(path) + ".metadata.json")
            for cells, expected in (
                ([{"is_interior": True}], "interior"),
                ([{"is_interior": True}, {"is_interior": True}], "interior"),
                ([{"is_interior": True}, {"is_interior": False}], "exterior"),
                ([{"is_interior": False}], "exterior"),
            ):
                sidecar.write_text(json.dumps({"metadata": {"selected_cells": cells}}))
                settings = renderer.presentation(self.scene, path, "auto", None, None, 0, None)
                self.assertEqual(settings["view"], expected)
                self.assertEqual(settings["elevation"], 70 if expected == "interior" else 48)
            sidecar.write_text(json.dumps({"metadata": {"selected_cell": {"is_interior": True}}}))
            settings = renderer.presentation(self.scene, path, "auto", None, None, 0, None)
            self.assertEqual(settings["view"], "interior")
            self.assertIsNotNone(settings["metadata_source"])
            settings = renderer.presentation(self.scene, path, "exterior", None, None, 0, None)
            self.assertIsNone(settings["cut_height"])
            sidecar.unlink()
            settings = renderer.presentation(self.scene, path, "auto", None, None, 0, None)
            self.assertEqual(settings["view"], "exterior")

    def test_cli_renders_both_outputs_with_effective_manifest_settings(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            path = folder / "scene.glb"
            write_scene(path, self.scene)
            Path(str(path) + ".metadata.json").write_text(json.dumps(
                {"metadata": {"selected_cell": {"is_interior": True}}}))
            argv = ["render_media.py", "--scene", str(path), "--comparison", str(path),
                    "--output", str(folder), "--attribution", "Synthetic interior"]
            with patch.object(sys, "argv", argv):
                renderer.main()
            manifest = json.loads((folder / "render-manifest.json").read_text())
            self.assertEqual(manifest["comparisons"]["navmesh-comparison.png"],
                             {"authored": 2, "generated": 2})
            for filename in ("scene.png", "navmesh-comparison.png"):
                settings = manifest["renders"][filename]
                self.assertEqual(settings["view"], "interior")
                self.assertEqual(settings["cut_height"], renderer.INTERIOR_CUT_MARGIN)
                self.assertEqual(settings["elevation"], 70)
                self.assertEqual(settings["cell_count"], 1)
                self.assertEqual(len(settings["metadata_source"]["sha256"]), 64)
                with renderer.Image.open(folder / filename) as image:
                    self.assertGreater(np.all(np.array(image) == renderer.GENERATED, axis=2).sum(), 100)
            with patch.object(sys, "argv", argv + ["--view", "exterior", "--cut-height", "10"]):
                with self.assertRaises(SystemExit) as error:
                    renderer.main()
                self.assertEqual(error.exception.code, 2)
            with patch.object(sys, "argv", argv + ["--cut-height", "nan"]):
                with self.assertRaises(SystemExit) as error:
                    renderer.main()
                self.assertEqual(error.exception.code, 2)
            with patch.object(sys, "argv", argv + ["--cell-count", "0"]):
                with self.assertRaises(SystemExit) as error:
                    renderer.main()
                self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
