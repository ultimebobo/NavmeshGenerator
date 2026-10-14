"""Showcase orchestration regressions using redistributable synthetic scenes."""

import importlib.util
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_media_renderer import square, write_scene


spec = importlib.util.spec_from_file_location(
    "remake_showcase", Path(__file__).parents[1] / "tools/remake_showcase.py")
showcase = importlib.util.module_from_spec(spec)
spec.loader.exec_module(showcase)


class ShowcaseTests(unittest.TestCase):
    def setUp(self):
        self.preset = {"id": "synthetic-region", "title": "Synthetic region",
                       "cells": ["SyntheticCell"], "view": "exterior", "azimuth": 0, "elevation": 78}

    def write_export(self, folder):
        scene = folder / "scene.glb"
        write_scene(scene, [("Original NAVM (current cell)", "authored", square(0, 1)),
                            ("Candidate NAVM", "generated", square(0, 1)),
                            ("Collision", "support", square(-1, 1))])
        metadata = {"selected_cell": {"form_id": "00000001", "editor_id": "SyntheticCell", "is_interior": False}}
        showcase.write_json(Path(str(scene) + ".metadata.json"), {"metadata": metadata})
        showcase.write_json(Path(str(scene) + ".provenance.json"), {"metadata": metadata, "objects": [
            {"layer": "Original NAVM (current cell)", "provenance": {"navmeshFormId": "00000002"}},
            {"layer": "Collision", "provenance": {"formId": "00000004", "model": "Shared\\Wall.nif"}},
        ]})
        showcase.write_json(folder / "load-order.json", {"records": [
            {"form_id": "00000001", "winning_plugin": "SyntheticOverride.esp", "origin_chain": ["Skyrim.esm", "SyntheticOverride.esp"]},
            {"form_id": "00000002", "winning_plugin": "SyntheticOverride.esp", "origin_chain": ["Skyrim.esm", "SyntheticOverride.esp"]},
            {"form_id": "00000003", "winning_plugin": "Unrelated.esp", "origin_chain": ["Unrelated.esp"]},
            {"form_id": "00000004", "winning_plugin": "Skyrim.esm", "origin_chain": ["Skyrim.esm"]},
        ]})
        showcase.write_json(folder / "input-report.json", {"snapshot_hash": "synthetic-snapshot", "loose_asset_winners": [
            {"logical_path": "meshes/shared/wall.nif", "source": "Synthetic Asset Replacement"},
            {"logical_path": "meshes/unrelated.nif", "source": "Unrelated Asset Mod"},
        ]})
        showcase.write_json(folder / "candidate-navm.json", {
            "topology": {"valid": True}, "triangle_tagging": False})

    def test_saved_export_rerenders_without_regenerating_and_retains_counts(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            recipes = folder / "presets.json"
            showcase.write_json(recipes, [self.preset])
            argv = ["remake_showcase.py", "--output", str(folder), "--presets", str(recipes),
                    "--exe", sys.executable, "--mo2", str(folder / "inputs"), "--profile", "Synthetic"]
            executed = []
            real_run = showcase.run_command

            def run(command, destination, stage):
                executed.append((command, destination, stage))
                if stage == "generator":
                    self.write_export(destination)
                else:
                    real_run(command, destination, stage)

            with patch.object(sys, "argv", argv), patch.object(showcase, "run_command", side_effect=run):
                showcase.main()
            self.assertIn("--no-triangle-tagging", executed[0][0])
            first_scene = json.loads((folder / "showcase-manifest.json").read_text())["scenes"][self.preset["id"]]["scene"]
            executed.clear()
            rerender_argv = argv[:7] + ["--render-only"]
            with patch.object(sys, "argv", rerender_argv), patch.object(showcase, "run_command", side_effect=run):
                showcase.main()
            self.assertEqual([stage for _, _, stage in executed], ["render"])
            manifest = json.loads((folder / "showcase-manifest.json").read_text())
            entry = manifest["scenes"][self.preset["id"]]
            self.assertEqual(entry["scene"], first_scene)
            self.assertIn("SyntheticOverride.esp", entry["attribution"])
            render = json.loads((folder / entry["render_manifest"]).read_text())
            self.assertEqual(render["renders"]["synthetic-region-navmesh-comparison.png"]["cell_count"], 1)
            self.assertEqual(render["comparisons"]["synthetic-region-navmesh-comparison.png"],
                             {"authored": 2, "generated": 2})
            self.assertTrue((folder / "gallery.html").is_file())
            # A fresh generation gets independent evidence even at the same output root.
            executed.clear()
            with patch.object(sys, "argv", argv[:7]), patch.object(showcase, "run_command", side_effect=run):
                showcase.main()
            self.assertNotEqual(str(executed[0][1] / "scene.glb"), first_scene)
            command = executed[0][0]
            self.assertEqual(command[command.index("--mo2") + 1], str(folder / "inputs"))
            self.assertEqual(command[command.index("--profile") + 1], "Synthetic")

    def test_attribution_uses_effective_scene_overrides_and_referenced_asset_winners(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            caption, evidence = showcase.scene_sources(folder / "scene.glb")
            self.assertIn("SyntheticOverride.esp", caption)
            self.assertIn("Synthetic Asset Replacement", caption)
            self.assertNotIn("Unrelated", caption)
            self.assertEqual(evidence["selected_cells"][0]["winning_plugin"], "SyntheticOverride.esp")
            self.assertEqual(evidence["authored_navm"][0]["winning_plugin"], "SyntheticOverride.esp")
            self.assertEqual(evidence["input_snapshot_hash"], "synthetic-snapshot")

    def test_switching_live_source_does_not_retain_a_caption_override_from_another_source(self):
        args = SimpleNamespace(mo2=Path("modded-inputs"), vortex=None, profile="Modded",
                               vortex_plugins=None, asset_cache=None, attribution=None)
        inputs = showcase.generation_inputs(args, {"mo2": "other-inputs", "profile": "Other",
                                                   "attribution": "Other input caption"})
        self.assertEqual(inputs["profile"], "Modded")
        self.assertNotIn("attribution", inputs)

    def test_missing_record_evidence_cannot_claim_official_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            showcase.write_json(folder / "load-order.json", {"records": []})
            with self.assertRaisesRegex(ValueError, "Missing winning scene records"):
                showcase.scene_sources(folder / "scene.glb")

    def test_render_only_cannot_silently_ignore_a_mod_manager_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            argv = ["remake_showcase.py", "--output", temporary, "--render-only", "--mo2", temporary]
            error_text = io.StringIO()
            with patch.object(sys, "argv", argv), redirect_stderr(error_text):
                with self.assertRaises(SystemExit) as error:
                    showcase.main()
            self.assertEqual(error.exception.code, 2)
            self.assertIn("omit it to apply mod-manager input changes", error_text.getvalue())

    def test_skips_invalid_topology_and_enabled_tagging_stop_showcase(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            for topology, tagging in ((False, False), (True, True)):
                showcase.write_json(folder / "candidate-navm.json", {
                    "topology": {"valid": topology}, "triangle_tagging": tagging})
                with self.assertRaisesRegex(ValueError, "Invalid topology"):
                    showcase.check_generation(folder)
            showcase.write_json(folder / "batch-report.json", {"cells": [{"status": "skipped_generation_failed"}]})
            with self.assertRaisesRegex(ValueError, "skipped cells"):
                showcase.check_generation(folder)

    def test_rerender_checks_cell_selection_and_preserves_case_notes(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            entry = {"scene": str(folder / "scene.glb"), "preset": self.preset,
                     "attribution": "Synthetic input", "note": "Synthetic generation qualification"}
            command = showcase.render_command(entry, folder)
            self.assertEqual(command[command.index("--note") + 1], entry["note"])
            entry["preset"] = dict(self.preset, cells=["DifferentCell"])
            with self.assertRaisesRegex(ValueError, "CELL selection does not match"):
                showcase.render_command(entry, folder)

    def test_failed_subprocess_retains_command_and_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            command = [sys.executable, "-c", "print('synthetic failure'); raise SystemExit(3)"]
            with self.assertRaisesRegex(ValueError, "generator failed"):
                showcase.run_command(command, folder, "generator")
            evidence = json.loads((folder / "generator-invocation.json").read_text())
            self.assertEqual(evidence["command"], command)
            self.assertEqual(evidence["exit_code"], 3)
            self.assertIn("synthetic failure", (folder / "generator.log").read_text())


if __name__ == "__main__":
    unittest.main()
