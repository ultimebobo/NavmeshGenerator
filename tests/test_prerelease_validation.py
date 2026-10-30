"""Release gates tested with synthetic exports; no installed game is required."""

from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import validate_prerelease as validation
from test_media_renderer import square, write_scene


class PrereleaseValidationTests(unittest.TestCase):
    def setUp(self):
        self.preset = {"id": "synthetic-region", "title": "Synthetic region",
                       "cells": ["SyntheticCell"], "view": "exterior", "azimuth": 0, "elevation": 78}
        self.inputs = {"mo2": "synthetic-instance", "profile": "Synthetic profile"}

    def write_export(self, folder, cells=None, counts=None):
        cells = cells or self.preset["cells"]
        counts = counts if counts is not None else [2] * len(cells)
        selection = [{"editor_id": name, "form_id": f"{index + 1:08X}"}
                     for index, name in enumerate(cells)]
        for cell, count in zip(selection, counts):
            destination = folder if len(cells) == 1 else folder / "cells" / cell["form_id"]
            destination.mkdir(parents=True, exist_ok=True)
            validation.showcase.write_json(destination / "candidate-navm.json", {
                "metadata": {"selected_cell": cell}, "triangle_tagging": False,
                "topology": {"valid": True}, "statistics": {"output_polygons": count},
                "polygons": [{"water": False, "preferred_path": False}] * count})
        write_scene(folder / "scene.glb", [("Candidate NAVM", "generated", square(0, 1))])
        validation.showcase.write_json(folder / "scene.glb.metadata.json", {
            "metadata": {"selected_cells": selection}})
        validation.showcase.write_json(folder / "scene.glb.provenance.json", {
            "metadata": {"selected_cells": selection}, "objects": []})
        validation.showcase.write_json(folder / "load-order.json", {"records": [
            {"form_id": cell["form_id"], "winning_plugin": "SyntheticOverride.esp"} for cell in selection]})
        validation.showcase.write_json(folder / "input-report.json", {
            "snapshot_hash": "synthetic-snapshot", "loose_asset_winners": []})
        if len(cells) > 1:
            validation.showcase.write_json(folder / "batch-report.json", {
                "status": "complete", "cells": [{"status": "generated"}] * len(cells),
                "generation_failed_cells": 0, "extraction_failed_cells": 0,
                "asset_failures": [], "generated_polygons": sum(counts)})

    def test_growth_boundary_does_not_round_up_and_reductions_pass(self):
        validation.check_polygon_growth(110, 100, 10)
        validation.check_polygon_growth(724, 659, 10)
        validation.check_polygon_growth(1, 659, 10)
        for polygons, baseline in ((111, 100), (725, 659)):
            with self.assertRaisesRegex(ValueError, "exceeds"):
                validation.check_polygon_growth(polygons, baseline, 10)

    def test_rejects_missing_invalid_tagged_empty_and_misidentified_exports(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            path = folder / "candidate-navm.json"
            original = validation.read_json(path)
            variants = [dict(original, topology={"valid": False}),
                        dict(original, triangle_tagging=True),
                        dict(original, polygons=[], statistics={"output_polygons": 0}),
                        dict(original, metadata={"selected_cell": {"editor_id": "OtherCell"}}),
                        dict(original, statistics={"output_polygons": 99})]
            for candidate in variants:
                with self.subTest(candidate=candidate):
                    validation.showcase.write_json(path, candidate)
                    with self.assertRaises(ValueError):
                        validation.check_candidates(folder, self.preset)
            path.unlink()
            with self.assertRaises(ValueError):
                validation.check_candidates(folder, self.preset)

    def test_batch_requires_complete_coverage_and_no_failures(self):
        preset = dict(self.preset, cells=["SyntheticCell", "SyntheticEmptyCell"])
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            # Empty cells are valid completed replacements; the region must contain navigation.
            self.write_export(folder, preset["cells"], [2, 0])
            self.assertEqual(validation.check_candidates(folder, preset)[1], 2)
            path = folder / "batch-report.json"
            original = validation.read_json(path)
            variants = [dict(original, status="partial"), dict(original, cells=[]),
                        dict(original, cells=[{"status": "skipped_generation_failed"}]),
                        dict(original, extraction_failed_cells=1),
                        dict(original, generation_failed_cells=1),
                        dict(original, asset_failures=["missing asset"]),
                        dict(original, generated_polygons=99)]
            for batch in variants:
                with self.subTest(batch=batch):
                    validation.showcase.write_json(path, batch)
                    with self.assertRaises(ValueError):
                        validation.check_candidates(folder, preset)
            path.unlink()
            with self.assertRaisesRegex(ValueError, "Missing batch"):
                validation.check_candidates(folder, preset)

    def test_real_render_matches_uncropped_candidates_and_audits_mod_winners(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.write_export(folder)
            image = validation.render_showcase(folder / "scene.glb", self.preset, folder, 2)
            self.assertTrue(Path(image).is_file())
            evidence = validation.read_json(folder / "source-evidence.json")
            self.assertEqual(evidence["record_plugins"], ["SyntheticOverride.esp"])
            with self.assertRaisesRegex(ValueError, "polygon total mismatch"):
                validation.render_showcase(folder / "scene.glb", self.preset, folder, 3)

    def test_growth_failure_retains_render_and_a_failed_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            policy = {"max_polygon_growth_percent": 10,
                      "baseline_polygons": {self.preset["id"]: 1}}
            with patch.object(validation.showcase, "run_command",
                              side_effect=lambda command, folder, stage: self.write_export(folder)), \
                 patch.object(validation, "render_showcase", return_value="synthetic.png") as render, \
                 redirect_stdout(io.StringIO()):
                row = validation.validate_showcase(self.preset, policy, self.inputs, root)
            self.assertEqual(row["status"], "failed")
            self.assertEqual(row["polygons"], 2)
            self.assertEqual(row["image"], "synthetic.png")
            self.assertIn("exceeds +10%", row["errors"][0])
            render.assert_called_once()

    def test_runs_remaining_showcases_after_failure_with_fresh_evidence_and_failure_exit(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            presets = [dict(self.preset, id="failed-region"), self.preset]
            recipes = root / "presets.json"
            validation.showcase.write_json(recipes, presets)
            policy = {"max_polygon_growth_percent": 10,
                      "baseline_polygons": {preset["id"]: 2 for preset in presets}}
            generated = []

            def run(command, folder, stage):
                if stage == "generator":
                    generated.append(folder)
                    self.assertIn("--mo2", command)
                    self.assertEqual(command[command.index("--mo2") + 1], self.inputs["mo2"])
                    self.assertEqual(command[command.index("--profile") + 1], self.inputs["profile"])
                    self.assertNotIn("--data", command)
                    self.assertNotIn("--load-order", command)
                    if folder.name == "failed-region":
                        raise ValueError("Synthetic generator failure")
                    self.write_export(folder)

            original_directory = Path.cwd()
            try:
                with patch.object(validation, "ROOT", root), \
                     patch.object(validation, "OUTPUT", root / "results"), \
                     patch.object(validation, "EXECUTABLE", Path(sys.executable)), \
                     patch.object(validation.showcase, "PRESETS", recipes), \
                     patch.object(validation, "load_policy", return_value=policy), \
                     patch.object(validation, "load_inputs", return_value=(self.inputs, recipes)), \
                     patch.object(validation.showcase, "run_command", side_effect=run), \
                     patch.object(validation, "render_showcase", return_value="synthetic.png"), \
                     patch.object(sys, "argv", ["validate_prerelease.py"]), redirect_stdout(io.StringIO()):
                    for _ in range(2):
                        self.assertEqual(validation.main(), 1)
                    latest = validation.read_json(root / "results/latest.json")
                    report = validation.read_json(Path(latest["directory"]) / "validation.json")
            finally:
                os.chdir(original_directory)
            self.assertEqual(len(generated), 4)
            self.assertNotEqual(generated[0].parent, generated[2].parent)
            self.assertEqual(report["status"], "failed")
            self.assertEqual([row["status"] for row in report["items"]], ["failed", "passed"])
            self.assertIn("Synthetic generator failure", report["items"][0]["errors"])

    def test_policy_requires_baselines_for_exactly_the_current_showcases(self):
        policy = {"max_polygon_growth_percent": 10,
                  "baseline_polygons": {self.preset["id"]: 2}}
        with patch.object(validation, "read_json", return_value=policy):
            self.assertEqual(validation.load_policy([self.preset]), policy)
            with self.assertRaisesRegex(ValueError, "No showcase"):
                validation.load_policy([])
            with self.assertRaisesRegex(ValueError, "baseline"):
                validation.load_policy([dict(self.preset, id="new-region")])

    def test_source_reuses_showcase_manifest_before_desktop_configuration(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / "output/showcase/showcase-manifest.json"
            manifest.parent.mkdir(parents=True)
            validation.showcase.write_json(manifest, {"input": self.inputs})
            with patch.object(validation, "ROOT", root):
                self.assertEqual(validation.load_inputs(), (self.inputs, manifest))
                validation.showcase.write_json(manifest, {"input": {"mo2": "instance"}})
                with self.assertRaisesRegex(ValueError, "needs a profile"):
                    validation.load_inputs()

    def test_source_reads_selected_desktop_manager_without_a_vanilla_fallback(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            settings_path = root / "NavmeshGenerator/config.ini"
            settings_path.parent.mkdir()
            with patch.object(validation, "ROOT", root), \
                 patch.object(validation.Path, "home", return_value=root), \
                 patch.dict(os.environ, LOCALAPPDATA=str(root), SKYRIM_DATA_DIR=str(root)):
                with self.assertRaisesRegex(ValueError, "Configure a mod-manager"):
                    validation.load_inputs()
                settings_path.write_text("[options]\nmod_manager=mo2\nmo2=synthetic-instance\n"
                                         "profile=Synthetic profile\n", encoding="utf-8")
                self.assertEqual(validation.load_inputs(), (self.inputs, settings_path))
                settings_path.write_text("[options]\nmod_manager=vortex\nvortex=synthetic-game\n"
                                         "vortex_plugins=synthetic-plugins\nmo2=ignored\nprofile=ignored\n",
                                         encoding="utf-8")
                inputs, origin = validation.load_inputs()
                self.assertEqual(inputs, {"vortex": "synthetic-game", "vortex_plugins": "synthetic-plugins"})
                self.assertEqual(origin, settings_path)


if __name__ == "__main__":
    unittest.main()
