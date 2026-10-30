"""Build the release executable and validate fresh showcases without arguments."""

import argparse
import configparser
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

from PIL import Image

import remake_showcase as showcase
import render_media as renderer


ROOT = Path(__file__).resolve().parents[1]
POLICY = Path(__file__).with_name("showcase-validation.json")
OUTPUT = ROOT / "output/prerelease-validation"
EXECUTABLE = ROOT / "build/windows/x64/release/NavmeshGenerator.exe"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition, message):
    """Keep release checks active even when Python runs with optimization."""
    if not condition:
        raise ValueError(message)


def load_policy(presets):
    policy = read_json(POLICY)
    baselines = policy["baseline_polygons"]
    identities = [preset["id"] for preset in presets]
    require(bool(identities), "No showcase presets selected")
    require(len(set(identities)) == len(identities), "Duplicate showcase IDs")
    require(set(baselines) == set(identities), "Every showcase must have exactly one polygon baseline")
    require(all(type(value) is int and value > 0 for value in baselines.values()),
            "Polygon baselines must be positive integers")
    growth = policy["max_polygon_growth_percent"]
    require(type(growth) is int and growth >= 0, "Polygon growth percent must be a nonnegative integer")
    return policy


def load_inputs():
    """Reuse the showcase's live source, or the desktop's configured mod manager."""
    manifest_path = ROOT / "output/showcase/showcase-manifest.json"
    if manifest_path.is_file():
        inputs = read_json(manifest_path)["input"]
        origin = manifest_path
    else:
        settings_paths = []
        if os.environ.get("LOCALAPPDATA"):
            settings_paths.append(Path(os.environ["LOCALAPPDATA"]) / "NavmeshGenerator/config.ini")
        settings_paths.append(Path.home() / "AppData/Local/NavmeshGenerator/config.ini")
        origin = next((path for path in settings_paths if path.is_file()), None)
        require(origin is not None,
                "Configure a mod-manager source in the desktop or generate the showcase once before validation")
        settings = configparser.ConfigParser(interpolation=None)
        settings.read(origin, encoding="utf-8-sig")
        options = settings["options"]
        manager = options.get("mod_manager", "mo2")
        require(manager in ("mo2", "vortex"), f"Unknown saved mod manager: {manager}")
        if manager == "mo2":
            inputs = {"mo2": options.get("mo2", ""), "profile": options.get("profile", "")}
        else:
            inputs = {"vortex": options.get("vortex", "")}
            if options.get("vortex_plugins"):
                inputs["vortex_plugins"] = options["vortex_plugins"]
        if options.get("asset_cache"):
            inputs["asset_cache"] = options["asset_cache"]
    require(bool(inputs.get("mo2")) != bool(inputs.get("vortex")),
            f"Expected one configured MO2 or Vortex source in {origin}")
    require(not inputs.get("mo2") or bool(inputs.get("profile")),
            f"Configured MO2 source needs a profile in {origin}")
    require(not inputs.get("vortex_plugins") or bool(inputs.get("vortex")),
            f"Configured Vortex plugin directory needs a Vortex source in {origin}")
    return inputs, origin


def check_candidates(folder, preset):
    """Check complete CELL coverage and count uncropped generated polygons."""
    scene = showcase.check_generation(folder)
    paths = sorted(folder.glob("cells/*/candidate-navm.json"))
    if (folder / "candidate-navm.json").is_file():
        paths.append(folder / "candidate-navm.json")
    expected = {name.lower() for name in preset["cells"]}
    require(len(paths) == len(expected), "Missing or duplicate candidate exports")
    names = []
    polygons = 0
    for path in paths:
        candidate = read_json(path)
        names.append(candidate["metadata"]["selected_cell"]["editor_id"].lower())
        faces = candidate["polygons"]
        require(candidate["statistics"]["output_polygons"] == len(faces),
                f"Candidate polygon statistics mismatch in {path}")
        require(all(not face["water"] and not face["preferred_path"] for face in faces),
                f"Unexpected triangle tags in {path}")
        polygons += len(faces)
    require(set(names) == expected, "Candidate CELL selection mismatch")
    require(polygons > 0, "Empty showcase navigation")
    batch_path = folder / "batch-report.json"
    if len(expected) > 1:
        require(batch_path.is_file(), "Missing batch report")
    if batch_path.exists():
        batch = read_json(batch_path)
        require(batch["status"] == "complete" and len(batch["cells"]) == len(expected),
                "Incomplete batch")
        require(batch["generation_failed_cells"] == batch["extraction_failed_cells"] == 0,
                "Batch generation or extraction failures")
        require(not batch["asset_failures"], "Required asset failures")
        require(batch["generated_polygons"] == polygons, "Batch polygon total mismatch")
    metadata = read_json(Path(str(scene) + ".metadata.json"))["metadata"]
    cells = metadata.get("selected_cells") or [metadata["selected_cell"]]
    require(len(cells) == len(expected) and {cell["editor_id"].lower() for cell in cells} == expected,
            "Scene CELL selection mismatch")
    return scene, polygons


def check_polygon_growth(polygons, baseline, growth_percent):
    # Compare integer products so a fractional limit never rounds up to a pass.
    require(polygons * 100 <= baseline * (100 + growth_percent),
            f"Polygon growth exceeds +{growth_percent}%: {polygons} versus baseline {baseline}")


def render_showcase(scene, preset, folder, polygons):
    """Cross-check scene and rendered counts against the fresh candidate evidence."""
    meshes = renderer.read_scene(scene)
    generated = renderer.layer_triangles(meshes, "Candidate NAVM")
    require(len(generated) == polygons, "Scene/candidate polygon total mismatch")
    visible_count = len(renderer.crop_triangles(generated, preset.get("bounds")))
    require(visible_count > 0, "No generated navigation inside the showcase crop")
    attribution, evidence = showcase.scene_sources(scene)
    showcase.write_json(folder / "source-evidence.json", evidence)
    entry = {"preset": preset, "scene": str(scene), "attribution": attribution}
    images = folder / "images"
    images.mkdir()
    showcase.run_command(showcase.render_command(entry, images), images, "render")
    filename = f"{preset['id']}-navmesh-comparison.png"
    with Image.open(images / filename) as image:
        image.verify()
    manifest = read_json(images / "render-manifest.json")
    require(manifest["comparisons"][filename]["generated"] == visible_count,
            "Rendered polygon count mismatch")
    require(manifest["renders"][filename]["cell_count"] == len(preset["cells"]),
            "Rendered CELL count mismatch")
    return str(images / filename)


def validate_showcase(preset, policy, inputs, run_root):
    folder = run_root / preset["id"]
    folder.mkdir()
    baseline = policy["baseline_polygons"][preset["id"]]
    row = {"id": preset["id"], "title": preset["title"], "cells": len(preset["cells"]),
           "baseline_polygons": baseline, "polygons": None, "growth_percent": None,
           "status": "failed", "errors": [], "directory": str(folder)}
    started = time.monotonic()
    print(f"Validating {preset['title']}...", flush=True)
    try:
        source = showcase.source_arguments(inputs)
        cache = Path(inputs["asset_cache"]) if inputs.get("asset_cache") else ROOT / ".cache/prerelease-assets"
        command = showcase.generation_command(EXECUTABLE, source, preset, folder, cache)
        showcase.run_command(command, folder, "generator")
        scene, polygons = check_candidates(folder, preset)
        row.update(polygons=polygons, growth_percent=(polygons / baseline - 1) * 100)
        try:
            check_polygon_growth(polygons, baseline, policy["max_polygon_growth_percent"])
        except ValueError as error:
            row["errors"].append(str(error))
        # Retain renders for diagnosing growth failures as well as successful runs.
        row["image"] = render_showcase(scene, preset, folder, polygons)
        if not row["errors"]:
            row["status"] = "passed"
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        row["errors"].append(str(error))
    row["seconds"] = time.monotonic() - started
    count = f"{row['polygons']} polygons" if row["polygons"] is not None else "no verified count"
    print(f"{row['status'].upper()} {preset['title']}: {count}"
          + (f"; {'; '.join(row['errors'])}" if row["errors"] else ""), flush=True)
    return row


def write_report(run_root, report):
    showcase.write_json(run_root / "validation.json", report)
    lines = ["# Prerelease validation", "", f"Result: **{report['status'].upper()}**", ""]
    if report.get("input"):
        lines.extend(["Mod-manager source: `" + " ".join(showcase.source_arguments(report["input"])) + "`", ""])
    lines.extend(["| Showcase | Baseline | Polygons | Change | Result |",
                  "| --- | ---: | ---: | ---: | --- |"])
    for row in report["items"]:
        growth = f"{row['growth_percent']:+.2f}%" if row["growth_percent"] is not None else "n/a"
        lines.append(f"| {row['title']} | {row['baseline_polygons']} | {row['polygons']} | "
                     f"{growth} | {row['status']} |")
    for row in report["items"]:
        lines.extend(f"\n{row['title']}: {error}\n" for error in row["errors"])
    lines.extend(["", *report["errors"], "", f"Evidence: {run_root}"])
    (run_root / "validation-report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    showcase.write_json(OUTPUT / "latest.json", {"status": report["status"], "directory": str(run_root)})


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    os.chdir(ROOT)
    run_root = OUTPUT / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_root.mkdir(parents=True)
    report = {"status": "failed", "checked_at": datetime.now(timezone.utc).isoformat(),
              "items": [], "errors": []}
    try:
        presets = read_json(showcase.PRESETS)
        policy = load_policy(presets)
        report["policy"] = policy
        report["presets"] = presets
        inputs, origin = load_inputs()
        report["input"] = inputs
        report["input_source"] = str(origin)
        print(f"Using mod-manager source from {origin}: {showcase.source_arguments(inputs)}", flush=True)
        print("Building release desktop executable...", flush=True)
        showcase.run_command(["xmake", "f", "-m", "release"], run_root, "configure")
        showcase.run_command(["xmake", "build", "NavmeshGenerator"], run_root, "build")
        report["executable"] = str(EXECUTABLE)
        report["executable_sha256"] = hashlib.sha256(EXECUTABLE.read_bytes()).hexdigest()
        for preset in presets:
            report["items"].append(validate_showcase(preset, policy, inputs, run_root))
            write_report(run_root, report)
        if all(row["status"] == "passed" for row in report["items"]):
            report["status"] = "passed"
    except (OSError, ValueError, KeyError, TypeError, configparser.Error) as error:
        report["errors"].append(str(error))
        print(f"FAILED: {error}", flush=True)
    write_report(run_root, report)
    passed = sum(row["status"] == "passed" for row in report["items"])
    print(f"{report['status'].upper()}: {passed}/{len(report['items'])} showcases passed. "
          f"Report: {run_root / 'validation-report.md'}", flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
