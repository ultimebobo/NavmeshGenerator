"""Build the release executable and validate fresh showcases without arguments."""

import argparse
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
    require(policy["plugins"] and all(isinstance(name, str) and Path(name).name == name
                                    for name in policy["plugins"]), "Expected plugin filenames")
    return policy


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


def render_showcase(scene, preset, folder, polygons, plugins):
    """Cross-check scene and rendered counts against the fresh candidate evidence."""
    meshes = renderer.read_scene(scene)
    generated = renderer.layer_triangles(meshes, "Candidate NAVM")
    require(len(generated) == polygons, "Scene/candidate polygon total mismatch")
    visible_count = len(renderer.crop_triangles(generated, preset.get("bounds")))
    require(visible_count > 0, "No generated navigation inside the showcase crop")
    attribution = "Input plugins: " + ", ".join(plugins)
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


def validate_showcase(preset, policy, data, load_order, run_root):
    folder = run_root / preset["id"]
    folder.mkdir()
    baseline = policy["baseline_polygons"][preset["id"]]
    row = {"id": preset["id"], "title": preset["title"], "cells": len(preset["cells"]),
           "baseline_polygons": baseline, "polygons": None, "growth_percent": None,
           "status": "failed", "errors": [], "directory": str(folder)}
    started = time.monotonic()
    print(f"Validating {preset['title']}...", flush=True)
    try:
        source = ["--data", str(data), "--load-order", str(load_order)]
        command = showcase.generation_command(EXECUTABLE, source, preset, folder,
                                              ROOT / ".cache/prerelease-assets")
        showcase.run_command(command, folder, "generator")
        scene, polygons = check_candidates(folder, preset)
        row.update(polygons=polygons, growth_percent=(polygons / baseline - 1) * 100)
        try:
            check_polygon_growth(polygons, baseline, policy["max_polygon_growth_percent"])
        except ValueError as error:
            row["errors"].append(str(error))
        # Retain renders for diagnosing growth failures as well as successful runs.
        row["image"] = render_showcase(scene, preset, folder, polygons, policy["plugins"])
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
    lines = ["# Prerelease validation", "", f"Result: **{report['status'].upper()}**", "",
             "| Showcase | Baseline | Polygons | Change | Result |",
             "| --- | ---: | ---: | ---: | --- |"]
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
        data_path = os.environ.get("SKYRIM_DATA_DIR")
        require(bool(data_path), "Set SKYRIM_DATA_DIR to your installed Skyrim Data directory")
        data = Path(data_path).resolve()
        for plugin in policy["plugins"]:
            require((data / plugin).is_file(), f"Missing input plugin: {data / plugin}")
        report["data_directory"] = str(data)
        report["input_plugins"] = [
            {"name": plugin, "sha256": hashlib.sha256((data / plugin).read_bytes()).hexdigest()}
            for plugin in policy["plugins"]]
        load_order = run_root / "load-order.txt"
        load_order.write_text("\n".join(policy["plugins"]) + "\n", encoding="utf-8")
        print("Building release desktop executable...", flush=True)
        showcase.run_command(["xmake", "f", "-m", "release"], run_root, "configure")
        showcase.run_command(["xmake", "build", "NavmeshGenerator"], run_root, "build")
        report["executable"] = str(EXECUTABLE)
        report["executable_sha256"] = hashlib.sha256(EXECUTABLE.read_bytes()).hexdigest()
        for preset in presets:
            report["items"].append(validate_showcase(preset, policy, data, load_order, run_root))
            write_report(run_root, report)
        if all(row["status"] == "passed" for row in report["items"]):
            report["status"] = "passed"
    except (OSError, ValueError, KeyError, TypeError) as error:
        report["errors"].append(str(error))
        print(f"FAILED: {error}", flush=True)
    write_report(run_root, report)
    passed = sum(row["status"] == "passed" for row in report["items"])
    print(f"{report['status'].upper()}: {passed}/{len(report['items'])} showcases passed. "
          f"Report: {run_root / 'validation-report.md'}", flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
