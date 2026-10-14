"""Regenerate showcase GLBs and comparisons, or rerender their saved exports.

Requires the same Python packages as render_media.py. Game inputs are read by
NavmeshGenerator's CLI; only run evidence and images are written to the output.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PRESETS = Path(__file__).with_name("showcase-scenes.json")
RENDERER = Path(__file__).with_name("render_media.py")
SHOWCASE_NOTE = "Triangle tagging disabled; review and test NPC navigation before use."
OFFICIAL_PLUGINS = {"skyrim.esm", "update.esm", "dawnguard.esm", "hearthfires.esm", "dragonborn.esm"}


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def run_command(command, folder, stage):
    """Retain the exact argv, combined log and exit status for each CLI stage."""
    started = time.monotonic()
    log_path = folder / f"{stage}.log"
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    write_json(folder / f"{stage}-invocation.json", {
        "command": command, "exit_code": result.returncode,
        "working_directory": str(Path.cwd()),
        "seconds": time.monotonic() - started,
    })
    if result.returncode:
        raise ValueError(f"{stage} failed; see {log_path}")


def generation_command(executable, source, preset, folder, asset_cache=None):
    """Generate previews through the shared Cell run path without writing plugins."""
    command = [str(executable), *source, "--cells", ",".join(preset["cells"]),
               "--generate-candidate", "--make-scene", "--no-triangle-tagging",
               "--batch-output", "full", "--geometry-layers", "navmesh,terrain,collision",
               "--neighboring-cell-radius", "0", "--output", str(folder)]
    if asset_cache:
        command.extend(["--asset-cache", str(asset_cache.resolve())])
    return command


def check_generation(folder):
    """Reject skipped generation and invalid or tagged candidates before rendering."""
    batch_path = folder / "batch-report.json"
    if batch_path.exists():
        report = json.loads(batch_path.read_text(encoding="utf-8"))
        failures = [cell for cell in report["cells"] if cell["status"] != "generated"]
        if failures:
            raise ValueError(f"Showcase generation skipped cells; see {batch_path}")
    candidates = list(folder.glob("cells/*/candidate-navm.json"))
    single = folder / "candidate-navm.json"
    if single.exists():
        candidates.append(single)
    if not candidates:
        raise ValueError(f"No candidate evidence in {folder}")
    for path in candidates:
        candidate = json.loads(path.read_text(encoding="utf-8"))
        if not candidate["topology"]["valid"] or candidate["triangle_tagging"] is not False:
            raise ValueError(f"Invalid topology or enabled triangle tagging in {path}")
    scene = folder / "scene.glb"
    if not scene.is_file() or not Path(str(scene) + ".metadata.json").is_file():
        raise ValueError(f"Missing combined scene or CELL metadata in {folder}")
    return scene


def scene_sources(scene):
    """Audit winning CELL, NAVM and support records rather than all active plugins.

    Scene provenance identifies the records and model paths used in the export.
    The resolved record report supplies their effective plugin winners; the input
    report identifies loose model replacements actually referenced by the scene.
    """
    folder = scene.parent
    provenance_path = Path(str(scene) + ".provenance.json")
    record_path = folder / "load-order.json"
    input_path = folder / "input-report.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    metadata = provenance["metadata"]
    cells = metadata.get("selected_cells") or [metadata["selected_cell"]]
    selected_ids = {cell["form_id"] for cell in cells}
    record_ids = set(selected_ids)
    authored_ids = set()
    models = set()
    for obj in provenance["objects"]:
        source = obj["provenance"]
        if obj["layer"] == "Original NAVM (current cell)":
            authored_ids.add(source["navmeshFormId"])
        elif obj["layer"] in ("Terrain", "Collision"):
            if source.get("formId"):
                record_ids.add(source["formId"])
            if source.get("model"):
                model = source["model"].replace("\\", "/").lower()
                models.add(model if model.startswith("meshes/") else "meshes/" + model)
    record_ids.update(authored_ids)
    records = {record["form_id"]: record for record in json.loads(record_path.read_text(encoding="utf-8"))["records"]
               if record["form_id"] in record_ids}
    missing = record_ids - records.keys()
    if missing:
        raise ValueError(f"Missing winning scene records in {record_path}: {', '.join(sorted(missing))}")
    inputs = json.loads(input_path.read_text(encoding="utf-8"))
    plugins = sorted({record["winning_plugin"] for record in records.values()})
    assets = sorted({winner["source"] for winner in inputs.get("loose_asset_winners", [])
                     if winner["logical_path"].replace("\\", "/").lower() in models
                     and winner["source"] not in ("game Data", "deployed Data")})
    mods = [plugin for plugin in plugins if plugin.lower() not in OFFICIAL_PLUGINS]
    caption = "Winning scene records: " + (" + ".join(mods) if mods else "official game plugins")
    if assets:
        caption += " • Loose assets: " + " + ".join(assets)
    evidence = {
        "selected_cells": [records[identity] for identity in sorted(selected_ids)],
        "authored_navm": [records[identity] for identity in sorted(authored_ids)],
        "record_plugins": plugins, "loose_asset_sources": assets,
        "input_snapshot_hash": inputs["snapshot_hash"],
        "reports": {path.name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    for path in (provenance_path, record_path, input_path)},
    }
    return caption, evidence


def generation_inputs(args, saved):
    """Reuse the selected live mod-manager source, never manufacture a filtered profile."""
    if args.mo2:
        inputs = {"mo2": str(args.mo2.resolve()), "profile": args.profile}
    elif args.vortex:
        inputs = {"vortex": str(args.vortex.resolve())}
    else:
        inputs = dict(saved or {})
    if not (inputs.get("mo2") or inputs.get("vortex")):
        raise ValueError("Generation requires --mo2 or --vortex for the first run")
    if args.profile:
        inputs["profile"] = args.profile
    if inputs.get("mo2") and not inputs.get("profile"):
        raise ValueError("--mo2 requires --profile")
    if inputs.get("profile") and not inputs.get("mo2"):
        raise ValueError("--profile requires --mo2")
    if args.vortex_plugins:
        inputs["vortex_plugins"] = str(args.vortex_plugins.resolve())
    if inputs.get("vortex_plugins") and not inputs.get("vortex"):
        raise ValueError("--vortex-plugins requires --vortex")
    if args.asset_cache:
        inputs["asset_cache"] = str(args.asset_cache.resolve())
    if args.attribution:
        inputs["attribution"] = args.attribution
    return inputs


def source_arguments(inputs):
    if inputs.get("mo2"):
        return ["--mo2", inputs["mo2"], "--profile", inputs["profile"]]
    command = ["--vortex", inputs["vortex"]]
    if inputs.get("vortex_plugins"):
        command.extend(["--vortex-plugins", inputs["vortex_plugins"]])
    return command


def render_command(entry, folder):
    """Use one recipe for fresh exports and subsequent presentation-only reruns."""
    preset = entry["preset"]
    metadata = json.loads(Path(entry["scene"] + ".metadata.json").read_text(encoding="utf-8"))["metadata"]
    cells = metadata.get("selected_cells") or [metadata.get("selected_cell")]
    actual = {cell["editor_id"].lower() for cell in cells if cell}
    if actual != {cell.lower() for cell in preset["cells"]}:
        raise ValueError(f"Scene CELL selection does not match {preset['id']}; regenerate the scene")
    command = [sys.executable, str(RENDERER), "--comparison", entry["scene"],
               "--output", str(folder), "--filename", f"{preset['id']}-navmesh-comparison.png",
               "--title", f"{preset['title']}: authored vs generated",
               "--attribution", entry["attribution"],
               "--note", preset.get("note", entry.get("note", SHOWCASE_NOTE)),
               "--view", preset["view"], "--azimuth", str(preset["azimuth"]),
               "--elevation", str(preset["elevation"]), "--cell-count", str(len(actual))]
    if preset.get("bounds"):
        command.extend(["--bounds", *map(str, preset["bounds"])])
    return command


def write_gallery(output, entries):
    """Link only completed renders; game geometry remains outside the gallery."""
    cards = []
    for entry in entries.values():
        if not entry.get("image"):
            continue
        image = html.escape(entry["image"], quote=True)
        title = html.escape(entry["preset"]["title"])
        cards.append(f'<article><h2>{title}</h2><a href="{image}">'
                     f'<img src="{image}" alt="{title}: authored and generated navmesh"></a></article>')
    (output / "gallery.html").write_text(
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<title>NavmeshGenerator showcase</title><style>'
        'body{background:#10171f;color:#e8f0f4;font:18px Segoe UI,sans-serif;margin:32px}'
        'main{display:grid;grid-template-columns:repeat(auto-fit,minmax(480px,1fr));gap:24px}'
        'img{width:100%}h2{font-size:22px}</style></head><body>'
        '<h1>NavmeshGenerator showcase</h1><p>Direct program exports. Triangle tagging disabled. '
        'Click an image for full resolution.</p><main>' + "".join(cards) + '</main></body></html>\n',
        encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "output/showcase")
    parser.add_argument("--exe", type=Path, default=ROOT / "build/windows/x64/release/NavmeshGenerator.exe")
    parser.add_argument("--presets", type=Path, default=PRESETS)
    parser.add_argument("--only", nargs="+", help="Preset IDs to remake; defaults to all presets")
    parser.add_argument("--list", action="store_true", help="List preset IDs and selected CELL editor IDs")
    parser.add_argument("--render-only", action="store_true", help="Reuse scenes recorded in showcase-manifest.json")
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument("--mo2", type=Path)
    source_group.add_argument("--vortex", type=Path)
    parser.add_argument("--profile")
    parser.add_argument("--vortex-plugins", type=Path)
    parser.add_argument("--asset-cache", type=Path)
    parser.add_argument("--attribution", help="Optional caption override; defaults to audited winning scene sources")
    args = parser.parse_args()
    presets = json.loads(args.presets.read_text(encoding="utf-8"))
    if args.list:
        for preset in presets:
            print(f"{preset['id']}: {', '.join(preset['cells'])}")
        return
    identities = {preset["id"] for preset in presets}
    if args.only and not set(args.only) <= identities:
        parser.error("Unknown preset IDs; use --list")
    selected = [preset for preset in presets if not args.only or preset["id"] in args.only]
    if not selected:
        parser.error("No showcase presets selected")
    output = args.output.resolve()
    manifest_path = output / "showcase-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {"scenes": {}}
    entries = manifest["scenes"]
    inputs = None
    if not args.render_only:
        try:
            inputs = generation_inputs(args, manifest.get("input"))
        except ValueError as error:
            parser.error(str(error))
        if not args.exe.is_file():
            parser.error(f"Build the desktop executable first: {args.exe}")
    if args.render_only:
        if args.mo2 or args.vortex or args.profile or args.vortex_plugins or args.asset_cache:
            parser.error("--render-only reuses saved geometry; omit it to apply mod-manager input changes")
        for preset in selected:
            entry = entries.get(preset["id"])
            if not entry or not Path(entry["scene"]).is_file():
                parser.error(f"No saved scene for {preset['id']}; generate it first")
    output.mkdir(parents=True, exist_ok=True)
    if inputs:
        manifest["input"] = inputs
        write_json(manifest_path, manifest)
    run_root = output / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    try:
        for preset in selected:
            identity = preset["id"]
            if not args.render_only:
                folder = run_root / identity
                folder.mkdir(parents=True)
                print(f"Generating {preset['title']}...", flush=True)
                cache = Path(inputs["asset_cache"]) if inputs.get("asset_cache") else None
                command = generation_command(args.exe.resolve(), source_arguments(inputs), preset, folder, cache)
                run_command(command, folder, "generator")
                scene = check_generation(folder)
                attribution, evidence = scene_sources(scene)
                entries[identity] = {"preset": preset, "scene": str(scene),
                                     "attribution": inputs.get("attribution") or attribution,
                                     "source_evidence": evidence, "input": inputs.copy(),
                                     "generator_command": command}
                write_json(manifest_path, manifest)
            entry = entries[identity]
            if args.render_only and args.attribution:
                entry["attribution"] = args.attribution
            # Reuse the recorded CELL selection, while allowing the current recipe
            # to change camera presentation without changing the source scene.
            if preset["cells"] != entry["preset"]["cells"]:
                raise ValueError(f"CELL selection changed for {identity}; regenerate the scene")
            entry["preset"] = preset
            folder = output / "images" / identity
            folder.mkdir(parents=True, exist_ok=True)
            print(f"Rendering {preset['title']}...", flush=True)
            command = render_command(entry, folder)
            run_command(command, folder, "render")
            entry["image"] = f"images/{identity}/{identity}-navmesh-comparison.png"
            entry["render_manifest"] = f"images/{identity}/render-manifest.json"
            write_json(manifest_path, manifest)
            write_gallery(output, entries)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"{error}\n")
    print(f"Showcase gallery: {output / 'gallery.html'}", flush=True)


if __name__ == "__main__":
    main()
