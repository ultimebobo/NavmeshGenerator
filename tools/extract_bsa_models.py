import argparse
from pathlib import Path
import sys


def main() -> int:
    import json
    import os
    from bsa_index import atomic_json, load_index, normalize, read_entry, relative_asset, prune_cache

    parser = argparse.ArgumentParser(description="Extract requested Skyrim NIFs using persistent BSA indexes.")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--archives", type=Path, help="Archive paths in increasing MO2 priority")
    parser.add_argument("--changed-archives", type=Path, help="Write model names whose winning provider is in this list")
    parser.add_argument("--changed-models", type=Path)
    parser.add_argument("--budget-bytes", type=int)
    parser.add_argument("--prune-only", action="store_true")
    parser.add_argument("--protect-candidates", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    root = args.output.parent
    marker = args.output / ".navmesh-assets.json"
    if not marker.exists():
        atomic_json(marker, {"schema": 1})
    if args.prune_only:
        if args.budget_bytes is None or args.budget_bytes < 0:
            parser.error("--prune-only requires a nonnegative --budget-bytes")
        print(f"Asset cache retained bytes: {prune_cache(root, args.budget_bytes, protect_candidates=args.protect_candidates)}")
        return 0
    if not args.manifest and not args.changed_archives:
        parser.error("--manifest or --changed-archives is required")
    archives = ([Path(line) for line in args.archives.read_text(encoding="utf-8").splitlines() if line]
                if args.archives else sorted(args.data.glob("*.bsa")))
    requested = {normalize(line.strip()) for line in args.manifest.read_text(encoding="utf-8").splitlines()
                 if line.strip()} if args.manifest else set()
    for logical in requested:
        relative_asset(logical)
    missing_path = args.output / ".missing-models.txt"
    missing = set(missing_path.read_text(encoding="utf-8").splitlines()) if missing_path.exists() else set()
    remaining = {logical for logical in requested if logical not in missing
                 and not (args.output / relative_asset(logical)).is_file()}
    changed = {str(Path(line).resolve()).lower() for line in args.changed_archives.read_text(encoding="utf-8").splitlines()
               if line} if args.changed_archives else set()
    changed_models = set()
    seen = set()
    stats = {"metadata_bytes_read": 0, "payload_bytes_read": 0, "entries_extracted": 0, "index_hits": 0}
    reliable = True
    for path in reversed(archives):
        if not remaining and not changed:
            break
        try:
            index, count = load_index(path, root / ".indexes")
            stats["metadata_bytes_read"] += count
            stats["index_hits"] += int(count == 0)
        except Exception as error:
            reliable = False
            print(f"BSA extraction: skipped {path.name}: {error}", file=sys.stderr)
            continue
        if changed:
            winners = set(index["entries"]) - seen
            if str(path.resolve()).lower() in changed:
                changed_models.update(winners)
            seen.update(index["entries"])
        for logical in sorted(remaining.intersection(index["entries"])):
            # An unreadable winning entry never falls back to a lower-priority provider.
            remaining.remove(logical)
            try:
                data, count = read_entry(path, index, logical)
                destination = args.output / relative_asset(logical)
                destination.parent.mkdir(parents=True, exist_ok=True)
                temporary = destination.with_name(destination.name + f".{os.getpid()}.tmp")
                temporary.write_bytes(data)
                temporary.replace(destination)
                stats["payload_bytes_read"] += count
                stats["entries_extracted"] += 1
            except Exception as error:
                reliable = False
                print(f"BSA extraction: unreadable winner {path.name}/{logical}: {error}", file=sys.stderr)
    if reliable:
        missing.update(remaining)
        missing_path.write_text("\n".join(sorted(missing)), encoding="utf-8")
    if args.changed_models:
        args.changed_models.write_text("\n".join(sorted(changed_models)), encoding="utf-8")
    statistics_path = args.output / ".archive-statistics.json"
    try:
        accumulated = json.loads(statistics_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        accumulated = {}
    atomic_json(statistics_path, {key: accumulated.get(key, 0) + value for key, value in stats.items()})
    print(f"BSA extraction: requested={len(requested)} extracted={stats['entries_extracted']} missing={len(remaining)} "
          f"metadata_bytes_read={stats['metadata_bytes_read']} payload_bytes_read={stats['payload_bytes_read']}")
    return 0 if reliable else 2


if __name__ == "__main__":
    raise SystemExit(main())
