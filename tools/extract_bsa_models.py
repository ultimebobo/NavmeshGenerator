import argparse
from pathlib import Path
import sys


def normalize(path: object) -> str:
    return str(path).replace("/", "\\").lower()


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract requested Skyrim assets from BSA archives.")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()

    package_root = Path(__file__).resolve().parent / "BSAFileExtractor"
    sys.path.insert(0, str(package_root))
    sys.path.insert(0, str(package_root / "bethesda-structs"))
    try:
        from bethesda_structs.archive import BSAArchive
    except ImportError as error:
        print(f"BSA extraction unavailable: {error}", file=sys.stderr)
        return 2

    requested = {
        normalize(line.strip())
        for line in args.manifest.read_text(encoding="utf-8", errors="replace").splitlines()
        if line.strip() and "\ufffd" not in line and all(character.isprintable() for character in line.strip())
    }
    remaining = set(requested)
    args.output.mkdir(parents=True, exist_ok=True)

    for archive_path in sorted(args.data.glob("*.bsa")):
        if not remaining:
            break
        if not BSAArchive.can_handle(archive_path):
            continue
        try:
            archive = BSAArchive.parse_file(archive_path)
            for archive_file in archive.iter_files():
                archive_name = normalize(archive_file.filepath)
                if archive_name not in remaining:
                    continue
                destination = args.output / Path(str(archive_file.filepath).replace("\\", "/"))
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(archive_file.data)
                remaining.remove(archive_name)
        except Exception as error:
            print(f"BSA extraction: skipped {archive_path.name}: {error}", file=sys.stderr)

    print(f"BSA extraction: requested={len(requested)} extracted={len(requested) - len(remaining)} missing={len(remaining)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
