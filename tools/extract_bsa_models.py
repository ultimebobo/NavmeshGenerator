import argparse
from pathlib import Path, PureWindowsPath
import sys


def normalize(path: object) -> str:
    return str(path).replace("/", "\\").lower()


def iter_requested_files(archive: object, requested: set[str]):
    """Read only matching records; unrelated BSA entries may use other payloads."""
    header = archive.container.header
    file_index = 0
    for directory in archive.container.directory_blocks:
        directory_path = PureWindowsPath(directory.name.rstrip("\x00"))
        for record in directory.file_records:
            name = archive.container.file_names[file_index]
            file_index += 1
            filepath = directory_path / name
            if normalize(filepath) not in requested:
                continue

            size = record.size & archive.SIZE_MASK
            payload = archive.content[record.offset : record.offset + size]
            if len(payload) != size:
                raise ValueError(f"truncated BSA entry: {filepath}")
            if header.archive_flags.files_prefixed:
                if not payload or len(payload) < payload[0] + 1:
                    raise ValueError(f"invalid filename prefix: {filepath}")
                payload = payload[payload[0] + 1 :]

            compressed = bool(header.archive_flags.files_compressed) != bool(
                record.size & 0x40000000
            )
            file_struct = (
                archive.compressed_file_struct if compressed else archive.uncompressed_file_struct
            )
            yield filepath, file_struct.parse(payload).data


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
            for filepath, data in iter_requested_files(archive, remaining):
                archive_name = normalize(filepath)
                destination = args.output / Path(str(filepath).replace("\\", "/"))
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(data)
                remaining.remove(archive_name)
        except Exception as error:
            print(f"BSA extraction: skipped {archive_path.name}: {error}", file=sys.stderr)

    print(f"BSA extraction: requested={len(requested)} extracted={len(requested) - len(remaining)} missing={len(remaining)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
