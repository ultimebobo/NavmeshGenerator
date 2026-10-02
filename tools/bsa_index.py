"""Versioned metadata indexes and bounded reads for named desktop Skyrim BSAs.

The directory tables are read once. Asset payloads are sought independently;
archive precedence is resolved before decompressing a requested entry.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import struct
import zlib

SCHEMA = 1
MAX_ASSET_BYTES = 256 * 1024 * 1024


def normalize(value: object) -> str:
    return str(value).replace("/", "\\").lower()


def relative_asset(value: object) -> Path:
    """Reject archive names that could escape the extraction directory."""
    parts = normalize(value).split("\\")
    if not parts or parts[0] != "meshes" or any(part in ("", ".", "..") or ":" in part for part in parts):
        raise ValueError(f"unsafe model path: {value}")
    if not parts[-1].endswith(".nif"):
        raise ValueError(f"unsupported model path: {value}")
    return Path(*parts)


def identity(path: Path) -> str:
    status = path.stat()
    value = json.dumps([SCHEMA, str(path.resolve()).lower(), status.st_size, status.st_mtime_ns])
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    try:
        temporary.write_text(json.dumps(value, separators=(",", ":")), encoding="utf-8")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _read(stream, count: int) -> bytes:
    if count < 0 or count > MAX_ASSET_BYTES:
        raise ValueError("unsafe BSA table or entry size")
    result = stream.read(count)
    if len(result) != count:
        raise ValueError("truncated BSA table or entry")
    return result


def read_index(path: Path) -> tuple[dict, int]:
    """Return NIF entries and metadata bytes read, without reading asset payloads."""
    file_size = path.stat().st_size
    with path.open("rb") as stream:
        magic, version, offset, flags, folders, files, folder_names, name_bytes, _ = struct.unpack("<4s8I", _read(stream, 36))
        if magic != b"BSA\0" or version not in (103, 104, 105):
            raise ValueError("unsupported BSA header")
        if flags & 0x40 or flags & 3 != 3:
            raise ValueError("BSA needs desktop directory and file names")
        record_size = 24 if version == 105 else 16
        if offset < 36 or folders * record_size + files * 16 + name_bytes > file_size - offset:
            raise ValueError("BSA counts exceed archive size")
        stream.seek(offset)
        counts = []
        for _ in range(folders):
            entry = _read(stream, record_size)
            counts.append(struct.unpack_from("<I", entry, 8)[0])
        if sum(counts) != files:
            raise ValueError("BSA folder and file counts disagree")
        records = []
        actual_folder_names = 0
        for count in counts:
            length = _read(stream, 1)[0]
            name = _read(stream, length)
            actual_folder_names += length
            if not name.endswith(b"\0"):
                raise ValueError("unterminated BSA folder name")
            directory = name[:-1].decode("utf-8", errors="strict")
            for _ in range(count):
                _, size, position = struct.unpack("<QII", _read(stream, 16))
                if position > file_size or size & 0x3fffffff > file_size - position:
                    raise ValueError("BSA entry exceeds archive size")
                records.append((directory, size, position))
        if actual_folder_names != folder_names:
            raise ValueError("BSA folder name lengths disagree")
        names = _read(stream, name_bytes)
        filenames = names.split(b"\0")
        # Some desktop archives pad the filename table with zero bytes before payload data.
        if len(filenames) < files + 1 or any(filenames[files:]):
            raise ValueError("BSA filename table disagrees with file count")
        entries = {}
        for (directory, size, position), name in zip(records, filenames[:files]):
            logical = normalize(directory + "\\" + name.decode("utf-8", errors="strict"))
            if not logical.endswith(".nif") or not logical.startswith("meshes\\"):
                continue
            relative_asset(logical)
            entries[logical] = [position, size & 0x3fffffff, bool(flags & 4) != bool(size & 0x40000000)]
        return {"schema": SCHEMA, "version": version, "prefixed": bool(flags & 0x100), "entries": entries}, stream.tell() - offset + 36


def load_index(path: Path, cache: Path) -> tuple[dict, int]:
    cached = cache / (identity(path) + ".json")
    try:
        result = json.loads(cached.read_text(encoding="utf-8"))
        if result["schema"] == SCHEMA and isinstance(result["entries"], dict):
            cached.touch()
            return result, 0
    except (OSError, ValueError, KeyError):
        pass
    result, read_bytes = read_index(path)
    atomic_json(cached, result)
    return result, read_bytes


def read_entry(path: Path, index: dict, logical: str) -> tuple[bytes, int]:
    offset, size, compressed = index["entries"][logical]
    if offset < 0 or size < 0 or size > MAX_ASSET_BYTES:
        raise ValueError(f"unsafe BSA payload size: {logical}")
    with path.open("rb") as stream:
        stream.seek(offset)
        payload = _read(stream, size)
    if index["prefixed"]:
        if not payload or len(payload) <= payload[0]:
            raise ValueError(f"invalid BSA filename prefix: {logical}")
        payload = payload[payload[0] + 1:]
    if not compressed:
        return payload, size
    if len(payload) < 4:
        raise ValueError(f"missing BSA decoded size: {logical}")
    expected = struct.unpack_from("<I", payload)[0]
    if expected > MAX_ASSET_BYTES:
        raise ValueError(f"unsafe BSA decoded size: {logical}")
    if index["version"] >= 105:
        import lz4.frame
        decoder = lz4.frame.LZ4FrameDecompressor()
        output = decoder.decompress(payload[4:], max_length=expected + 1)
        if not decoder.eof or decoder.unused_data:
            raise ValueError(f"incomplete BSA LZ4 frame: {logical}")
    else:
        decoder = zlib.decompressobj()
        output = decoder.decompress(payload[4:], expected + 1)
        if not decoder.eof or decoder.unused_data:
            raise ValueError(f"incomplete BSA zlib stream: {logical}")
    if len(output) != expected:
        raise ValueError(f"BSA decoded size mismatch: {logical}")
    return output, size


def prune_cache(root: Path, budget: int, protected: Path | None = None, protect_candidates: bool = False) -> int:
    """Evict only marked snapshots and schema-owned index files, oldest first."""
    candidates = []
    if not root.exists():
        return 0
    for child in root.iterdir():
        if child.is_dir() and not child.is_symlink() and len(child.name) == 64 and all(c in "0123456789abcdef" for c in child.name) and (child / ".navmesh-assets.json").is_file():
            for path in child.rglob("*"):
                if path.is_file() and not path.is_symlink():
                    if any(parent.is_symlink() for parent in path.parents if parent != root):
                        continue
                    if path.suffix not in (".nif", ".gz", ".json", ".txt") or path.name.endswith(".tmp"):
                        continue
                    retained = child == protected or (protect_candidates and "candidates" in path.relative_to(child).parts)
                    candidates.append((path.stat().st_mtime_ns, path, path.stat().st_size, retained))
        elif child.name in (".indexes", ".mo2") and child.is_dir() and not child.is_symlink():
            pattern = "*.json" if child.name == ".indexes" else "loose-assets-*.tsv"
            for path in child.glob(pattern):
                if path.is_file() and not path.is_symlink():
                    candidates.append((path.stat().st_mtime_ns, path, path.stat().st_size, False))
    total = sum(item[2] for item in candidates)
    for _, path, size, retained in sorted(candidates):
        if total <= budget:
            break
        if retained or path.name == ".navmesh-assets.json":
            continue
        path.unlink(missing_ok=True)
        total -= size
    return total
