"""Synthetic BSA regression fixtures; payloads contain no game assets."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

from tools.bsa_index import identity, load_index, read_entry, read_index, relative_asset, prune_cache


def write_bsa(path, entries, version=104, default_compression=False, prefixed=False):
    """Build one named folder with mixed per-entry compression and optional filename prefixes."""
    folder = b"meshes\0"
    names = b"".join(name.encode() + b"\0" for name, _, _ in entries)
    record_size = 24 if version == 105 else 16
    payload_offset = 36 + record_size + 1 + len(folder) + 16 * len(entries) + len(names)
    records = b""
    payloads = b""
    for name, data, compressed in entries:
        payload = data
        if compressed:
            if version >= 105:
                import lz4.frame
                payload = lz4.frame.compress(data)
            else:
                payload = zlib.compress(data)
            payload = struct.pack("<I", len(data)) + payload
        if prefixed:
            prefix = ("meshes\\" + name).encode()
            payload = bytes([len(prefix)]) + prefix + payload
        size = len(payload) | (0x40000000 if compressed != default_compression else 0)
        records += struct.pack("<QII", 0, size, payload_offset + len(payloads))
        payloads += payload
    flags = 3 | (4 if default_compression else 0) | (0x100 if prefixed else 0)
    header = struct.pack("<4s8I", b"BSA\0", version, 36, flags, 1, len(entries), len(folder), len(names), 1)
    folder_record = struct.pack("<QIIQ", 0, len(entries), 0, 36 + record_size) if version == 105 else struct.pack("<QII", 0, len(entries), 36 + record_size)
    path.write_bytes(header + folder_record + bytes([len(folder)]) + folder + records + names + payloads)


class BsaIndexes(unittest.TestCase):
    def test_filename_table_allows_alignment_padding(self):
        path = Path(self.temp.name) / "padding.bsa"
        write_bsa(path, [("padded.nif", b"fixture", False)])
        data = bytearray(path.read_bytes())
        name_size = struct.unpack_from("<I", data, 28)[0]
        payload = 36 + 16 + 1 + len(b"meshes\0") + 16 + name_size
        data[payload:payload] = bytes(3)
        struct.pack_into("<I", data, 28, name_size + 3)
        struct.pack_into("<I", data, 36 + 16 + 1 + len(b"meshes\0") + 12, payload + 3)
        path.write_bytes(data)
        index, _ = load_index(path, Path(self.temp.name) / "indexes")
        self.assertEqual(read_entry(path, index, "meshes\\padded.nif")[0], b"fixture")

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="navmesh-index-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_named_versions_compression_and_prefixes(self):
        for version in (103, 104, 105):
            for default in (False, True):
                for prefixed in (False, True):
                    with self.subTest(version=version, default=default, prefixed=prefixed):
                        archive = self.root / "Fixture.bsa"
                        write_bsa(archive, [("plain.nif", b"plain", False), ("packed.nif", b"packed" * 100, True),
                                            ("unrequested.dds", b"opaque" * 10000, False)], version, default, prefixed)
                        index, read_bytes = read_index(archive)
                        self.assertLess(read_bytes, archive.stat().st_size // 10)
                        self.assertEqual(set(index["entries"]), {"meshes\\plain.nif", "meshes\\packed.nif"})
                        self.assertEqual(read_entry(archive, index, "meshes\\plain.nif")[0], b"plain")
                        self.assertEqual(read_entry(archive, index, "meshes\\packed.nif")[0], b"packed" * 100)

    def test_index_reuse_and_archive_revision(self):
        archive = self.root / "Fixture.bsa"
        write_bsa(archive, [("one.nif", b"one", False)])
        original = identity(archive)
        index, first_bytes = load_index(archive, self.root / "indexes")
        self.assertGreater(first_bytes, 0)
        self.assertEqual(load_index(archive, self.root / "indexes"), (index, 0))
        write_bsa(archive, [("two.nif", b"different", False)])
        self.assertNotEqual(identity(archive), original)
        revised, bytes_read = load_index(archive, self.root / "indexes")
        self.assertGreater(bytes_read, 0)
        self.assertEqual(set(revised["entries"]), {"meshes\\two.nif"})

    def run_helper(self, *args):
        subprocess.run([sys.executable, "tools/extract_bsa_models.py", "--data", str(self.root),
                        "--output", str(self.root / "cache" / ("a" * 64)), *map(str, args)],
                       cwd=Path(__file__).resolve().parent.parent, check=True, capture_output=True, text=True)
        return self.root / "cache" / ("a" * 64)

    def test_priority_negative_cache_and_precise_archive_impacts(self):
        low, high = self.root / "Low.bsa", self.root / "High.bsa"
        write_bsa(low, [("shared.nif", b"low", False), ("low.nif", b"low only", False)])
        write_bsa(high, [("shared.nif", b"high", True), ("high.nif", b"high only", False)])
        manifest, archives, changed = self.root / "models.txt", self.root / "archives.txt", self.root / "changed.txt"
        manifest.write_text("meshes\\shared.nif\nmeshes\\missing.nif\n")
        archives.write_text(f"{low}\n{high}\n")
        changed.write_text(str(low))
        cache = self.run_helper("--manifest", manifest, "--archives", archives)
        self.assertEqual((cache / "meshes" / "shared.nif").read_bytes(), b"high")
        before = json.loads((cache / ".archive-statistics.json").read_text())
        self.run_helper("--manifest", manifest, "--archives", archives)
        self.assertEqual(json.loads((cache / ".archive-statistics.json").read_text()), before)
        changed_models = self.root / "changed-models.txt"
        self.run_helper("--archives", archives, "--changed-archives", changed, "--changed-models", changed_models)
        self.assertEqual(changed_models.read_text(), "meshes\\low.nif")

    def test_bad_paths_and_truncated_tables(self):
        for value in ("meshes\\..\\escape.nif", "C:\\escape.nif", "meshes\\x:bad.nif", "textures\\one.dds"):
            with self.assertRaises(ValueError):
                relative_asset(value)
        archive = self.root / "Broken.bsa"
        write_bsa(archive, [("one.nif", b"payload", False)])
        archive.write_bytes(archive.read_bytes()[:-1])
        with self.assertRaises(ValueError):
            read_index(archive)

    def test_pruning_preserves_foreign_files_and_active_candidates(self):
        root = self.root / "cache"
        owned = root / ("b" * 64)
        (owned / "candidates").mkdir(parents=True)
        (owned / ".navmesh-assets.json").write_text('{"schema":1}')
        (owned / "one.nif").write_bytes(bytes(1024))
        (owned / "candidates" / "one.gz").write_bytes(bytes(1024))
        foreign = root / "operator-files"
        foreign.mkdir()
        (foreign / "one.nif").write_bytes(bytes(2048))
        prune_cache(root, 0, protect_candidates=True)
        self.assertFalse((owned / "one.nif").exists())
        self.assertTrue((owned / "candidates" / "one.gz").exists())
        prune_cache(root, 0)
        self.assertFalse((owned / "candidates" / "one.gz").exists())
        self.assertTrue((foreign / "one.nif").exists())


if __name__ == "__main__":
    unittest.main()
