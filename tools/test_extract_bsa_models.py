import unittest
from types import SimpleNamespace

from tools.extract_bsa_models import iter_requested_files


class RecordingStruct:
    def __init__(self):
        self.payloads = []

    def parse(self, payload):
        self.payloads.append(payload)
        return SimpleNamespace(data=payload)


class RequestedBsaFilesTest(unittest.TestCase):
    def test_skips_unrequested_payloads_and_decodes_each_record_independently(self):
        raw = RecordingStruct()
        compressed = RecordingStruct()
        # The first entry stands in for a texture whose bytes cannot be decoded
        # as an LZ4 frame.
        content = b"\x00bad!\x00plain\x04namepacked"
        archive = SimpleNamespace(
            SIZE_MASK=0x3FFFFFFF,
            content=content,
            container=SimpleNamespace(
                header=SimpleNamespace(
                    archive_flags=SimpleNamespace(files_compressed=True, files_prefixed=True)
                ),
                directory_blocks=[SimpleNamespace(
                    name="meshes\x00",
                    file_records=[
                        SimpleNamespace(size=5, offset=0),
                        SimpleNamespace(size=0x40000000 | 6, offset=5),
                        SimpleNamespace(size=11, offset=11),
                    ],
                )],
                file_names=["texture.dds", "plain.nif", "packed.nif"],
            ),
            uncompressed_file_struct=raw,
            compressed_file_struct=compressed,
        )

        files = list(iter_requested_files(archive, {"meshes\\plain.nif", "meshes\\packed.nif"}))

        self.assertEqual([(str(path), data) for path, data in files], [
            ("meshes\\plain.nif", b"plain"),
            ("meshes\\packed.nif", b"packed"),
        ])
        self.assertEqual(raw.payloads, [b"plain"])
        self.assertEqual(compressed.payloads, [b"packed"])


if __name__ == "__main__":
    unittest.main()
