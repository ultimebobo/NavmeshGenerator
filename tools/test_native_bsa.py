"""Differential regressions for the production native reader, using synthetic BSAs."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.test_bsa_index import write_bsa
from tools.bsa_index import read_entry, read_index

PROBE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else Path(
    'build/windows/x64/releasedbg/navmesh-bsa-probe.exe').resolve()


class NativeBsa(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='navmesh-native-bsa-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def probe(self, archives, names, snapshot='cache', mode='bulk', success=True):
        archive_list = self.root / 'archives.txt'
        request_list = self.root / 'requests.txt'
        archive_list.write_text('\n'.join(map(str, archives)), encoding='utf-8')
        request_list.write_text('\n'.join(map(str, names)), encoding='utf-8')
        cache = self.root / snapshot
        result = subprocess.run([str(PROBE), str(archive_list), str(cache), str(request_list), mode],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0 if success else 2, result.stderr)
        self.assertEqual(json.loads(result.stdout)['success'], success)
        return cache

    def test_versions_compression_prefixes_and_byte_equivalence(self):
        archive = self.root / 'Fixture.bsa'
        for version in (103, 104, 105):
            for default in (False, True):
                for prefixed in (False, True):
                    with self.subTest(version=version, default=default, prefixed=prefixed):
                        entries = [('plain.nif', b'plain', False), ('packed.nif', b'packed' * 1000, True),
                                   ('empty.nif', b'', True), ('unrequested.dds', bytes(1000000), False)]
                        write_bsa(archive, entries, version, default, prefixed)
                        cache = self.probe([archive], ['meshes/' + entry[0] for entry in entries[:3]],
                                           snapshot=f'{version}-{default}-{prefixed}')
                        index, metadata_bytes = read_index(archive)
                        for name, _, _ in entries[:3]:
                            self.assertEqual((cache / 'meshes' / name).read_bytes(),
                                             read_entry(archive, index, 'meshes\\' + name)[0])
                        statistics = json.loads((cache / '.archive-statistics.json').read_text())
                        self.assertEqual(statistics['metadata_bytes_read'], metadata_bytes)
                        self.assertEqual(statistics['entries_extracted'], 3)
                        self.assertLess(statistics['payload_bytes_read'], archive.stat().st_size // 10)

    def test_priority_missing_cache_and_run_scoped_index_reuse(self):
        low, high = self.root / 'Low.bsa', self.root / 'High.bsa'
        write_bsa(low, [('shared.nif', b'low', False), ('low.nif', b'low-only', False)])
        write_bsa(high, [('shared.nif', b'high', True), ('high.nif', b'high-only', True)])
        names = ['meshes/shared.nif', 'MESHES\\HIGH.NIF', 'meshes/missing.nif', 'meshes/low.nif']
        cache = self.probe([low, high], names, mode='incremental')
        self.assertEqual((cache / 'meshes/shared.nif').read_bytes(), b'high')
        before = (cache / '.archive-statistics.json').read_bytes()
        self.assertGreater(json.loads(before)['index_hits'], 0)
        self.probe([low, high], names, mode='incremental')
        self.assertEqual((cache / '.archive-statistics.json').read_bytes(), before)
        changed = self.probe([low, high], [low], mode='changed')
        self.assertEqual((changed / 'changed-models.txt').read_text().splitlines(), ['meshes/low.nif'])

    def test_unreadable_winner_never_falls_back_and_failure_is_retryable(self):
        low, high = self.root / 'Low.bsa', self.root / 'High.bsa'
        write_bsa(low, [('shared.nif', b'low', False)])
        write_bsa(high, [('shared.nif', b'high', True)])
        encoded = bytearray(high.read_bytes())
        encoded[-1] ^= 0xff
        high.write_bytes(encoded)
        cache = self.probe([low, high], ['meshes/shared.nif', 'meshes/missing.nif'], success=False)
        self.assertFalse((cache / 'meshes/shared.nif').exists())
        self.assertFalse((cache / '.missing-models.txt').exists())
        write_bsa(high, [('shared.nif', b'repaired', True)])
        self.probe([low, high], ['meshes/shared.nif', 'meshes/missing.nif'])
        self.assertEqual((cache / 'meshes/shared.nif').read_bytes(), b'repaired')

    def test_path_traversal_and_truncated_tables(self):
        archive = self.root / 'Fixture.bsa'
        write_bsa(archive, [('one.nif', b'one', False)])
        for name in ('meshes/../escape.nif', 'meshes//one.nif', 'meshes/C:bad.nif', '/meshes/one.nif'):
            self.probe([archive], [name], success=False)
        write_bsa(archive, [('../escape.nif', b'escape', False)])
        self.probe([archive], ['meshes/one.nif'], success=False)
        write_bsa(archive, [('one.nif', b'one', False)])
        archive.write_bytes(archive.read_bytes()[:-1])
        self.probe([archive], ['meshes/one.nif'], success=False)

    def test_declared_decoded_size_and_trailing_data_are_rejected(self):
        archive = self.root / 'Fixture.bsa'
        for version in (104, 105):
            for expected in (0, 4, 256 * 1024 * 1024 + 1):
                write_bsa(archive, [('one.nif', b'one', True)], version)
                data = bytearray(archive.read_bytes())
                index, _ = read_index(archive)
                offset = index['entries']['meshes\\one.nif'][0]
                struct.pack_into('<I', data, offset, expected)
                archive.write_bytes(data)
                self.probe([archive], ['meshes/one.nif'], success=False)
            write_bsa(archive, [('one.nif', b'one', True)], version)
            data = bytearray(archive.read_bytes())
            record_offset = 36 + (24 if version == 105 else 16) + 1 + len(b'meshes\0')
            size = struct.unpack_from('<I', data, record_offset + 8)[0]
            struct.pack_into('<I', data, record_offset + 8, size + 1)
            archive.write_bytes(data + b'x')
            self.probe([archive], ['meshes/one.nif'], success=False)

    def test_cli_uses_native_extraction_outside_the_project_without_python(self):
        exe = PROBE.with_name('NavmeshGenerator.exe')
        self.assertTrue(exe.is_file(), 'Build the desktop executable before this test')

        def sub(kind, payload):
            return kind.encode() + struct.pack('<H', len(payload)) + payload

        def record(kind, form, payload):
            return kind.encode() + struct.pack('<IIIII', len(payload), 0, form, 0, 44) + payload

        def group(label, kind, payload):
            return b'GRUP' + struct.pack('<IIIII', len(payload) + 24, label, kind, 0, 0) + payload

        header = record('TES4', 0, sub('HEDR', struct.pack('<fII', 1.7, 3, 0x800)))
        base = record('STAT', 0x500, sub('MODL', b'fixture.nif\0'))
        cell = record('CELL', 0x100, sub('EDID', b'FixtureInterior\0') + sub('DATA', struct.pack('<H', 1)))
        reference = record('REFR', 0x600, sub('NAME', struct.pack('<I', 0x500))
                           + sub('DATA', struct.pack('<6f', 1, 2, 3, 0, 0, 0)))
        plugin = self.root / 'Fixture.esp'
        plugin.write_bytes(header + group(int.from_bytes(b'STAT', 'little'), 0, base)
                           + group(int.from_bytes(b'CELL', 'little'), 0,
                                   cell + group(0x100, 6, group(0x100, 9, reference))))
        archive = self.root / 'Fixture.bsa'
        # An opaque model proves archive extraction reaches the shared CLI path;
        # actual supported NIF decoding is covered by the native geometry fixture.
        write_bsa(archive, [('fixture.nif', b'opaque fixture', False)])
        env = os.environ.copy()
        env['NAVMESH_PYTHON'] = str(self.root / 'absent-python.exe')
        env['PATH'] = ''
        cache_root = self.root / 'shared-cache'
        result = subprocess.run([str(exe), '--data', str(self.root), '--plugin', str(plugin),
                                 '--cell-formid', '100', '--asset-cache', str(cache_root),
                                 '--output', str(self.root / 'inspection')], cwd=self.root,
                                env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        extracted = list(cache_root.glob('*/meshes/fixture.nif'))
        self.assertEqual(len(extracted), 1)
        self.assertEqual(extracted[0].read_bytes(), b'opaque fixture')
        self.assertEqual(json.loads((extracted[0].parents[1] / '.archive-statistics.json').read_text())
                         ['entries_extracted'], 1)

    def test_filename_table_alignment_padding(self):
        archive = self.root / 'Fixture.bsa'
        write_bsa(archive, [('one.nif', b'one', False)])
        data = bytearray(archive.read_bytes())
        index, _ = read_index(archive)
        offset = index['entries']['meshes\\one.nif'][0]
        data[offset:offset] = bytes(3)
        struct.pack_into('<I', data, 28, struct.unpack_from('<I', data, 28)[0] + 3)
        record_offset = 36 + 16 + 1 + len(b'meshes\0')
        struct.pack_into('<I', data, record_offset + 12, offset + 3)
        archive.write_bytes(data)
        cache = self.probe([archive], ['meshes/one.nif'])
        self.assertEqual((cache / 'meshes/one.nif').read_bytes(), b'one')


if __name__ == '__main__':
    unittest.main()
