"""Compare selective Python/native extraction using read-only local game archives.

Only requested payloads are copied into a temporary workspace. No game assets
are retained by this script after completion; the report contains counters and hashes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.bsa_index import load_index, read_entry, relative_asset


def timed(operation):
    start = time.perf_counter()
    value = operation()
    return value, time.perf_counter() - start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--requests', type=int, default=40)
    parser.add_argument('--repeats', type=int, default=5)
    args = parser.parse_args()
    if args.requests < 1 or args.repeats < 1:
        parser.error('requests and repeats must be positive')
    archives = sorted(path.resolve() for path in args.data.iterdir() if path.suffix.lower() == '.bsa')
    probe = args.probe.resolve()
    helper = Path(__file__).with_name('extract_bsa_models.py')
    report = {'schema': 'navmesh-generator/bsa-performance', 'archives': len(archives),
              'archive_bytes': sum(path.stat().st_size for path in archives), 'repeats': args.repeats,
              'methodology': 'OS file cache is not flushed. Cold/warm refer to generated metadata indexes. '
                             'Python extraction includes helper startup; native timings exclude probe startup, '
                             'matching the in-process application path. Stage timings exclude NIF decoding and Recast.'}
    with tempfile.TemporaryDirectory(prefix='navmesh-bsa-benchmark-') as temporary:
        root = Path(temporary)
        archive_list = root / 'archives.txt'
        archive_list.write_text('\n'.join(map(str, archives)), encoding='utf-8')
        winners = {}
        for archive in archives:
            index, _ = load_index(archive, root / 'reference-indexes')
            for name in index['entries']:
                winners[name] = (archive, index)
        if not winners:
            raise RuntimeError('No indexed model entries in the supplied archives')
        all_names = sorted(winners)
        names = all_names[::max(1, len(all_names) // args.requests)][:args.requests]
        request_list = root / 'requests.txt'
        request_list.write_text('\n'.join(names), encoding='utf-8')
        report['winning_models'] = len(winners)
        report['requested_models'] = len(names)
        reference = {}
        payload_bytes = decoded_bytes = 0
        for name in names:
            payload, count = read_entry(*winners[name], name)
            reference[name] = payload
            payload_bytes += count
            decoded_bytes += len(payload)
        report['payload_bytes'] = payload_bytes
        report['decoded_bytes'] = decoded_bytes
        rows = []

        def native(snapshot, mode, requests=request_list):
            completed = subprocess.run([str(probe), str(archive_list), str(snapshot), str(requests), mode],
                                       check=True, capture_output=True, text=True)
            return json.loads(completed.stdout)['seconds']

        def python(snapshot, requests):
            subprocess.run([sys.executable, str(helper), '--data', str(args.data.resolve()), '--output', str(snapshot),
                            '--archives', str(archive_list), '--manifest', str(requests)],
                           check=True, capture_output=True)

        def verify(snapshot):
            for name, payload in reference.items():
                if (snapshot / relative_asset(name)).read_bytes() != payload:
                    raise AssertionError('Native/Python payload mismatch')

        for repeat in range(args.repeats):
            row = {'repeat': repeat}
            python_root = root / f'python-{repeat}'
            native_root = root / f'native-{repeat}'
            _, row['python_index_cold_seconds'] = timed(
                lambda: [load_index(path, python_root / '.indexes') for path in archives])
            _, row['python_index_warm_seconds'] = timed(
                lambda: [load_index(path, python_root / '.indexes') for path in archives])
            row['native_index_seconds'] = native(native_root / 'index', 'index')
            _, row['python_payload_only_seconds'] = timed(
                lambda: [read_entry(*winners[name], name) for name in names])

            def write_payloads():
                for name, payload in reference.items():
                    destination = root / f'write-{repeat}' / relative_asset(name)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(payload)
            _, row['decoded_file_write_seconds'] = timed(write_payloads)
            _, row['python_bulk_warm_indexes_seconds'] = timed(lambda: python(python_root / 'bulk', request_list))

            def incremental():
                for name in names:
                    single = root / 'single.txt'
                    single.write_text(name, encoding='utf-8')
                    python(python_root / 'incremental', single)
            _, row['python_incremental_warm_indexes_seconds'] = timed(incremental)
            row['native_bulk_seconds'] = native(native_root / 'bulk', 'bulk')
            row['native_incremental_seconds'] = native(native_root / 'incremental', 'incremental')
            row['native_cached_seconds'] = native(native_root / 'incremental', 'incremental')
            verify(python_root / 'bulk')
            verify(python_root / 'incremental')
            verify(native_root / 'bulk')
            verify(native_root / 'incremental')
            native(native_root / 'changed', 'changed', archive_list)
            actual = (native_root / 'changed' / 'changed-models.txt').read_text(encoding='utf-8').splitlines()
            if actual != sorted(name.replace('\\', '/') for name in winners):
                raise AssertionError('Native/Python winning directory index mismatch')
            rows.append(row)
        report['runs'] = rows
        report['medians'] = {key: statistics.median(row[key] for row in rows)
                             for key in rows[0] if key.endswith('_seconds')}
        medians = report['medians']
        report['incremental_speedup'] = medians['python_incremental_warm_indexes_seconds'] / medians['native_incremental_seconds']
        report['bulk_speedup'] = medians['python_bulk_warm_indexes_seconds'] / medians['native_bulk_seconds']
        report['all_payloads_identical'] = True
        report['all_winning_names_identical'] = True
        digest = hashlib.sha256()
        for name in names:
            digest.update(name.encode('utf-8'))
            digest.update(hashlib.sha256(reference[name]).digest())
        report['sample_digest'] = digest.hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report['medians'], indent=2))
    print(f"Incremental speedup: {report['incremental_speedup']:.1f}x; bulk: {report['bulk_speedup']:.1f}x")


if __name__ == '__main__':
    main()
