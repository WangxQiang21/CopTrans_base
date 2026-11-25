"""Recover committed native bridge takes after the bridge process has stopped."""
import csv
import os
from pathlib import Path
import struct
import tempfile

HEADER = struct.Struct('<8sQQ')
SLOT = struct.Struct('<Qqqqqqddd')
FIELDS = ('robot', 'observed_steady_ns', 'taken_realtime_ns', 'source_realtime_ns',
          'received_realtime_ns', 'vx_mps', 'vy_mps', 'wz_radps', 'trace_overflow')


def convert(path):
    target = path.with_suffix('.csv')
    with path.open('rb') as src:
        raw = src.read(HEADER.size)
        if len(raw) != HEADER.size:
            raise ValueError(f'Truncated bridge journal header: {path.name}')
        magic, capacity, count = HEADER.unpack(raw)
        if magic != b'CTTAKE01' or capacity != 65536 or path.stat().st_size != HEADER.size + capacity * SLOT.size:
            raise ValueError(f'Invalid bridge journal: {path.name}')
        if count > capacity:
            raise ValueError(f'Overflow in {path.name}: {count - capacity} takes')
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix=path.name, delete=False,
                                         newline='', encoding='ascii') as out:
            temporary = Path(out.name)
            try:
                writer = csv.writer(out, lineterminator='\n')
                writer.writerow(FIELDS)
                for i in range(count):
                    committed, robot, steady, realtime, source, received, vx, vy, wz = SLOT.unpack(src.read(SLOT.size))
                    if committed != i + 1 or robot not in (1, 2, 3):
                        raise ValueError(f'Incomplete bridge take {i+1}/{count} in {path.name}')
                    writer.writerow((f'robot{robot}', steady, realtime, source, received, vx, vy, wz, 0))
                out.flush()
                os.fsync(out.fileno())
            except BaseException:
                temporary.unlink(missing_ok=True)
                raise
    temporary.replace(target)
    path.unlink()
    return target


def finalize(directory):
    """Old probe builds already wrote CSV. New builds leave an auditable shared journal."""
    return [convert(p) for p in sorted(directory.glob('bridge_take_*.journal'))]


def coverage(directory, robots):
    found = {}
    for path in sorted(directory.glob('bridge_take_*.csv')):
        with path.open(newline='') as stream:
            rows = csv.DictReader(stream)
            if tuple(rows.fieldnames or ()) != FIELDS:
                raise ValueError(f'Invalid bridge header in {path.name}')
            for row in rows:
                name = row['robot']
                if name not in robots or int(row['trace_overflow']):
                    raise ValueError(f'Unexpected or overflowing bridge trace in {path.name}')
                found[name] = found.get(name, 0) + 1
    if set(found) != set(robots):
        raise ValueError(f'Missing DDS bridge takes for {sorted(set(robots)-set(found))}')
    return found
