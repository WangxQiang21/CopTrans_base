#!/usr/bin/env python3
"""Build the machine-readable E09 scan matrix from the frozen scan logs."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent

BATCHES = [
    ('batch_01_small_obstacles.json', 7, 12),
    ('batch_02_wide_obstacles.json', 13, 18),
    ('batch_03_over_barrier.json', 19, 24),
    ('batch_04_low_barrier.json', 25, 30),
]


def load_summary(label):
    path = ROOT / (label + '.log')
    if not path.exists():
        return None
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('SUMMARY '):
            return json.loads(line[len('SUMMARY '):])
    return None


def has_instrumentation(label):
    return (ROOT / (label + '.assessments.json')).exists()


def main():
    combos = json.loads((ROOT / 'combos.json').read_text())['combinations']
    rows = []
    for combo in combos:
        label = combo['run_labels'][0]
        summary = load_summary(label)
        row = {
            'combination_index': combo['index'],
            'label': label,
            'parameters': combo['parameters'],
        }
        if summary is None:
            row['status'] = 'NOT_RUN'
            rows.append(row)
            continue
        chain = summary.get('chain') or {}
        allocation = summary.get('allocation') or []
        cuts = [a[1] for a in allocation if a[2] == 0 and a[1] < 1.0]
        first_cut = max(cuts) if cuts else None
        row.update({
            'grid_sha256': summary.get('grid_sha256'),
            'request_sha256': summary.get('request_sha256'),
            'context_sha256': summary.get('context_sha256'),
            'chain_regions': chain.get('regions'),
            'chain_digest': chain.get('digest'),
            'allocation_region0_last_cut': first_cut,
            'allocation': allocation,
            'guide_points': summary.get('guide_points'),
            'status': summary.get('status'),
            'reason': summary.get('reason'),
            'executable': summary.get('executable'),
            'audit_status': summary.get('audit_status'),
            'audit_detail': summary.get('audit_detail'),
            'template_attempts': summary.get('template_attempts'),
            'collision_station_count': (
                len(summary.get('collision_stations') or [])
                if has_instrumentation(label) else 'not_instrumented'),
            'captures': len(summary.get('captures') or []),
            'certificate_status': summary.get('certificate_status'),
            'f_only_gate_migrated': summary.get('f_only_gate_migrated'),
        })
        first = (summary.get('collision_stations') or [None])[0]
        if first:
            row['first_collision_station'] = {
                'sigma': first.get('sigma'),
                's_m': first.get('s_m'),
                'guide_pose': first.get('guide_pose'),
                'vehicles': first.get('vehicles'),
            }
        rows.append(row)

    repeats = []
    for label, of in (('s02b', 's02'), ('s03c', 's03')):
        summary = load_summary(label)
        repeats.append({
            'label': label,
            'repeat_of': of,
            'status': None if summary is None else summary.get('status'),
            'audit_status': None if summary is None else summary.get('audit_status'),
            'audit_detail': None if summary is None else summary.get('audit_detail'),
            'collision_station_count': (
                len(summary.get('collision_stations') or [])
                if summary is not None and has_instrumentation(label)
                else 'not_instrumented'),
        })

    for row in rows:
        evidence = [item for item in repeats
                    if item['repeat_of'] == row['label']]
        if evidence:
            row['instrumented_repeats'] = evidence

    payload = {
        'schema': 'task03_rework02.e09_scan_matrix.v1',
        'maximum_combinations': 30,
        'unique_combination_count': len(rows),
        'repeat_diagnostics': repeats,
        'rows': rows,
    }
    (ROOT / 'scan_matrix.json').write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    lines = [
        '# E09 frozen combination matrix (30/30)',
        '',
        '| # | label | deg | wall x | wall y0..y1 | half | pad | regions | alloc cut (R0) | assessment | collision stations | captures | final |',
        '|---|---|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for row in rows:
        p = row['parameters']
        wall = 'n/a' if 'wall_x' not in p else '{}..{}'.format(
            p.get('wall_y0'), p.get('wall_y1'))
        collisions = row.get('collision_station_count', 'n/a')
        if collisions == 'not_instrumented':
            repeats_for_row = row.get('instrumented_repeats') or []
            if repeats_for_row:
                collisions = 'repeat:{}'.format(
                    repeats_for_row[0]['collision_station_count'])
            else:
                collisions = 'not_instrumented'
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            row['combination_index'], row['label'], p.get('deg', 'n/a'),
            p.get('wall_x', 'n/a'), wall, p.get('wall_half', 'n/a'),
            p.get('pad', 'n/a'), row.get('chain_regions', 'n/a'),
            row.get('allocation_region0_last_cut', 'n/a'),
            row.get('audit_status', 'n/a'), collisions,
            row.get('captures', 'n/a'), row.get('status', 'n/a')))
    (ROOT / 'scan_matrix.md').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8')
    print('rows', len(rows), 'written scan_matrix.json/md')


if __name__ == '__main__':
    raise SystemExit(main())