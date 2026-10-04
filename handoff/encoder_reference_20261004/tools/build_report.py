#!/usr/bin/env python3
"""Regenerate handoff reports from archived evidence, entirely offline.

No SDK import, subprocess, network or hardware access. Writes only reports and
manifest in the explicit bundle directory; raw evidence is never rewritten.
"""
import argparse
import csv
import hashlib
import json
import statistics
from datetime import datetime
from pathlib import Path


TARGETS = {i: 1946 if i == 8 else 2081 if i == 12 else 2048 for i in range(1, 21)}
CAPTURES = (
    'encoder_capture_20261004_180130_557636',
    'encoder_capture_20261004_180349_734853',
    'encoder_capture_20261004_180445_688090',
)
GOALS = (
    'centerized_reference_20261004T110043_458436Z',
    'centerized_reference_20261004T110103_691892Z',
    'centerized_reference_20261004T110409_969079Z',
)
JOINT_NAMES = (
    'Lengan Kiri', 'Lengan Kanan', 'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
    'Tangan Kiri', 'Tangan Kanan', 'Paha Kanan Putar', 'Paha Kiri Putar',
    'Paha Atas Kanan', 'Paha Atas Kiri', 'Paha Bawah Kanan', 'Paha Bawah Kiri',
    'Lutut Kanan', 'Lutut Kiri', 'Kaki Kanan Atas', 'Kaki Kiri Atas',
    'Kaki Kanan Bawah', 'Kaki Kiri Bawah', 'Leher Putar', 'Kepala Putar',
)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_capture(folder):
    meta = read_json(folder / 'capture_metadata.json')
    with (folder / 'encoder_reference_centerized.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    good = [r for r in rows if not r['error']]
    require(len(rows) == len(good) == 100, f'{folder.name}: incomplete/invalid rows')
    require(meta['read_failures'] == 0, f'{folder.name}: declared read failures')
    require(not meta['read_retries'], f'{folder.name}: recorded read retries')
    require(not meta['configuration_errors'], f'{folder.name}: snapshot errors')
    require({int(r['id']) for r in good} == set(TARGETS), 'Unexpected ID set')
    result = []
    for i, target in TARGETS.items():
        rr = [r for r in good if int(r['id']) == i]
        require(len(rr) == 5 and {int(r['sample']) for r in rr} == set(range(1, 6)),
                f'{folder.name}: wrong sample count for ID{i}')
        require(all(int(r['goal_ticks']) == target for r in rr), f'ID{i}: goal mismatch')
        expected_model = 29 if i <= 6 or i >= 19 else 310
        require(all(int(r['model_number']) == expected_model for r in rr), f'ID{i}: model mismatch')
        require(all(r['torque_enable'] == '1' and r['moving'] == '0'
                    and r['present_speed_raw'] == '0' for r in rr), f'ID{i}: not stationary/ON')
        values = [int(r['present_ticks']) for r in rr]
        require(max(values) - min(values) <= 1, f'ID{i}: unexpected within-capture variation')
        mean = statistics.mean(values)
        result.append(dict(capture=folder.name, id=i, joint=JOINT_NAMES[i-1],
                           samples=5, goal_ticks=target, present_mean_ticks=round(mean, 3),
                           present_min_ticks=min(values), present_max_ticks=max(values),
                           range_ticks=max(values)-min(values),
                           mean_error_ticks=round(mean-target, 3),
                           all_samples_within_8_ticks=all(abs(v-target) <= 8 for v in values)))
    return meta, result


def read_goal(folder):
    meta = read_json(folder / 'reference_metadata.json')
    events = [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines()]
    settle = [e for e in events if e['event'] == 'settling_feedback']
    positions = settle[-1]['positions'] if settle else meta.get('positions', {})
    summary = {k: meta.get(k) for k in ('started_utc', 'finished_utc', 'completed',
                                        'status', 'error', 'usb_latency_ms')}
    summary.update(folder=folder.name, final_settling_samples=len(settle),
                   outside_8_ticks_in_last_feedback=[int(i) for i, r in positions.items()
                                                     if abs(r['present_ticks']-TARGETS[int(i)]) > 8],
                   final_feedback=positions)
    return meta, summary, positions


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def build(bundle, source_repo=None):
    evidence = bundle / 'evidence'
    if source_repo:
        for name in CAPTURES + GOALS:
            original = source_repo / name
            copied = evidence / name
            require({p.relative_to(original) for p in original.rglob('*') if p.is_file()}
                    == {p.relative_to(copied) for p in copied.rglob('*') if p.is_file()},
                    f'{name}: original/archive file lists differ')
            for path in copied.rglob('*'):
                if path.is_file():
                    require(path.read_bytes() == (original / path.relative_to(copied)).read_bytes(),
                            f'Archived evidence changed: {path}')
    goal_data = [read_goal(evidence / name) for name in GOALS]
    capture_data = [read_capture(evidence / name) for name in CAPTURES]
    latest_meta, latest_goal, latest_positions = goal_data[-1]
    primary_meta, primary_rows = capture_data[-1]
    require(latest_goal['status'] == 'ALREADY_AT_REFERENCE_GOALS_NO_WRITES'
            and latest_goal['final_settling_samples'] == 0, 'Unexpected latest goal result')
    require(set(latest_positions) == {str(i) for i in TARGETS}, 'Missing goal positions')
    require(all(latest_positions[str(i)]['goal_ticks'] == target
                for i, target in TARGETS.items()), 'Latest goal registers do not match targets')
    for meta, _ in capture_data:
        require(meta['source_sha256'] == latest_meta['source_sha256'],
                'Shared snapshot hash mismatch')
    for row in primary_rows:
        row['delta_from_latest_goal_feedback_ticks'] = round(
            row['present_mean_ticks']-latest_positions[str(row['id'])]['present_ticks'], 3)
    outside = [r['id'] for r in primary_rows if not r['all_samples_within_8_ticks']]
    require(outside == [14, 15, 16, 19], f'Unexpected primary offsets: {outside}')
    comparison_rows = [row for _, rr in capture_data for row in rr]
    # Ensure identical CSV columns without implying older captures follow latest goal.
    for row in comparison_rows:
        row.setdefault('delta_from_latest_goal_feedback_ticks', '')
    captures = []
    for name, (meta, rr) in zip(CAPTURES, capture_data):
        captures.append(dict(folder=name, started_utc=meta['started_utc'],
                             finished_utc=meta['finished_utc'], valid_rows=100,
                             read_failures=meta['read_failures'], read_retries=meta['read_retries'],
                             pose_confirmed_by_operator=meta['pose_confirmed_by_operator'],
                             max_within_capture_range_ticks=max(r['range_ticks'] for r in rr),
                             max_absolute_sample_error_ticks=max(max(abs(r['present_min_ticks']-r['goal_ticks']),
                                                                     abs(r['present_max_ticks']-r['goal_ticks']))
                                                                 for r in rr),
                             outside_8_ticks=[r['id'] for r in rr if not r['all_samples_within_8_ticks']]))
    time_gap = (datetime.fromisoformat(primary_meta['started_utc'])
                - datetime.fromisoformat(latest_meta['finished_utc'])).total_seconds()
    prior_positions = goal_data[1][2]
    well_paired_delta = max(abs(r['present_mean_ticks']-prior_positions[str(r['id'])]['present_ticks'])
                           for r in capture_data[0][1])
    require(well_paired_delta <= 1, 'Earlier paired capture no longer matches feedback')
    summary = dict(schema_version=1, package_status='finalized_handoff_candidate_not_final_calibration',
                   geometric_zero_certified=False, communication_reliability_certified=False,
                   dynamics_or_rl_readiness_certified=False, assistant_operated_hardware=False,
                   parent_git_commit='def2aebdd1175422397a749621a7a4a6ae90f622',
                   targets_raw_ticks=TARGETS, units='raw legacy-MX encoder ticks',
                   sampling='sequential register/joint reads, not simultaneous samples',
                   primary_capture=CAPTURES[-1], latest_goal_run=GOALS[-1],
                   captures=captures, goals=[item[1] for item in goal_data],
                   shared_snapshot_hashes_match=True, primary_start_after_latest_goal_exit_s=round(time_gap, 6),
                   earlier_well_paired_max_encoder_delta_ticks=round(well_paired_delta, 3),
                   limitations=['Already-target shortcut bypasses final settling checks',
                                'Operator pose confirmation is not independent geometric verification',
                                'Unknown handling/load/support changes between runs',
                                'Capture completed flag alone does not prove all reads succeeded',
                                'No cause of intermittent communication failures established'],
                   evidence_originals_byte_verified=bool(source_repo))
    reports = bundle / 'reports'
    reports.mkdir(exist_ok=True)
    write_csv(reports / 'primary_encoder_summary.csv', primary_rows)
    write_csv(reports / 'capture_comparison.csv', comparison_rows)
    (reports / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    entries = {str(p.relative_to(bundle)): dict(bytes=p.stat().st_size,
               sha256=hashlib.sha256(p.read_bytes()).hexdigest())
               for p in sorted(bundle.rglob('*')) if p.is_file() and p != bundle / 'manifest.json'}
    (bundle / 'manifest.json').write_text(json.dumps(dict(algorithm='SHA-256', files=entries), indent=2)+'\n')
    print(f'PASS: 3 captures, 300 valid rows; primary offset IDs {outside}; {len(entries)} files hashed.')
    print('Handoff complete; geometric reference and RL readiness NOT certified.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--source-repo', type=Path, help='Optional byte-for-byte comparison with original run folders')
    args = parser.parse_args()
    build(args.bundle.resolve(), args.source_repo.resolve() if args.source_repo else None)
