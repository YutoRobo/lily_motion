#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import argparse
import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_constraint_evaluator import (  # noqa: E402
    LegacyConstraintEvaluator,
    _point_segment_distance,
    _segment_distance,
)

DEFAULT_COMMAND_LOG = 'data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl'
DEFAULT_OUTPUT_DIR = 'testdata/near_contact_phase_review'
RF2_PREFIX = 'RF-2'
RF3_PREFIX = 'RF-3'
LINK_SPECS = (('upper', 'upper_segment'), ('lower', 'lower_segment'))


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def load_records(path):
    out = []
    with open(path) as f:
        for line_index, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            rec['_line_index'] = line_index
            out.append(rec)
    return out


def phase_tag(phase_name):
    phase_name = str(phase_name or '')
    if phase_name.startswith(RF2_PREFIX):
        return 'RF-2'
    if phase_name.startswith(RF3_PREFIX):
        return 'RF-3'
    return 'other'


def severity(distance_m, warning_m, danger_m, reject_m):
    if distance_m is None:
        return 'none'
    if distance_m < reject_m:
        return 'reject'
    if distance_m < danger_m:
        return 'danger'
    if distance_m < warning_m:
        return 'warning'
    return 'ok'


def frame_id(rec, fallback):
    return int(rec.get('frame_index', rec.get('command_index', rec.get('_line_index', fallback))))


def make_base_record(rec, fallback_index):
    return {
        'line_index': int(rec.get('_line_index', fallback_index)),
        'frame_index': frame_id(rec, fallback_index),
        'command_index': rec.get('command_index'),
        'roll_index': rec.get('roll_index'),
        'phase_name': rec.get('phase_name'),
        'phase_step_index': rec.get('phase_step_index'),
        'phase_step_count': rec.get('phase_step_count'),
        'source_frame_index': rec.get('source_frame_index'),
    }


def pair_record(base, kind, distance_m, a, b, extra):
    row = dict(base)
    row.update({
        'kind': kind,
        'distance_m': distance_m,
    })
    row.update(extra)
    return row


def evaluate_frame(evaluator, rec, fallback_index, warning_m, danger_m, reject_m):
    geom = evaluator._frame_geometry(rec)
    ids = sorted(geom.keys())
    base = make_base_record(rec, fallback_index)
    base['phase_tag'] = phase_tag(base.get('phase_name'))

    all_pairs = []
    min_inter_leg = None
    min_joint_housing = None
    worst_inter_leg = None
    worst_joint_housing = None

    for ai in range(len(ids)):
        for bi in range(ai + 1, len(ids)):
            a = geom[ids[ai]]
            b = geom[ids[bi]]
            for link_a_name, link_a_key in LINK_SPECS:
                for link_b_name, link_b_key in LINK_SPECS:
                    seg_a = a[link_a_key]
                    seg_b = b[link_b_key]
                    d = _segment_distance(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
                    row = pair_record(base, 'inter_leg_link_segment', d, a, b, {
                        'leg_a_id': ids[ai],
                        'leg_a_name': a['leg_name'],
                        'part_a': link_a_name,
                        'leg_b_id': ids[bi],
                        'leg_b_name': b['leg_name'],
                        'part_b': link_b_name,
                        'threshold_warning_m': warning_m,
                        'threshold_danger_m': danger_m,
                        'threshold_reject_m': reject_m,
                        'severity': severity(d, warning_m, danger_m, reject_m),
                    })
                    all_pairs.append(row)
                    if min_inter_leg is None or d < min_inter_leg:
                        min_inter_leg = d
                        worst_inter_leg = row

    for jid in ids:
        joint_leg = geom[jid]
        joint_center = joint_leg['knee_abs']
        for lid in ids:
            if lid == jid:
                continue
            link_leg = geom[lid]
            for link_name, link_key in LINK_SPECS:
                seg = link_leg[link_key]
                d = _point_segment_distance(joint_center, seg[0], seg[1])
                row = pair_record(base, 'second_joint_housing_to_other_leg_link', d, joint_leg, link_leg, {
                    'leg_a_id': jid,
                    'leg_a_name': joint_leg['leg_name'],
                    'part_a': 'second_joint_housing',
                    'leg_b_id': lid,
                    'leg_b_name': link_leg['leg_name'],
                    'part_b': link_name,
                    'threshold_warning_m': warning_m,
                    'threshold_danger_m': danger_m,
                    'threshold_reject_m': reject_m,
                    'severity': severity(d, warning_m, danger_m, reject_m),
                })
                all_pairs.append(row)
                if min_joint_housing is None or d < min_joint_housing:
                    min_joint_housing = d
                    worst_joint_housing = row

    candidates = [x for x in (worst_inter_leg, worst_joint_housing) if x is not None]
    worst = None
    if candidates:
        worst = sorted(candidates, key=lambda x: x['distance_m'])[0]
    frame_row = dict(base)
    frame_row.update({
        'min_distance_m': None if worst is None else worst['distance_m'],
        'min_kind': None if worst is None else worst['kind'],
        'min_leg_a_id': None if worst is None else worst['leg_a_id'],
        'min_leg_a_name': None if worst is None else worst['leg_a_name'],
        'min_part_a': None if worst is None else worst['part_a'],
        'min_leg_b_id': None if worst is None else worst['leg_b_id'],
        'min_leg_b_name': None if worst is None else worst['leg_b_name'],
        'min_part_b': None if worst is None else worst['part_b'],
        'min_inter_leg_link_segment_m': min_inter_leg,
        'min_second_joint_housing_to_link_m': min_joint_housing,
        'severity': severity(None if worst is None else worst['distance_m'], warning_m, danger_m, reject_m),
    })
    return frame_row, all_pairs, worst


def first_rf2_rf3_boundary(first_roll_records):
    prev = None
    for idx, rec in enumerate(first_roll_records):
        tag = phase_tag(rec.get('phase_name'))
        if prev == 'RF-2' and tag == 'RF-3':
            return idx
        if tag != 'other':
            prev = tag
    return None


def in_focus_window(idx, rec, boundary_idx, transition_window):
    tag = phase_tag(rec.get('phase_name'))
    if tag in ('RF-2', 'RF-3'):
        return True
    if boundary_idx is not None and abs(idx - boundary_idx) <= transition_window:
        return True
    return False


def write_csv(path, rows, fieldnames):
    with open(path, 'w') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        for row in rows:
            w.writerow(row)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Scan first-quarter RF-2/RF-3 near-contact distances for candidate02 before hardware use.')
    ap.add_argument('--command-log', default=DEFAULT_COMMAND_LOG)
    ap.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR)
    ap.add_argument('--roll-index', type=int, default=0)
    ap.add_argument('--transition-window', type=int, default=10)
    ap.add_argument('--context-window', type=int, default=10)
    ap.add_argument('--warning-threshold-m', type=float, default=0.020)
    ap.add_argument('--danger-threshold-m', type=float, default=0.010)
    ap.add_argument('--reject-threshold-m', type=float, default=0.005)
    ap.add_argument('--top-n', type=int, default=80)
    args = ap.parse_args(argv)

    ensure_dir(args.output_dir)
    records = load_records(args.command_log)
    first_roll = [r for r in records if int(r.get('roll_index', -999)) == int(args.roll_index)]
    if not first_roll:
        raise SystemExit('no records found for roll_index=%s in %s' % (args.roll_index, args.command_log))

    boundary_idx = first_rf2_rf3_boundary(first_roll)
    _json_stdout = sys.stdout
    _devnull = open(os.devnull, 'w')
    sys.stdout = _devnull
    try:
        evaluator = LegacyConstraintEvaluator()
    finally:
        sys.stdout = _json_stdout
        _devnull.close()

    frame_rows_all = []
    frame_rows_focus = []
    dangerous_pairs = []
    focus_pair_rows = []
    worst_rows = []
    min_by_phase = {}

    for idx, rec in enumerate(first_roll):
        frame_row, pair_rows, worst = evaluate_frame(
            evaluator, rec, idx,
            args.warning_threshold_m, args.danger_threshold_m, args.reject_threshold_m)
        frame_row['first_roll_local_index'] = idx
        frame_row['in_rf2_rf3_focus'] = in_focus_window(idx, rec, boundary_idx, args.transition_window)
        frame_rows_all.append(frame_row)
        if frame_row['in_rf2_rf3_focus']:
            frame_rows_focus.append(frame_row)
            if worst is not None:
                w = dict(worst)
                w['first_roll_local_index'] = idx
                worst_rows.append(w)
            for row in pair_rows:
                row = dict(row)
                row['first_roll_local_index'] = idx
                focus_pair_rows.append(row)
                if row['severity'] in ('warning', 'danger', 'reject'):
                    dangerous_pairs.append(row)
        tag = frame_row['phase_tag']
        if tag in ('RF-2', 'RF-3'):
            cur = min_by_phase.get(tag)
            if cur is None or frame_row['min_distance_m'] < cur['min_distance_m']:
                min_by_phase[tag] = dict(frame_row)

    if not frame_rows_focus:
        raise SystemExit('no RF-2/RF-3 focus frames found')
    focus_worst = sorted(frame_rows_focus, key=lambda x: x['min_distance_m'])[0]
    all_first_roll_worst = sorted(frame_rows_all, key=lambda x: x['min_distance_m'])[0]
    focus_pair_minimum = sorted(focus_pair_rows, key=lambda x: x['distance_m'])[0] if focus_pair_rows else None
    focus_inter_leg_link_minimum = sorted([r for r in focus_pair_rows if r['kind'] == 'inter_leg_link_segment'], key=lambda x: x['distance_m'])[0] if focus_pair_rows else None
    housing_rows = [r for r in focus_pair_rows if r['kind'] == 'second_joint_housing_to_other_leg_link']
    focus_second_joint_housing_minimum = sorted(housing_rows, key=lambda x: x['distance_m'])[0] if housing_rows else None

    focus_line = int(focus_worst['line_index'])
    lo = focus_line - int(args.context_window)
    hi = focus_line + int(args.context_window)
    near_contact_frames = [r for r in frame_rows_all if int(r['line_index']) >= lo and int(r['line_index']) <= hi]

    dangerous_pairs_sorted = sorted(dangerous_pairs, key=lambda x: x['distance_m'])[:args.top_n]
    worst_rows_sorted = sorted(worst_rows, key=lambda x: x['distance_m'])

    rf2_min = min_by_phase.get('RF-2')
    rf3_min = min_by_phase.get('RF-3')
    if rf2_min and rf3_min:
        closer_phase = 'RF-2' if rf2_min['min_distance_m'] <= rf3_min['min_distance_m'] else 'RF-3'
    elif rf2_min:
        closer_phase = 'RF-2'
    elif rf3_min:
        closer_phase = 'RF-3'
    else:
        closer_phase = None

    fields_frame = [
        'line_index', 'first_roll_local_index', 'frame_index', 'command_index', 'roll_index',
        'phase_name', 'phase_tag', 'phase_step_index', 'phase_step_count', 'source_frame_index',
        'in_rf2_rf3_focus', 'min_distance_m', 'min_kind',
        'min_leg_a_id', 'min_leg_a_name', 'min_part_a',
        'min_leg_b_id', 'min_leg_b_name', 'min_part_b',
        'min_inter_leg_link_segment_m', 'min_second_joint_housing_to_link_m', 'severity',
    ]
    write_csv(os.path.join(args.output_dir, 'min_distance_by_frame.csv'), frame_rows_all, fields_frame)

    fields_pair = [
        'line_index', 'first_roll_local_index', 'frame_index', 'command_index', 'roll_index',
        'phase_name', 'phase_tag', 'phase_step_index', 'kind', 'distance_m', 'severity',
        'leg_a_id', 'leg_a_name', 'part_a', 'leg_b_id', 'leg_b_name', 'part_b',
        'threshold_warning_m', 'threshold_danger_m', 'threshold_reject_m',
    ]
    write_csv(os.path.join(args.output_dir, 'dangerous_pairs.csv'), dangerous_pairs_sorted, fields_pair)
    write_csv(os.path.join(args.output_dir, 'near_contact_frames.csv'), near_contact_frames, fields_frame)

    review_frames = []
    for row in near_contact_frames:
        review_frames.append(int(row['line_index']))
    for row in worst_rows_sorted[:20]:
        review_frames.append(int(row['line_index']))
    review_frames = sorted(set(review_frames))
    with open(os.path.join(args.output_dir, 'gazebo_review_frames.txt'), 'w') as f:
        f.write('# JSONL line_index / command-log frame indices for Gazebo visual review\n')
        for v in review_frames:
            f.write('%d\n' % v)

    status = severity(focus_worst['min_distance_m'], args.warning_threshold_m, args.danger_threshold_m, args.reject_threshold_m)
    summary = {
        'command_log': args.command_log,
        'roll_index': args.roll_index,
        'frame_count_total': len(records),
        'first_roll_frame_count': len(first_roll),
        'focus_frame_count': len(frame_rows_focus),
        'focus_definition': 'roll_index=%s, RF-2/RF-3 phases plus +/- %d frames around RF-2->RF-3 transition' % (args.roll_index, args.transition_window),
        'thresholds_m': {
            'warning': args.warning_threshold_m,
            'danger': args.danger_threshold_m,
            'reject': args.reject_threshold_m,
        },
        'overall_status': status,
        'focus_minimum': focus_worst,
        'focus_pair_minimum': focus_pair_minimum,
        'focus_inter_leg_link_minimum': focus_inter_leg_link_minimum,
        'focus_second_joint_housing_minimum': focus_second_joint_housing_minimum,
        'first_quarter_overall_minimum': all_first_roll_worst,
        'phase_minimums': {
            'RF-2': rf2_min,
            'RF-3': rf3_min,
        },
        'rf2_or_rf3_closer_minimum': closer_phase,
        'dangerous_pair_count': len(dangerous_pairs),
        'dangerous_pair_count_top_written': len(dangerous_pairs_sorted),
        'gazebo_review_frame_count': len(review_frames),
        'outputs': {
            'summary': os.path.join(args.output_dir, 'summary.json'),
            'min_distance_by_frame': os.path.join(args.output_dir, 'min_distance_by_frame.csv'),
            'dangerous_pairs': os.path.join(args.output_dir, 'dangerous_pairs.csv'),
            'near_contact_frames': os.path.join(args.output_dir, 'near_contact_frames.csv'),
            'gazebo_review_frames': os.path.join(args.output_dir, 'gazebo_review_frames.txt'),
            'remaining_issues': os.path.join(args.output_dir, 'remaining_issues.md'),
        },
        'notes': [
            'Uses LegacyConstraintEvaluator FK and distance helpers; candidate command log is read-only.',
            'Minimum distance is centerline/geometric primitive distance, not mesh clearance.',
            'Joint housing check approximates the second/third joint housing as a sphere center against other-leg upper/lower link segments.',
        ],
        'can0_opened': False,
        'hardware_can_sent': False,
        'external_can_interface_executed': False,
    }
    with open(os.path.join(args.output_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write('\n')

    with open(os.path.join(args.output_dir, 'remaining_issues.md'), 'w') as f:
        f.write('# Near-Contact Phase Review Remaining Issues\n\n')
        f.write('- This is a geometric primitive scan, not a Gazebo mesh collision query.\n')
        f.write('- The suspected upper-contact-leg vs middle-leg wording is represented by all inter-leg link and second-joint-housing-to-link pairs; exact semantic upper/middle role labels are not encoded in the command log.\n')
        f.write('- Use `gazebo_review_frames.txt` for visual review around the minimum-distance frame.\n')
        if status in ('reject', 'danger'):
            f.write('- Status `%s`: review or correction is recommended before hardware motion.\n' % status)
        elif status == 'warning':
            f.write('- Status `warning`: no automatic rejection, but visual/hardware clearance review should remain explicit.\n')
        else:
            f.write('- Status `ok`: no frame crossed the provisional warning threshold in this scan.\n')

    print(json.dumps({
        'output_dir': args.output_dir,
        'overall_status': status,
        'min_distance_m': focus_worst['min_distance_m'],
        'line_index': focus_worst['line_index'],
        'frame_index': focus_worst['frame_index'],
        'roll_index': focus_worst['roll_index'],
        'phase_name': focus_worst['phase_name'],
        'pair': '%s:%s -> %s:%s' % (
            focus_worst['min_leg_a_name'], focus_worst['min_part_a'],
            focus_worst['min_leg_b_name'], focus_worst['min_part_b']),
        'rf2_or_rf3_closer_minimum': closer_phase,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
