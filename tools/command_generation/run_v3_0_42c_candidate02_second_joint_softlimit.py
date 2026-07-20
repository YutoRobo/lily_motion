#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import copy
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, JOINT_THIGH, LEG_NAMES_BY_ID
from lily_motion_v3.command_resampler import write_command_records

TARGET_PHASES_DEFAULT = 'RF-4_Goal4_LandMiddlePair,RF-5_Goal5_MainBodyRoll'


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _load_jsonl(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _to_float_list(values):
    return [float(v) for v in values]


def _normalize_commands(records):
    notes = []
    deg_created = 0
    rad_created = 0
    for i, rec in enumerate(records):
        has_deg = 'joint_command_deg' in rec and rec.get('joint_command_deg') is not None
        has_rad = 'joint_command_rad' in rec and rec.get('joint_command_rad') is not None
        if has_deg:
            rec['joint_command_deg'] = _to_float_list(rec['joint_command_deg'])
        if has_rad:
            rec['joint_command_rad'] = _to_float_list(rec['joint_command_rad'])
        if not has_deg and not has_rad:
            raise ValueError('record %d has neither joint_command_deg nor joint_command_rad' % i)
        if not has_deg:
            rec['joint_command_deg'] = [math.degrees(v) for v in rec['joint_command_rad']]
            deg_created += 1
        if not has_rad:
            rec['joint_command_rad'] = [math.radians(v) for v in rec['joint_command_deg']]
            rad_created += 1
        if len(rec['joint_command_deg']) != len(rec['joint_command_rad']):
            raise ValueError('record %d command length mismatch: deg=%d rad=%d' % (
                i, len(rec['joint_command_deg']), len(rec['joint_command_rad'])))
    if deg_created:
        notes.append('joint_command_deg was created from joint_command_rad for %d records' % deg_created)
    if rad_created:
        notes.append('joint_command_rad was created from joint_command_deg for %d records' % rad_created)
    if not notes:
        notes.append('joint_command_deg and joint_command_rad were both present; modified samples were synchronized')
    return notes


def _thigh_entries():
    entries = []
    for joint_state_index, pair in enumerate(JOINT_STATE_ORDER):
        legacy_leg_id, joint_index = pair
        if joint_index == JOINT_THIGH:
            entries.append({
                'joint_state_index': joint_state_index,
                'legacy_leg_id': legacy_leg_id,
                'leg_name': LEG_NAMES_BY_ID[legacy_leg_id],
                'joint_index': joint_index,
                'joint_name': 'thigh',
            })
    return entries


def _max_adjacent_delta_deg(records):
    worst = {'value': 0.0, 'record_index': None, 'joint_state_index': None}
    for i in range(1, len(records)):
        q0 = records[i - 1]['joint_command_deg']
        q1 = records[i]['joint_command_deg']
        for j, (a, b) in enumerate(zip(q0, q1)):
            d = abs(float(b) - float(a))
            if d > worst['value']:
                worst = {'value': d, 'record_index': i, 'joint_state_index': j}
    return worst


def _max_second_diff_deg(records):
    worst = {'value': 0.0, 'record_index': None, 'joint_state_index': None}
    for i in range(1, len(records) - 1):
        qm = records[i - 1]['joint_command_deg']
        q0 = records[i]['joint_command_deg']
        qp = records[i + 1]['joint_command_deg']
        for j, (a, b, c) in enumerate(zip(qm, q0, qp)):
            d = abs(float(c) - 2.0 * float(b) + float(a))
            if d > worst['value']:
                worst = {'value': d, 'record_index': i, 'joint_state_index': j}
    return worst


def _second_joint_stats(records, thigh_entries, violation_limit_deg):
    max_abs = 0.0
    violation_count = 0
    worst = None
    for i, rec in enumerate(records):
        q = rec['joint_command_deg']
        for entry in thigh_entries:
            j = entry['joint_state_index']
            if j >= len(q):
                continue
            abs_angle = abs(float(q[j]))
            if abs_angle > violation_limit_deg:
                violation_count += 1
            if abs_angle > max_abs:
                max_abs = abs_angle
                worst = dict(entry)
                worst.update({
                    'record_index': i,
                    'frame_index': rec.get('frame_index', i),
                    'roll_index': rec.get('roll_index'),
                    'phase_name': rec.get('phase_name'),
                    'angle_deg': q[j],
                    'abs_angle_deg': abs_angle,
                })
    return {'max_abs_angle_deg': max_abs, 'violation_count': violation_count, 'worst': worst}


def _find_violation_intervals(records, thigh_entries, target_phases, soft_limit_deg):
    intervals = []
    for entry in thigh_entries:
        j = entry['joint_state_index']
        for phase in target_phases:
            start = None
            end = None
            max_abs = 0.0
            for i, rec in enumerate(records):
                is_violation = (
                    rec.get('phase_name') == phase and
                    j < len(rec['joint_command_deg']) and
                    abs(float(rec['joint_command_deg'][j])) > soft_limit_deg
                )
                if is_violation:
                    if start is None:
                        start = i
                    end = i
                    max_abs = max(max_abs, abs(float(rec['joint_command_deg'][j])))
                elif start is not None:
                    intervals.append(dict(entry, phase_name=phase, start_record_index=start,
                                          end_record_index=end, max_abs_angle_deg=max_abs))
                    start = None
                    end = None
                    max_abs = 0.0
            if start is not None:
                intervals.append(dict(entry, phase_name=phase, start_record_index=start,
                                      end_record_index=end, max_abs_angle_deg=max_abs))
    return intervals


def _boundary_correction(records, record_index, joint_state_index, soft_limit_deg):
    angle = float(records[record_index]['joint_command_deg'][joint_state_index])
    if angle == 0.0:
        return 0.0, 0
    sign = 1 if angle > 0.0 else -1
    return angle - sign * soft_limit_deg, sign


def _build_correction_envelope(records, intervals, target_phases, soft_limit_deg, taper_radius):
    corrections = {}
    envelope_samples = 0
    for interval in intervals:
        j = interval['joint_state_index']
        phase = interval['phase_name']
        start = interval['start_record_index']
        end = interval['end_record_index']
        start_corr, start_sign = _boundary_correction(records, start, j, soft_limit_deg)
        end_corr, end_sign = _boundary_correction(records, end, j, soft_limit_deg)
        lo = max(0, start - taper_radius)
        hi = min(len(records) - 1, end + taper_radius)
        for i in range(lo, hi + 1):
            if records[i].get('phase_name') not in target_phases:
                continue
            if records[i].get('phase_name') != phase:
                continue
            if j >= len(records[i]['joint_command_deg']):
                continue
            angle = float(records[i]['joint_command_deg'][j])
            if angle == 0.0:
                continue
            angle_sign = 1 if angle > 0.0 else -1
            if start <= i <= end:
                corr = angle - angle_sign * soft_limit_deg
            elif i < start:
                if angle_sign != start_sign:
                    continue
                distance = start - i
                weight = float(taper_radius + 1 - distance) / float(taper_radius + 1)
                corr = start_corr * max(0.0, weight)
            else:
                if angle_sign != end_sign:
                    continue
                distance = i - end
                weight = float(taper_radius + 1 - distance) / float(taper_radius + 1)
                corr = end_corr * max(0.0, weight)
            if abs(corr) <= 1e-12:
                continue
            key = (i, j)
            envelope_samples += 1
            if key not in corrections or abs(corr) > abs(corrections[key]['correction_deg']):
                corrections[key] = {
                    'correction_deg': corr,
                    'phase_name': records[i].get('phase_name'),
                    'leg_name': interval['leg_name'],
                    'joint_state_index': j,
                    'source_interval': {
                        'phase_name': phase,
                        'leg_name': interval['leg_name'],
                        'start_record_index': start,
                        'end_record_index': end,
                    },
                }
    return corrections, envelope_samples


def _apply_corrections(records, corrections):
    modified_frames = set()
    modified_leg_names = set()
    modified_phase_names = set()
    max_abs_correction = 0.0
    modified_samples = []
    for key, info in sorted(corrections.items()):
        i, j = key
        corr = float(info['correction_deg'])
        old_deg = float(records[i]['joint_command_deg'][j])
        new_deg = old_deg - corr
        records[i]['joint_command_deg'][j] = new_deg
        records[i]['joint_command_rad'][j] = math.radians(new_deg)
        modified_frames.add(i)
        modified_leg_names.add(info['leg_name'])
        modified_phase_names.add(info['phase_name'])
        max_abs_correction = max(max_abs_correction, abs(corr))
        modified_samples.append({
            'record_index': i,
            'frame_index': records[i].get('frame_index', i),
            'roll_index': records[i].get('roll_index'),
            'phase_name': info['phase_name'],
            'leg_name': info['leg_name'],
            'joint_state_index': j,
            'old_angle_deg': old_deg,
            'new_angle_deg': new_deg,
            'correction_deg': corr,
        })
    return {
        'modified_sample_count': len(modified_samples),
        'modified_frame_count': len(modified_frames),
        'modified_leg_names': sorted(modified_leg_names),
        'modified_phase_names': sorted(modified_phase_names),
        'max_abs_correction_deg': max_abs_correction,
        'modified_samples_preview': modified_samples[:50],
    }


def main():
    ap = argparse.ArgumentParser(description='Apply local soft-limit postprocess to v3.0.42c candidate_02 thigh joints.')
    ap.add_argument('--input', default='data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl')
    ap.add_argument('--output-dir', default='testdata/candidate02_softlimit')
    ap.add_argument('--output-command-log', default=None)
    ap.add_argument('--report-output', default=None)
    ap.add_argument('--soft-limit-deg', type=float, default=94.8)
    ap.add_argument('--violation-limit-deg', type=float, default=95.0)
    ap.add_argument('--taper-radius', type=int, default=5)
    ap.add_argument('--target-phases', default=TARGET_PHASES_DEFAULT)
    ap.add_argument('--warning-threshold-deg', type=float, default=0.05)
    args = ap.parse_args()

    output_dir = args.output_dir
    _ensure_dir(output_dir)
    output_command_log = args.output_command_log or os.path.join(output_dir, 'commands.jsonl')
    report_output = args.report_output or os.path.join(output_dir, 'softlimit_report.json')
    target_phases = [x.strip() for x in args.target_phases.split(',') if x.strip()]

    original_records = _load_jsonl(args.input)
    before_records = copy.deepcopy(original_records)
    after_records = copy.deepcopy(original_records)
    before_notes = _normalize_commands(before_records)
    after_notes = _normalize_commands(after_records)
    thigh_entries = _thigh_entries()

    before_second = _second_joint_stats(before_records, thigh_entries, args.violation_limit_deg)
    before_adj = _max_adjacent_delta_deg(before_records)
    before_second_diff = _max_second_diff_deg(before_records)

    intervals = _find_violation_intervals(after_records, thigh_entries, target_phases, args.soft_limit_deg)
    corrections, envelope_samples = _build_correction_envelope(
        after_records, intervals, set(target_phases), args.soft_limit_deg, max(0, args.taper_radius))
    modification_summary = _apply_corrections(after_records, corrections)

    after_second = _second_joint_stats(after_records, thigh_entries, args.violation_limit_deg)
    after_adj = _max_adjacent_delta_deg(after_records)
    after_second_diff = _max_second_diff_deg(after_records)

    warnings = []
    if after_adj['value'] > before_adj['value'] + args.warning_threshold_deg:
        warnings.append('max adjacent delta worsened before Gazebo replay: before=%.9f after=%.9f threshold=%.9f' % (
            before_adj['value'], after_adj['value'], args.warning_threshold_deg))
    if after_second_diff['value'] > before_second_diff['value'] + args.warning_threshold_deg:
        warnings.append('max second difference worsened before Gazebo replay: before=%.9f after=%.9f threshold=%.9f' % (
            before_second_diff['value'], after_second_diff['value'], args.warning_threshold_deg))

    write_command_records(after_records, output_command_log)

    report = {
        'input': args.input,
        'output_command_log': output_command_log,
        'soft_limit_deg': args.soft_limit_deg,
        'violation_limit_deg': args.violation_limit_deg,
        'taper_radius': args.taper_radius,
        'target_phases': target_phases,
        'joint_selection': {
            'source': 'lily_motion_v3.interface_config.JOINT_STATE_ORDER',
            'joint_index': JOINT_THIGH,
            'joint_name': 'thigh',
            'joint_state_indices': thigh_entries,
        },
        'command_sync_notes_before': before_notes,
        'command_sync_notes_after': after_notes,
        'interval_count': len(intervals),
        'intervals': intervals,
        'envelope_candidate_sample_count': envelope_samples,
        'modified_sample_count': modification_summary['modified_sample_count'],
        'modified_frame_count': modification_summary['modified_frame_count'],
        'modified_leg_names': modification_summary['modified_leg_names'],
        'modified_phase_names': modification_summary['modified_phase_names'],
        'max_abs_correction_deg': modification_summary['max_abs_correction_deg'],
        'max_adjacent_delta_before_deg': before_adj['value'],
        'max_adjacent_delta_after_deg': after_adj['value'],
        'max_adjacent_delta_before_worst': before_adj,
        'max_adjacent_delta_after_worst': after_adj,
        'max_second_diff_before_deg': before_second_diff['value'],
        'max_second_diff_after_deg': after_second_diff['value'],
        'max_second_diff_before_worst': before_second_diff,
        'max_second_diff_after_worst': after_second_diff,
        'second_joint_max_before_deg': before_second['max_abs_angle_deg'],
        'second_joint_max_after_deg': after_second['max_abs_angle_deg'],
        'second_joint_worst_before': before_second['worst'],
        'second_joint_worst_after': after_second['worst'],
        'violation_count_before': before_second['violation_count'],
        'violation_count_after': after_second['violation_count'],
        'warnings': warnings,
        'modified_samples_preview': modification_summary['modified_samples_preview'],
        'gazebo_replay_command': 'python tools/gazebo/run_v3_0_gazebo_replay.py --command-log %s --strict-command-log-input --rate 15 --hold-start-sec 2.0 --hold-end-sec 2.0 --diagnose-command-log' % output_command_log,
    }
    with open(report_output, 'w') as f:
        json.dump(report, f, indent=2, sort_keys=True)

    summary = {
        'output_command_log': output_command_log,
        'report_output': report_output,
        'modified_sample_count': report['modified_sample_count'],
        'modified_frame_count': report['modified_frame_count'],
        'max_abs_correction_deg': report['max_abs_correction_deg'],
        'second_joint_max_before_deg': report['second_joint_max_before_deg'],
        'second_joint_max_after_deg': report['second_joint_max_after_deg'],
        'violation_count_before': report['violation_count_before'],
        'violation_count_after': report['violation_count_after'],
        'max_adjacent_delta_before_deg': report['max_adjacent_delta_before_deg'],
        'max_adjacent_delta_after_deg': report['max_adjacent_delta_after_deg'],
        'max_second_diff_before_deg': report['max_second_diff_before_deg'],
        'max_second_diff_after_deg': report['max_second_diff_after_deg'],
        'warnings': warnings,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    if warnings:
        for warning in warnings:
            print('WARNING: %s' % warning, file=sys.stderr)


if __name__ == '__main__':
    main()
