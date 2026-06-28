#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID
from lily_motion_v3.legacy_state_machine_emulator import (
    LegacyStateMachineConfig, LegacyStateMachineEmulator, write_jsonl)
from lily_motion_v3.command_resampler import (
    load_command_records, write_command_records, resample_command_records,
    moving_average_command_records)


def parse_int_list(s):
    return [int(x) for x in str(s).split(',') if str(x).strip()]


def _ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


def _make_legacy_repeated_records(args):
    seq = parse_int_list(args.surface_sequence)
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=seq[0],
        z=args.legacy_body_z,
        initialize_step=args.initialize_step,
        include_initialize=False,
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=args.goal2_pitch_scale,
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
        middle_swing_y_escape=getattr(args, 'middle_swing_y_escape', 0.0),
        middle_swing_y_escape_mode=getattr(args, 'middle_swing_y_escape_mode', 'none'),
        middle_swing_y_escape_apply_rf3=getattr(args, 'middle_swing_y_escape_apply_rf3', False),
        middle_swing_y_escape_apply_rf4=getattr(args, 'middle_swing_y_escape_apply_rf4', False),
        goal5_x_scale=args.goal5_x_scale,
        goal5_pitch_scale=args.goal5_pitch_scale,
        rf1_current_angle_anchor=args.rf1_current_angle_anchor,
    )
    emu = LegacyStateMachineEmulator(cfg)
    try:
        records = emu.run_forward_repeated(surface_sequence=seq, include_initialize=args.include_initialize)
        completed = True
        error = None
    except Exception as e:
        records = list(getattr(emu, '_records', []))
        completed = False
        error = {'type': e.__class__.__name__, 'message': str(e)}
    return records, completed, error


def _source_frame_count(records):
    return len(records)


def _safe_key(value):
    if value is None:
        return 'None'
    return str(value)


def _new_group(group_type, group_key):
    return {
        'group_type': group_type,
        'group_key': group_key,
        'frame_count': 0,
        'sample_count': 0,
        'violation_count': 0,
        'violation_frame_count': 0,
        'max_abs_angle_deg': 0.0,
        'max_excess_deg': 0.0,
        'worst': None,
        '_violation_frames': set(),
    }


def _update_group(groups, group_type, group_key, item, frame_key):
    k = (group_type, _safe_key(group_key))
    if k not in groups:
        groups[k] = _new_group(group_type, _safe_key(group_key))
    g = groups[k]
    g['sample_count'] += 1
    if item.get('abs_angle_deg', 0.0) > g['max_abs_angle_deg']:
        g['max_abs_angle_deg'] = item['abs_angle_deg']
        g['worst'] = dict(item)
    if item.get('is_violation'):
        g['violation_count'] += 1
        g['_violation_frames'].add(frame_key)
        if item.get('excess_deg', 0.0) > g['max_excess_deg']:
            g['max_excess_deg'] = item['excess_deg']


def _finalize_group(g):
    out = dict(g)
    vf = out.pop('_violation_frames', set())
    out['violation_frame_count'] = len(vf)
    if out['sample_count']:
        out['violation_sample_rate'] = float(out['violation_count']) / float(out['sample_count'])
    else:
        out['violation_sample_rate'] = 0.0
    if out['frame_count']:
        out['violation_frame_rate'] = float(out['violation_frame_count']) / float(out['frame_count'])
    else:
        out['violation_frame_rate'] = 0.0
    return out


def _compute_boundary_indices(records, key_name):
    idxs = set()
    if not records:
        return idxs
    prev = records[0].get(key_name)
    for i in range(1, len(records)):
        cur = records[i].get(key_name)
        if cur != prev:
            idxs.add(i - 1)
            idxs.add(i)
        prev = cur
    return idxs


def _boundary_label(i, roll_boundary_indices, phase_boundary_indices, window):
    labels = []
    if window < 0:
        window = 0
    for b in roll_boundary_indices:
        if abs(i - b) <= window:
            labels.append('roll_boundary')
            break
    for b in phase_boundary_indices:
        if abs(i - b) <= window:
            labels.append('phase_boundary')
            break
    if not labels:
        return 'non_boundary'
    return '+'.join(labels)


def _record_frame_key(source_label, rec, record_index):
    return '%s:%s:%s' % (source_label, str(rec.get('frame_index', record_index)), str(record_index))


def _analyze_source(records, source_label, limit_deg, boundary_window, top_n):
    groups = {}
    violations = []
    samples = []
    worst = None
    violation_frame_keys = set()
    frame_keys = set()

    roll_boundary_indices = _compute_boundary_indices(records, 'roll_index')
    phase_boundary_indices = _compute_boundary_indices(records, 'phase_name')

    # Pre-count frame_count for groups.  This is intentionally based on records,
    # not joint samples, so per-group frame rates remain meaningful.
    frame_group_keys = {}
    for i, rec in enumerate(records):
        frame_key = _record_frame_key(source_label, rec, i)
        frame_keys.add(frame_key)
        phase = str(rec.get('phase_name', 'unknown'))
        roll = rec.get('roll_index')
        roll_key = _safe_key(roll)
        legless = [
            ('roll_index', roll_key),
            ('phase_name', phase),
            ('roll_phase', '%s|%s' % (roll_key, phase)),
            ('boundary_window', _boundary_label(i, roll_boundary_indices, phase_boundary_indices, boundary_window)),
        ]
        for gt, gk in legless:
            k = (gt, _safe_key(gk))
            if k not in groups:
                groups[k] = _new_group(gt, _safe_key(gk))
            groups[k]['frame_count'] += 1

    for i, rec in enumerate(records):
        q = rec.get('joint_command_rad') or []
        phase = str(rec.get('phase_name', 'unknown'))
        roll = rec.get('roll_index')
        roll_key = _safe_key(roll)
        boundary = _boundary_label(i, roll_boundary_indices, phase_boundary_indices, boundary_window)
        frame_key = _record_frame_key(source_label, rec, i)
        for idx, pair in enumerate(JOINT_STATE_ORDER):
            legacy_leg_id, joint_index = pair
            if joint_index != 1:
                continue
            if idx >= len(q):
                continue
            abs_deg = abs(math.degrees(float(q[idx])))
            excess = max(0.0, abs_deg - limit_deg)
            is_violation = abs_deg > limit_deg
            leg_name = LEG_NAMES_BY_ID[legacy_leg_id]
            item = {
                'source_label': source_label,
                'record_index': i,
                'frame_index': rec.get('frame_index', i),
                'source_frame_index': rec.get('source_frame_index'),
                'resampled_index': rec.get('resampled_index'),
                'roll_index': roll,
                'roll_surface_transition': rec.get('roll_surface_transition'),
                'phase_name': phase,
                'phase_step_index': rec.get('phase_step_index'),
                'phase_step_count': rec.get('phase_step_count'),
                'boundary_label': boundary,
                'legacy_leg_id': legacy_leg_id,
                'leg_name': leg_name,
                'joint_state_index': idx,
                'joint_index': joint_index,
                'joint_name': 'thigh',
                'abs_angle_deg': abs_deg,
                'limit_deg': limit_deg,
                'excess_deg': excess,
                'is_violation': is_violation,
            }
            samples.append(item)
            if is_violation:
                violations.append(item)
                violation_frame_keys.add(frame_key)
            if worst is None or abs_deg > worst['abs_angle_deg']:
                worst = dict(item)

            _update_group(groups, 'roll_index', roll_key, item, frame_key)
            _update_group(groups, 'phase_name', phase, item, frame_key)
            _update_group(groups, 'leg_name', leg_name, item, frame_key)
            _update_group(groups, 'roll_phase', '%s|%s' % (roll_key, phase), item, frame_key)
            _update_group(groups, 'leg_phase', '%s|%s' % (leg_name, phase), item, frame_key)
            _update_group(groups, 'boundary_window', boundary, item, frame_key)

            # Frame count for leg-specific groups: count each source record once per leg.
            for gt, gk in (('leg_name', leg_name), ('leg_phase', '%s|%s' % (leg_name, phase))):
                k = (gt, _safe_key(gk))
                # Add a private per-frame marker set lazily to avoid double-counting.
                marker_name = '_frame_marker'
                if marker_name not in groups[k]:
                    groups[k][marker_name] = set()
                if frame_key not in groups[k][marker_name]:
                    groups[k][marker_name].add(frame_key)
                    groups[k]['frame_count'] += 1

    # Clean private marker sets.
    finalized = []
    for g in groups.values():
        if '_frame_marker' in g:
            del g['_frame_marker']
        finalized.append(_finalize_group(g))

    by_type = {}
    for g in finalized:
        by_type.setdefault(g['group_type'], []).append(g)
    for k in by_type:
        by_type[k] = sorted(
            by_type[k],
            key=lambda x: (x['max_abs_angle_deg'], x['violation_count'], x['sample_count']),
            reverse=True)

    return {
        'source_label': source_label,
        'frame_count': len(records),
        'second_joint_sample_count': len(samples),
        'limit_deg': limit_deg,
        'max_abs_angle_deg': 0.0 if worst is None else worst['abs_angle_deg'],
        'max_excess_deg': 0.0 if worst is None else max(0.0, worst['abs_angle_deg'] - limit_deg),
        'violation_count': len(violations),
        'violation_frame_count': len(violation_frame_keys),
        'violation_sample_rate': 0.0 if not samples else float(len(violations)) / float(len(samples)),
        'violation_frame_rate': 0.0 if not records else float(len(violation_frame_keys)) / float(len(records)),
        'worst': worst,
        'group_summary': by_type,
        'top_worst_samples': sorted(samples, key=lambda x: x['abs_angle_deg'], reverse=True)[:top_n],
        'top_violations': sorted(violations, key=lambda x: x['excess_deg'], reverse=True)[:top_n],
        'boundary_window_frames': boundary_window,
        'roll_boundary_record_indices': sorted(list(roll_boundary_indices))[:200],
        'phase_boundary_record_indices': sorted(list(phase_boundary_indices))[:200],
    }


def _compare_sources(raw_result, filtered_result):
    if not raw_result or not filtered_result:
        return None
    return {
        'note': 'Counts are not directly comparable when resample factors differ. Prefer max_abs_angle_deg and violation_sample_rate/frame_rate for raw-vs-filtered tendency, and use worst samples for localization.',
        'raw_max_abs_angle_deg': raw_result.get('max_abs_angle_deg'),
        'filtered_max_abs_angle_deg': filtered_result.get('max_abs_angle_deg'),
        'delta_filtered_minus_raw_max_deg': None if raw_result.get('max_abs_angle_deg') is None or filtered_result.get('max_abs_angle_deg') is None else filtered_result.get('max_abs_angle_deg') - raw_result.get('max_abs_angle_deg'),
        'raw_violation_count': raw_result.get('violation_count'),
        'filtered_violation_count': filtered_result.get('violation_count'),
        'raw_frame_count': raw_result.get('frame_count'),
        'filtered_frame_count': filtered_result.get('frame_count'),
        'raw_violation_sample_rate': raw_result.get('violation_sample_rate'),
        'filtered_violation_sample_rate': filtered_result.get('violation_sample_rate'),
        'raw_violation_frame_rate': raw_result.get('violation_frame_rate'),
        'filtered_violation_frame_rate': filtered_result.get('violation_frame_rate'),
    }


def main():
    ap = argparse.ArgumentParser(description='v3.0.41 second joint angle violation localization. This is a diagnostic script; it does not modify gait generation.')
    ap.add_argument('--raw-command-log', default=None, help='Existing raw JSONL command log. If omitted, generate baseline raw records.')
    ap.add_argument('--filtered-command-log', default=None, help='Existing filtered JSONL command log. If omitted, create filtered records from raw using resample/smoothing options.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--rf1-current-angle-anchor', action='store_true', default=True)
    ap.add_argument('--no-rf1-current-angle-anchor', action='store_false', dest='rf1_current_angle_anchor')
    ap.add_argument('--goal2-dist-front', type=float, default=0.4)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-pitch-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--middle-swing-y-escape', type=float, default=0.0)
    ap.add_argument('--middle-swing-y-escape-mode', default='none',
                    choices=['none', 'outward', 'inward', 'same_sign_plus', 'same_sign_minus'])
    ap.add_argument('--middle-swing-y-escape-apply-rf3', action='store_true')
    ap.add_argument('--middle-swing-y-escape-apply-rf4', action='store_true')
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)
    ap.add_argument('--resample-factor', type=int, default=8)
    ap.add_argument('--smooth-window', type=int, default=40)
    ap.add_argument('--segment-key', default='', help='Leave empty for current baseline: smooth the full 4-roll stream without roll_index segmentation.')
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--boundary-window', type=int, default=3, help='Record distance used to classify samples near roll/phase boundaries.')
    ap.add_argument('--top-n', type=int, default=30)
    ap.add_argument('--output-raw-command-log', default='testdata/v3_0_41_baseline_raw_commands.jsonl')
    ap.add_argument('--output-filtered-command-log', default='testdata/v3_0_41_baseline_x8_sw40_commands.jsonl')
    ap.add_argument('--report-output', default='testdata/v3_0_41_second_joint_angle_localization_report.json')
    args = ap.parse_args()

    json_stdout = sys.stdout
    sys.stdout = sys.stderr

    generation_completed = None
    generation_error = None
    if args.raw_command_log:
        raw_records = load_command_records(args.raw_command_log)
        raw_source = args.raw_command_log
    else:
        raw_records, generation_completed, generation_error = _make_legacy_repeated_records(args)
        raw_source = 'generated_v3_0_36_pure_legacy_repeated_roll'

    if args.output_raw_command_log:
        _ensure_dir(args.output_raw_command_log)
        write_jsonl(raw_records, args.output_raw_command_log)

    if args.filtered_command_log:
        filtered_records = load_command_records(args.filtered_command_log)
        filtered_source = args.filtered_command_log
    else:
        segment_key = args.segment_key.strip() or None
        filtered_records = resample_command_records(raw_records, factor=args.resample_factor, segment_key=segment_key)
        filtered_records = moving_average_command_records(filtered_records, window=args.smooth_window, segment_key=segment_key)
        filtered_source = 'generated_from_raw_resample_and_smooth'
        if args.output_filtered_command_log:
            _ensure_dir(args.output_filtered_command_log)
            write_command_records(filtered_records, args.output_filtered_command_log)

    raw_result = _analyze_source(raw_records, 'raw', args.second_joint_abs_max_deg, args.boundary_window, args.top_n)
    filtered_result = _analyze_source(filtered_records, 'filtered', args.second_joint_abs_max_deg, args.boundary_window, args.top_n)

    report = {
        'version_note': 'v3.0.41: second_joint_angle_violation_localization. Diagnostic only; no gait command generation behavior is changed except optional baseline generation for input preparation.',
        'roadmap_position': {
            'phase': 'Phase 1',
            'name': 'v3.0.41 diagnostic implementation',
            'purpose': 'Localize the remaining second joint angle > 95 deg issue before changing gait parameters.',
            'next_phase': 'Phase 2: inspect roll/phase/leg/frame localization and choose minimal countermeasure.',
        },
        'inputs': {
            'raw_source': raw_source,
            'filtered_source': filtered_source,
            'generated_candidate_completed': generation_completed,
            'generation_error': generation_error,
        },
        'baseline_parameters': {
            'surface_sequence': args.surface_sequence,
            'move_dist': args.move_dist,
            'support_dist': args.support_dist,
            'legacy_body_z': args.legacy_body_z,
            'max_step': args.max_step,
            'rf1_current_angle_anchor': args.rf1_current_angle_anchor,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'segment_key': args.segment_key,
            'second_joint_abs_max_deg': args.second_joint_abs_max_deg,
            'boundary_window': args.boundary_window,
        },
        'joint_order_note': 'joint_command_rad is in JOINT_STATE_ORDER, not legacy leg_id order. This script only evaluates entries whose joint_index == 1 in JOINT_STATE_ORDER.',
        'raw': raw_result,
        'filtered': filtered_result,
        'raw_filtered_comparison': _compare_sources(raw_result, filtered_result),
        'interpretation_notes': [
            'Do not compare raw and filtered violation_count directly when resample factors differ.',
            'A high violation concentration in one phase suggests a local gait/IK geometry issue there.',
            'If raw is already above 95 deg, smoothing alone is not the root fix.',
            'This report intentionally does not choose a countermeasure; use it to select v3.0.42 candidates.',
        ],
    }

    if args.report_output:
        _ensure_dir(args.report_output)
        with open(args.report_output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)

    sys.stdout = json_stdout
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
