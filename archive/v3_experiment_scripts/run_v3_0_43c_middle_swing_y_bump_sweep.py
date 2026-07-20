#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

"""v3.0.43C middle_swing_y_bump_sweep.

Purpose:
  Test a simple, reversible middle-leg swing Y bump.  The intended geometry is:

      baseline v2 at lift-off  ->  outward Y apex during swing  ->  baseline v2 at landing

  In practice this uses the existing RF-3 apex target for the middle pair and
  keeps RF-4 landing at the provisional baseline-v2 Y coordinate.  Therefore
  the contact/landing target coordinate is explicitly returned to baseline v2.

Design rule:
  This is intentionally a minimum-difference experiment.  It does not refactor
  the sweep pipeline and it does not change IK/support/landing algorithms.
  With middle_swing_y_escape=0.0 and mode=none, generation must reproduce the
  provisional baseline v2 command sequence.  Use archive/v3_experiment_scripts/run_v3_0_provisional_baseline_v2.py
  at any time to regenerate the unchanged baseline v2 logs.

Recommended first pass:
  python archive/v3_experiment_scripts/run_v3_0_43c_middle_swing_y_bump_sweep.py \
    --middle-swing-y-escapes 0.00,0.05,0.10,0.15,0.20 \
    --middle-swing-y-escape-modes outward \
    --middle-swing-y-escape-phases rf3_only
"""

import argparse
import itertools
import json
import math
import multiprocessing
import os
import shutil
import subprocess
import tempfile
try:
    from shlex import quote as _shell_quote
except ImportError:
    from pipes import quote as _shell_quote
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import write_jsonl
from lily_motion_v3.command_resampler import (
    resample_command_records, moving_average_command_records,
    write_command_records)

import run_v3_0_41_second_joint_angle_localization as loc
import run_v3_0_baseline_filtered_constraint_eval as base_eval
import run_v3_0_42c_candidate03_local_refine as c42


class Obj(object):
    pass


def parse_float_list(text):
    if text is None:
        return []
    vals = []
    for x in str(text).split(','):
        x = x.strip()
        if x:
            vals.append(float(x))
    return vals


def parse_string_list(text):
    if text is None:
        return []
    vals = []
    for x in str(text).split(','):
        x = x.strip()
        if x:
            vals.append(x)
    return vals


def ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


def make_apply_flags(phase_mode):
    phase_mode = str(phase_mode)
    if phase_mode == 'none':
        return False, False
    if phase_mode == 'rf3_only':
        return True, False
    if phase_mode == 'rf4_only':
        return False, True
    if phase_mode == 'rf3_rf4':
        return True, True
    raise ValueError('unknown middle_swing_y_escape_phase: %s' % phase_mode)


def make_case_key(case):
    order = [
        'middle_swing_y_escape',
        'middle_swing_y_escape_mode',
        'middle_swing_y_escape_phase',
        'body_z', 'support_dist', 'move_dist',
        'goal2_dist_front', 'goal2_x_scale', 'goal2_pitch_scale',
        'goal3_lift_z', 'goal3_target_x', 'goal4_target_x',
        'goal5_x_scale', 'goal5_pitch_scale']
    parts = []
    for k in order:
        if k in case:
            parts.append('%s=%s' % (k, case[k]))
    return ','.join(parts)


def make_loc_args(args, case):
    a = Obj()
    a.surface_sequence = args.surface_sequence
    a.move_dist = float(case.get('move_dist', args.move_dist))
    a.support_dist = float(case.get('support_dist', args.support_dist))
    a.legacy_body_z = float(case.get('body_z', args.legacy_body_z))
    a.max_step = args.max_step
    a.initialize_step = args.initialize_step
    a.include_initialize = args.include_initialize
    a.rf1_current_angle_anchor = args.rf1_current_angle_anchor
    a.goal2_dist_front = float(case.get('goal2_dist_front', args.goal2_dist_front))
    a.goal2_x_scale = float(case.get('goal2_x_scale', args.goal2_x_scale))
    a.goal2_pitch_scale = float(case.get('goal2_pitch_scale', args.goal2_pitch_scale))
    a.goal2_landing_z = float(case.get('goal2_landing_z', args.goal2_landing_z))
    a.goal3_lift_z = float(case.get('goal3_lift_z', args.goal3_lift_z))
    a.goal3_target_x = float(case.get('goal3_target_x', args.goal3_target_x))
    a.goal4_target_x = float(case.get('goal4_target_x', args.goal4_target_x))
    a.goal5_x_scale = float(case.get('goal5_x_scale', args.goal5_x_scale))
    a.goal5_pitch_scale = float(case.get('goal5_pitch_scale', args.goal5_pitch_scale))
    a.middle_swing_y_escape = float(case.get('middle_swing_y_escape', 0.0))
    a.middle_swing_y_escape_mode = str(case.get('middle_swing_y_escape_mode', 'none'))
    rf3, rf4 = make_apply_flags(case.get('middle_swing_y_escape_phase', 'none'))
    a.middle_swing_y_escape_apply_rf3 = rf3
    a.middle_swing_y_escape_apply_rf4 = rf4
    return a


def make_constraint_args(args, loc_args):
    a = Obj()
    a.second_joint_abs_max_deg = args.second_joint_abs_max_deg
    a.ground_z = args.ground_z
    a.ground_tolerance = args.ground_tolerance
    a.inter_leg_limit = args.inter_leg_limit
    a.inter_leg_link_radius = args.inter_leg_link_radius
    a.inter_leg_safety_margin = args.inter_leg_safety_margin
    a.inter_leg_joint_housing_radius = args.inter_leg_joint_housing_radius
    a.inter_leg_joint_housing_safety_margin = args.inter_leg_joint_housing_safety_margin
    a.constraint_stride = args.constraint_stride
    a.top_n = args.top_n
    a.legacy_body_z = loc_args.legacy_body_z
    return a


def phase_summary_item(ph):
    if not ph:
        return None
    cbp = ph.get('clearance_by_part') or {}
    sj = cbp.get('second_joint') or {}
    ft = cbp.get('foot') or {}
    return {
        'phase_name': ph.get('phase_name'),
        'frame_count': ph.get('frame_count'),
        'max_second_joint_deg': ph.get('max_second_joint_deg'),
        'second_joint_violation_count': ph.get('second_joint_violation_count'),
        'ground_penetration_count': ph.get('ground_penetration_count'),
        'min_clearance_m': ph.get('min_clearance_m'),
        'second_joint_min_clearance_m': sj.get('min_clearance_m'),
        'second_joint_penetration_count': sj.get('penetration_count'),
        'foot_min_clearance_m': ft.get('min_clearance_m'),
        'foot_penetration_count': ft.get('penetration_count'),
        'min_inter_leg_distance_m': ph.get('min_inter_leg_distance_m'),
        'min_joint_housing_distance_m': ph.get('min_joint_housing_distance_m'),
        'inter_leg_collision_count': ph.get('inter_leg_collision_count'),
        'inter_leg_near_count': ph.get('inter_leg_near_count'),
        'joint_housing_collision_count': ph.get('joint_housing_collision_count'),
        'joint_housing_near_count': ph.get('joint_housing_near_count'),
    }


def middle_phase_metrics(rep):
    phases = rep.get('phase_summary') or []
    middle = []
    for ph in phases:
        name = str(ph.get('phase_name', ''))
        if 'RF-3' in name or 'RF-4' in name:
            middle.append(phase_summary_item(ph))
    vals_inter = [x.get('min_inter_leg_distance_m') for x in middle if x and x.get('min_inter_leg_distance_m') is not None]
    vals_house = [x.get('min_joint_housing_distance_m') for x in middle if x and x.get('min_joint_housing_distance_m') is not None]
    return {
        'phase_summary': middle,
        'min_inter_leg_distance_m': None if not vals_inter else min(vals_inter),
        'min_joint_housing_distance_m': None if not vals_house else min(vals_house),
        'inter_leg_collision_count': sum([(x.get('inter_leg_collision_count') or 0) for x in middle]),
        'inter_leg_near_count': sum([(x.get('inter_leg_near_count') or 0) for x in middle]),
        'joint_housing_collision_count': sum([(x.get('joint_housing_collision_count') or 0) for x in middle]),
        'joint_housing_near_count': sum([(x.get('joint_housing_near_count') or 0) for x in middle]),
        'second_joint_violation_count': sum([(x.get('second_joint_violation_count') or 0) for x in middle]),
        'ground_penetration_count': sum([(x.get('ground_penetration_count') or 0) for x in middle]),
    }


def compact_constraints_43(rep):
    if not rep:
        return None
    base = c42.compact_constraints(rep)
    base['inter_leg_min_distance_m'] = ((rep.get('inter_leg_collision') or {}).get('min_distance_m'))
    base['joint_housing_min_distance_m'] = ((rep.get('inter_leg_joint_housing_collision') or {}).get('min_distance_m'))
    base['middle_phase'] = middle_phase_metrics(rep)
    base['phase_summary_rf3_rf4'] = base['middle_phase'].get('phase_summary')
    base['top_inter_leg_near'] = (rep.get('top_inter_leg_near') or [])[:8]
    base['top_inter_leg_joint_housing_near'] = (rep.get('top_inter_leg_joint_housing_near') or [])[:8]
    return base


def safe_ratio(value, baseline, default=1.0):
    return c42.ratio(value, baseline, default=default)


def safe_delta(value, baseline, default=0.0):
    try:
        if value is None or baseline is None:
            return default
        return float(value) - float(baseline)
    except Exception:
        return default


def score_candidate(item, baseline, args):
    """Lower is better.  v3.0.43C prioritizes middle RF-3/RF-4 clearance as a screening metric only."""
    if not item.get('candidate_completed'):
        return 1e99

    filt = item.get('filtered') or {}
    con = item.get('constraint_filtered') or {}
    sm = item.get('smoothness_filtered') or {}
    cf = item.get('compact_flip_filtered') or {}
    mid = con.get('middle_phase') or {}

    b_filt = baseline.get('filtered') or {}
    b_con = baseline.get('constraint_filtered') or {}
    b_sm = baseline.get('smoothness_filtered') or {}
    b_cf = baseline.get('compact_flip_filtered') or {}
    b_mid = (b_con.get('middle_phase') or {})

    score = 0.0

    # Hard physical failures dominate.
    score += (con.get('inter_leg_collision_count') or 0) * 50000000.0
    score += (con.get('inter_leg_joint_housing_collision_count') or 0) * 50000000.0
    score += (mid.get('inter_leg_collision_count') or 0) * 60000000.0
    score += (mid.get('joint_housing_collision_count') or 0) * 60000000.0

    # Middle-phase near counts are the main v3.0.43C screening target.
    score += (mid.get('inter_leg_near_count') or 0) * 3000000.0
    score += (mid.get('joint_housing_near_count') or 0) * 3000000.0

    # Reward increased middle-phase minimum distances versus escape=0 baseline.
    inter_delta = safe_delta(mid.get('min_inter_leg_distance_m'), b_mid.get('min_inter_leg_distance_m'))
    house_delta = safe_delta(mid.get('min_joint_housing_distance_m'), b_mid.get('min_joint_housing_distance_m'))
    score -= inter_delta * 80000000.0
    score -= house_delta * 80000000.0

    # Do not accept angle or floor regressions just because clearance improved.
    angle = filt.get('max_abs_angle_deg', 999.0)
    base_angle = b_filt.get('max_abs_angle_deg', angle)
    if angle > args.second_joint_abs_max_deg:
        score += (angle - args.second_joint_abs_max_deg) * 3000000.0
    if angle > base_angle + args.max_angle_regression_deg:
        score += (angle - base_angle - args.max_angle_regression_deg) * 5000000.0

    score += (con.get('ground_penetration_count') or 0) * 200000.0
    score += (con.get('foot_penetration_count') or 0) * 50000.0
    score += (mid.get('ground_penetration_count') or 0) * 300000.0

    # Smoothness / compact flip regressions are soft but visible in Gazebo.
    adj_ratio = safe_ratio(sm.get('max_adjacent_delta_deg'), b_sm.get('max_adjacent_delta_deg'))
    d2_ratio = safe_ratio(sm.get('max_second_diff_deg'), b_sm.get('max_second_diff_deg'))
    roll_b_ratio = safe_ratio(sm.get('roll_boundary_max_delta_deg'), b_sm.get('roll_boundary_max_delta_deg'))
    flip_ratio = safe_ratio(cf.get('max_flip_extension_score'), b_cf.get('max_flip_extension_score'))
    support_ext = cf.get('max_support_phase_two_link_extension_ratio') or 0.0

    for r, limit, weight in [
            (adj_ratio, args.max_adjacent_delta_ratio, 1200000.0),
            (d2_ratio, args.max_second_diff_ratio, 2000000.0),
            (roll_b_ratio, args.max_boundary_delta_ratio, 1000000.0),
            (flip_ratio, args.max_flip_extension_ratio, 2000000.0)]:
        if r > limit:
            score += (r - limit) * weight
    if support_ext > args.max_support_extension_ratio:
        score += (support_ext - args.max_support_extension_ratio) * 8000000.0

    return score


def gate_status(item, baseline, args):
    filt = item.get('filtered') or {}
    con = item.get('constraint_filtered') or {}
    sm = item.get('smoothness_filtered') or {}
    mid = (con.get('middle_phase') or {})
    b_filt = baseline.get('filtered') or {}
    b_con = baseline.get('constraint_filtered') or {}
    b_sm = baseline.get('smoothness_filtered') or {}
    b_mid = (b_con.get('middle_phase') or {})

    checks = []
    def add(name, ok, value=None, limit=None):
        checks.append({'name': name, 'ok': bool(ok), 'value': value, 'limit': limit})

    add('candidate_completed', item.get('candidate_completed'), item.get('candidate_completed'), True)
    add('angle_below_limit', filt.get('max_abs_angle_deg', 999.0) <= args.second_joint_abs_max_deg, filt.get('max_abs_angle_deg'), args.second_joint_abs_max_deg)
    add('angle_not_regressed', filt.get('max_abs_angle_deg', 999.0) <= (b_filt.get('max_abs_angle_deg', 999.0) + args.max_angle_regression_deg), filt.get('max_abs_angle_deg'), b_filt.get('max_abs_angle_deg'))
    add('middle_inter_leg_distance_not_worse', (mid.get('min_inter_leg_distance_m') is None or b_mid.get('min_inter_leg_distance_m') is None or mid.get('min_inter_leg_distance_m') >= b_mid.get('min_inter_leg_distance_m') - args.max_middle_distance_regression_m), mid.get('min_inter_leg_distance_m'), b_mid.get('min_inter_leg_distance_m'))
    add('middle_joint_housing_distance_not_worse', (mid.get('min_joint_housing_distance_m') is None or b_mid.get('min_joint_housing_distance_m') is None or mid.get('min_joint_housing_distance_m') >= b_mid.get('min_joint_housing_distance_m') - args.max_middle_distance_regression_m), mid.get('min_joint_housing_distance_m'), b_mid.get('min_joint_housing_distance_m'))
    add('no_inter_leg_collision', (con.get('inter_leg_collision_count') or 0) == 0, con.get('inter_leg_collision_count'), 0)
    add('no_joint_housing_collision', (con.get('inter_leg_joint_housing_collision_count') or 0) == 0, con.get('inter_leg_joint_housing_collision_count'), 0)
    add('middle_no_inter_leg_collision', (mid.get('inter_leg_collision_count') or 0) == 0, mid.get('inter_leg_collision_count'), 0)
    add('middle_no_joint_housing_collision', (mid.get('joint_housing_collision_count') or 0) == 0, mid.get('joint_housing_collision_count'), 0)
    add('ground_penetration_not_worse_than_baseline', (con.get('ground_penetration_count') or 0) <= (b_con.get('ground_penetration_count') or 0), con.get('ground_penetration_count'), b_con.get('ground_penetration_count'))
    add('adjacent_delta_ratio_ok', safe_ratio(sm.get('max_adjacent_delta_deg'), b_sm.get('max_adjacent_delta_deg')) <= args.max_adjacent_delta_ratio, safe_ratio(sm.get('max_adjacent_delta_deg'), b_sm.get('max_adjacent_delta_deg')), args.max_adjacent_delta_ratio)
    add('second_diff_ratio_ok', safe_ratio(sm.get('max_second_diff_deg'), b_sm.get('max_second_diff_deg')) <= args.max_second_diff_ratio, safe_ratio(sm.get('max_second_diff_deg'), b_sm.get('max_second_diff_deg')), args.max_second_diff_ratio)

    hard_names = set([
        'candidate_completed',
        'angle_below_limit',
        'angle_not_regressed',
        'middle_inter_leg_distance_not_worse',
        'middle_joint_housing_distance_not_worse',
        'no_inter_leg_collision',
        'no_joint_housing_collision',
        'middle_no_inter_leg_collision',
        'middle_no_joint_housing_collision',
        'ground_penetration_not_worse_than_baseline',
        'second_diff_ratio_ok'])
    hard = [c for c in checks if c['name'] in hard_names]
    return {
        'checks': checks,
        'hard_gate_ok': all(c['ok'] for c in hard),
        'gazebo_replay_required': True,
        'note': 'v3.0.43C gate is for screening only. It checks middle RF-3/RF-4 proximity and basic non-regression; Gazebo strict replay and 3rd/4th-roll jump inspection remain mandatory.',
    }


def evaluate_case(args, case, label):
    loc_args = make_loc_args(args, case)
    item = {'case': dict(case), 'case_key': make_case_key(case), 'label': label}
    raw_records = None
    filtered_records = None
    try:
        raw_records, completed, error = loc._make_legacy_repeated_records(loc_args)
        item['candidate_completed'] = completed
        item['generation_error'] = error
        item['raw'] = c42.compact_loc(loc._analyze_source(raw_records, 'raw', args.second_joint_abs_max_deg, args.boundary_window, args.top_n))
        item['smoothness_raw'] = c42.command_smoothness_summary(raw_records)
        item['compact_flip_raw'] = c42.compact_flip_extension_summary(raw_records)

        segment_key = args.segment_key.strip() or None
        filtered_records = resample_command_records(raw_records, factor=args.resample_factor, segment_key=segment_key)
        filtered_records = moving_average_command_records(filtered_records, window=args.smooth_window, segment_key=segment_key)
        item['filtered'] = c42.compact_loc(loc._analyze_source(filtered_records, 'filtered', args.second_joint_abs_max_deg, args.boundary_window, args.top_n))
        item['smoothness_filtered'] = c42.command_smoothness_summary(filtered_records)
        item['compact_flip_filtered'] = c42.compact_flip_extension_summary(filtered_records)

        cargs = make_constraint_args(args, loc_args)
        if args.evaluate_raw_constraints:
            item['constraint_raw'] = compact_constraints_43(base_eval._evaluate_constraints(raw_records, cargs, 'raw'))
        item['constraint_filtered'] = compact_constraints_43(base_eval._evaluate_constraints(filtered_records, cargs, 'filtered_gazebo_command'))
    except Exception as e:
        item['candidate_completed'] = False
        item['generation_error'] = {'type': e.__class__.__name__, 'message': str(e)}
    return item, raw_records, filtered_records


def _evaluate_case_worker(args, case, label, queue):
    try:
        queue.put(evaluate_case(args, case, label))
    except Exception as e:
        queue.put(({'case': dict(case),
                    'case_key': make_case_key(case),
                    'label': label,
                    'candidate_completed': False,
                    'generation_error': {'type': e.__class__.__name__, 'message': str(e)}},
                   None, None))


def evaluate_case_isolated(args, case, label):
    """Evaluate one case in a fresh process.

    The vendored legacy runtime is not robust when multiple full repeated-roll
    generations are executed in one long-lived Python process.  v3.0.43C keeps
    the motion code unchanged and isolates each case instead of refactoring the
    legacy runtime.  This preserves the minimum-difference experiment while
    avoiding cross-case global-state contamination.
    """
    q = multiprocessing.Queue()
    p = multiprocessing.Process(target=_evaluate_case_worker, args=(args, case, label, q))
    p.start()
    item, raw_records, filtered_records = q.get()
    p.join()
    if p.exitcode not in (0, None) and item.get('candidate_completed') is not False:
        item['candidate_completed'] = False
        item['generation_error'] = {'type': 'ChildProcessExit', 'message': 'exitcode=%s' % p.exitcode}
    return item, raw_records, filtered_records


def evaluate_case_subprocess(args, case, label, work_dir, tag):
    """Run one case through a fresh Python interpreter and read its JSON item."""
    if not os.path.isdir(work_dir):
        os.makedirs(work_dir)
    case_path = os.path.join(work_dir, '%s_case.json' % tag)
    item_path = os.path.join(work_dir, '%s_item.json' % tag)
    raw_path = os.path.join(work_dir, '%s_raw_commands.jsonl' % tag)
    filt_path = os.path.join(work_dir, '%s_x%d_sw%d_commands.jsonl' % (tag, args.resample_factor, args.smooth_window))
    with open(case_path, 'w') as f:
        json.dump(case, f, indent=2, sort_keys=True)

    cmd = [
        sys.executable, os.path.abspath(__file__),
        '--single-case-file', case_path,
        '--single-case-label', label,
        '--single-case-output', item_path,
        '--single-case-raw-output', raw_path,
        '--single-case-filtered-output', filt_path,
        '--surface-sequence', args.surface_sequence,
        '--move-dist', str(args.move_dist),
        '--support-dist', str(args.support_dist),
        '--legacy-body-z', str(args.legacy_body_z),
        '--max-step', str(args.max_step),
        '--initialize-step', str(args.initialize_step),
        '--goal2-dist-front', str(args.goal2_dist_front),
        '--goal2-x-scale', str(args.goal2_x_scale),
        '--goal2-pitch-scale', str(args.goal2_pitch_scale),
        '--goal2-landing-z', str(args.goal2_landing_z),
        '--goal3-lift-z', str(args.goal3_lift_z),
        '--goal3-target-x', str(args.goal3_target_x),
        '--goal4-target-x', str(args.goal4_target_x),
        '--goal5-x-scale', str(args.goal5_x_scale),
        '--goal5-pitch-scale', str(args.goal5_pitch_scale),
        '--resample-factor', str(args.resample_factor),
        '--smooth-window', str(args.smooth_window),
        '--segment-key', args.segment_key,
        '--second-joint-abs-max-deg', str(args.second_joint_abs_max_deg),
        '--boundary-window', str(args.boundary_window),
        '--top-n', str(args.top_n),
        '--ground-z', str(args.ground_z),
        '--ground-tolerance', str(args.ground_tolerance),
        '--inter-leg-limit', str(args.inter_leg_limit),
        '--inter-leg-link-radius', str(args.inter_leg_link_radius),
        '--inter-leg-safety-margin', str(args.inter_leg_safety_margin),
        '--inter-leg-joint-housing-radius', str(args.inter_leg_joint_housing_radius),
        '--inter-leg-joint-housing-safety-margin', str(args.inter_leg_joint_housing_safety_margin),
        '--constraint-stride', str(args.constraint_stride),
    ]
    if args.include_initialize:
        cmd.append('--include-initialize')
    if args.rf1_current_angle_anchor:
        cmd.append('--rf1-current-angle-anchor')
    else:
        cmd.append('--no-rf1-current-angle-anchor')
    if args.evaluate_raw_constraints:
        cmd.append('--evaluate-raw-constraints')

    cmd_text = ' '.join([_shell_quote(str(x)) for x in cmd])
    with open(os.devnull, 'w') as devnull:
        subprocess.check_call(cmd_text, stdout=devnull, stderr=devnull, shell=True)
    print('Finished worker %s' % tag, file=sys.stderr)
    item = json.load(open(item_path))
    return item, raw_path, filt_path


def main():
    ap = argparse.ArgumentParser(description='v3.0.43C middle_swing_y_bump_sweep: reversible RF-3-only swing Y bump from provisional baseline v2.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')

    # Provisional baseline v2 / v3.0.42C case27 defaults.
    ap.add_argument('--move-dist', type=float, default=0.40)
    ap.add_argument('--support-dist', type=float, default=0.77)
    ap.add_argument('--legacy-body-z', type=float, default=0.40)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--rf1-current-angle-anchor', action='store_true', default=True)
    ap.add_argument('--no-rf1-current-angle-anchor', action='store_false', dest='rf1_current_angle_anchor')

    ap.add_argument('--goal2-dist-front', type=float, default=0.40)
    ap.add_argument('--goal2-x-scale', type=float, default=0.95)
    ap.add_argument('--goal2-pitch-scale', type=float, default=0.90)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.20)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)

    ap.add_argument('--middle-swing-y-escapes', default='0.00,0.05,0.10,0.15,0.20')
    ap.add_argument('--middle-swing-y-escape-modes', default='outward', help='Comma list: outward,inward,same_sign_plus,same_sign_minus')
    ap.add_argument('--middle-swing-y-escape-phases', default='rf3_only', help='Comma list: rf3_only,rf4_only,rf3_rf4')

    ap.add_argument('--resample-factor', type=int, default=8)
    ap.add_argument('--smooth-window', type=int, default=40)
    ap.add_argument('--segment-key', default='', help='Leave empty: do not split smoothing by roll_index.')
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--boundary-window', type=int, default=3)
    ap.add_argument('--top-n', type=int, default=12)

    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--inter-leg-link-radius', type=float, default=0.015)
    ap.add_argument('--inter-leg-safety-margin', type=float, default=0.010)
    ap.add_argument('--inter-leg-joint-housing-radius', type=float, default=0.030)
    ap.add_argument('--inter-leg-joint-housing-safety-margin', type=float, default=0.005)
    ap.add_argument('--constraint-stride', type=int, default=8)
    ap.add_argument('--evaluate-raw-constraints', action='store_true')

    ap.add_argument('--max-angle-regression-deg', type=float, default=0.5)
    ap.add_argument('--max-middle-distance-regression-m', type=float, default=0.001)
    ap.add_argument('--max-adjacent-delta-ratio', type=float, default=1.10)
    ap.add_argument('--max-second-diff-ratio', type=float, default=1.20)
    ap.add_argument('--max-boundary-delta-ratio', type=float, default=1.25)
    ap.add_argument('--max-support-extension-ratio', type=float, default=0.99995)
    ap.add_argument('--max-flip-extension-ratio', type=float, default=1.10)

    ap.add_argument('--output', default='testdata/v3_0_43c_middle_swing_y_bump_sweep.json')
    ap.add_argument('--candidate-output-dir', default='testdata/v3_0_43c_candidates')
    ap.add_argument('--save-top-n', type=int, default=3)

    # Hidden worker mode used by the normal sweep.  The vendored legacy runtime
    # is safest when each full repeated-roll generation is run in a fresh
    # interpreter process.
    ap.add_argument('--single-case-file', default=None)
    ap.add_argument('--single-case-label', default='single_case')
    ap.add_argument('--single-case-output', default=None)
    ap.add_argument('--single-case-raw-output', default=None)
    ap.add_argument('--single-case-filtered-output', default=None)

    args = ap.parse_args()

    if args.single_case_file:
        case = json.load(open(args.single_case_file))
        item, raw_records, filtered_records = evaluate_case(args, case, args.single_case_label)
        if args.single_case_raw_output and raw_records is not None:
            ensure_dir(args.single_case_raw_output)
            write_jsonl(raw_records, args.single_case_raw_output)
            item['raw_command_log'] = args.single_case_raw_output
        if args.single_case_filtered_output and filtered_records is not None:
            ensure_dir(args.single_case_filtered_output)
            write_command_records(filtered_records, args.single_case_filtered_output)
            item['filtered_command_log'] = args.single_case_filtered_output
        if args.single_case_output:
            ensure_dir(args.single_case_output)
            with open(args.single_case_output, 'w') as f:
                json.dump(item, f, indent=2, sort_keys=True)
        print(json.dumps(item, indent=2, sort_keys=True))
        return

    json_stdout = sys.stdout
    sys.stdout = sys.stderr

    baseline_case = {
        'body_z': args.legacy_body_z,
        'support_dist': args.support_dist,
        'move_dist': args.move_dist,
        'goal2_dist_front': args.goal2_dist_front,
        'goal2_x_scale': args.goal2_x_scale,
        'goal2_pitch_scale': args.goal2_pitch_scale,
        'goal2_landing_z': args.goal2_landing_z,
        'goal3_lift_z': args.goal3_lift_z,
        'goal3_target_x': args.goal3_target_x,
        'goal4_target_x': args.goal4_target_x,
        'goal5_x_scale': args.goal5_x_scale,
        'goal5_pitch_scale': args.goal5_pitch_scale,
        'middle_swing_y_escape': 0.0,
        'middle_swing_y_escape_mode': 'none',
        'middle_swing_y_escape_phase': 'none',
    }

    print('Evaluating v3.0.43C bump=0 baseline...', file=sys.stderr)
    worker_dir = tempfile.mkdtemp(prefix='v3_0_43c_', dir=os.path.dirname(args.output) or None)
    baseline, baseline_raw_path, baseline_filtered_path = evaluate_case_subprocess(args, baseline_case, 'baseline_escape0', worker_dir, 'baseline')

    escapes = parse_float_list(args.middle_swing_y_escapes)
    modes = parse_string_list(args.middle_swing_y_escape_modes)
    phase_modes = parse_string_list(args.middle_swing_y_escape_phases)
    cases = []
    for esc, mode, phase_mode in itertools.product(escapes, modes, phase_modes):
        # escape=0 is already evaluated once as baseline_escape0.  Do not run
        # it again as a sweep case; repeated full legacy generations are slow
        # and the baseline item is the clearer comparison reference.
        if abs(float(esc)) < 1e-12:
            continue
        case = dict(baseline_case)
        case['middle_swing_y_escape'] = float(esc)
        case['middle_swing_y_escape_mode'] = mode
        case['middle_swing_y_escape_phase'] = phase_mode
        if make_case_key(case) not in [make_case_key(c) for c in cases]:
            cases.append(case)

    results = []
    records_by_case_index = {}
    for idx, case in enumerate(cases):
        print('Evaluating swing Y bump case %d/%d: %s' % (idx + 1, len(cases), make_case_key(case)), file=sys.stderr)
        item, raw_path, filtered_path = evaluate_case_subprocess(args, case, 'case_%d' % idx, worker_dir, 'case_%03d' % idx)
        item['case_index'] = idx
        item['score'] = score_candidate(item, baseline, args)
        item['gate_status'] = gate_status(item, baseline, args)
        results.append(item)
        records_by_case_index[idx] = (raw_path, filtered_path)

    sorted_results = sorted(results, key=lambda x: x.get('score', 1e99))

    saved = []
    if args.save_top_n > 0:
        ensure_dir(os.path.join(args.candidate_output_dir, 'dummy'))
        for rank, item in enumerate(sorted_results[:args.save_top_n], start=1):
            src_raw_path, src_filt_path = records_by_case_index.get(item.get('case_index'), (None, None))
            if src_raw_path is None or src_filt_path is None:
                continue
            raw_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_raw_commands.jsonl' % rank)
            filt_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_x8_sw40_commands.jsonl' % rank)
            meta_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_metadata.json' % rank)
            shutil.copyfile(src_raw_path, raw_path)
            shutil.copyfile(src_filt_path, filt_path)
            meta = {
                'rank': rank,
                'case_index': item.get('case_index'),
                'case': item.get('case'),
                'score': item.get('score'),
                'gate_status': item.get('gate_status'),
                'filtered_max_abs_angle_deg': c42.get_nested(item, ['filtered', 'max_abs_angle_deg']),
                'middle_phase_filtered': c42.get_nested(item, ['constraint_filtered', 'middle_phase']),
                'constraint_filtered': item.get('constraint_filtered'),
                'raw_command_log': raw_path,
                'filtered_command_log': filt_path,
                'gazebo_replay_command': 'python tools/gazebo/run_v3_0_gazebo_replay.py --command-log %s --strict-command-log-input --rate 15 --hold-start-sec 2.0 --hold-end-sec 2.0 --diagnose-command-log' % filt_path,
            }
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=2, sort_keys=True)
            saved.append(meta)

    out = {
        'version_note': 'v3.0.43C middle_swing_y_bump_sweep. Minimum-difference experiment from provisional baseline v2. It changes only the RF-3 middle-pair Y apex when explicitly enabled; RF-4 landing remains at the baseline-v2 Y coordinate for rf3_only. escape=0/mode=none is the baseline reproduction check.',
        'roadmap_position': {
            'name': 'v3.0.43C middle_swing_y_bump_sweep',
            'purpose': 'Check whether a larger but reversible RF-3-only outward Y bump makes the middle swing visibly open while returning the landing target to provisional baseline v2.',
            'not_a_refactor': True,
            'gazebo_required': True,
        },
        'baseline_case': baseline_case,
        'baseline': baseline,
        'sweep_parameters': {
            'case_count': len(cases),
            'middle_swing_y_escapes': args.middle_swing_y_escapes,
            'middle_swing_y_escape_modes': args.middle_swing_y_escape_modes,
            'middle_swing_y_escape_phases': args.middle_swing_y_escape_phases,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'segment_key': args.segment_key,
            'constraint_stride': args.constraint_stride,
            'score_focus': 'screening only: middle RF-3/RF-4 inter-leg and joint-housing clearance plus basic non-regression. Gazebo visual inspection and jump check remain mandatory.',
        },
        'saved_candidates': saved,
        'top_cases': sorted_results[:min(20, len(sorted_results))],
        'all_cases': results,
    }

    if args.output:
        ensure_dir(args.output)
        with open(args.output, 'w') as f:
            json.dump(out, f, indent=2, sort_keys=True)

    sys.stdout = json_stdout
    print(json.dumps(out, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
