#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

"""v3.0.42C candidate_03 local refine.

Roadmap position:
  Phase 4: candidate_03-centered local refinement after Gazebo visual feedback.

This script intentionally does *not* select a candidate only by the smallest
second-joint angle.  It starts from v3.0.42B candidate_03, because Gazebo
visual inspection showed candidate_03 has the most natural compact flip.
It searches only a small neighborhood and scores candidates with four gates:

  1. numeric constraint gate
     - second-joint angle
     - ground penetration
     - inter-leg capsule collision/near
     - joint-housing-vs-link collision/near
  2. command-sequence gate
     - adjacent joint-command jump
     - second difference of joint commands, as an acceleration-like proxy
     - phase/roll boundary jump
  3. compact-flip/support-extension gate
     - penalize leg-extension while a base joint is flipping
     - penalize RF-2..RF-5 support-phase near-straight extension
  4. Gazebo handoff gate
     - save only a few top filtered command logs for strict replay and visual
       confirmation; this script does not claim Gazebo acceptance.

The current baseline is v3.0.36 RF-1 current-angle anchor pure legacy repeated
roll with resample_factor=8, smooth_window=40, and no roll_index segmented
smoothing.
"""

import argparse
import itertools
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import write_jsonl
from lily_motion_v3.command_resampler import (
    resample_command_records, moving_average_command_records,
    write_command_records, full_command_diagnostics,
    boundary_transition_diagnostics)

import run_v3_0_41_second_joint_angle_localization as loc
import run_v3_0_baseline_filtered_constraint_eval as base_eval


THIGH_LENGTH_M = 0.3
TIBIA_LENGTH_M = 0.3
COXA_LENGTH_M = 0.05
VISUAL_ANCHOR_CASE = {
    'body_z': 0.39,
    'support_dist': 0.75,
    'move_dist': 0.40,
    'goal2_dist_front': 0.40,
    'goal2_x_scale': 1.00,
    'goal2_pitch_scale': 0.90,
    'goal2_landing_z': 0.00,
    'goal3_lift_z': 0.05,
    'goal3_target_x': 0.20,
    'goal4_target_x': 0.05,
    'goal5_x_scale': 1.00,
    'goal5_pitch_scale': 1.00,
}


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


def ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


def make_case_key(case):
    order = ['body_z', 'support_dist', 'move_dist', 'goal2_dist_front',
             'goal2_x_scale', 'goal2_pitch_scale', 'goal3_target_x',
             'goal4_target_x', 'goal5_x_scale', 'goal5_pitch_scale']
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


def compact_loc(src):
    if not src:
        return None
    return {
        'frame_count': src.get('frame_count'),
        'second_joint_sample_count': src.get('second_joint_sample_count'),
        'max_abs_angle_deg': src.get('max_abs_angle_deg'),
        'max_excess_deg': src.get('max_excess_deg'),
        'violation_count': src.get('violation_count'),
        'violation_frame_count': src.get('violation_frame_count'),
        'violation_sample_rate': src.get('violation_sample_rate'),
        'violation_frame_rate': src.get('violation_frame_rate'),
        'worst': src.get('worst'),
        'phase_summary': (src.get('group_summary') or {}).get('phase_name', [])[:8],
        'roll_phase_summary': (src.get('group_summary') or {}).get('roll_phase', [])[:12],
    }


def compact_constraints(rep):
    if not rep:
        return None
    sjc = rep.get('second_joint_clearance') or {}
    fc = rep.get('foot_clearance') or {}
    cp = rep.get('clearance_by_part') or {}
    return {
        'error': rep.get('error'),
        'source_frame_count': rep.get('source_frame_count'),
        'evaluated_frame_count': rep.get('evaluated_frame_count'),
        'constraint_stride': rep.get('constraint_stride'),
        'max_second_joint_deg': rep.get('max_second_joint_deg'),
        'second_joint_violation_count': rep.get('second_joint_violation_count'),
        'ground_penetration_count': rep.get('ground_penetration_count'),
        'inter_leg_collision_count': rep.get('inter_leg_collision_count'),
        'inter_leg_near_count': rep.get('inter_leg_near_count'),
        'inter_leg_joint_housing_collision_count': rep.get('inter_leg_joint_housing_collision_count'),
        'inter_leg_joint_housing_near_count': rep.get('inter_leg_joint_housing_near_count'),
        'second_joint_min_clearance_m': sjc.get('min_clearance_m'),
        'second_joint_penetration_count': sjc.get('penetration_count'),
        'second_joint_max_penetration_depth_m': sjc.get('max_penetration_depth_m'),
        'foot_min_clearance_m': fc.get('min_clearance_m'),
        'foot_penetration_count': fc.get('penetration_count'),
        'foot_max_penetration_depth_m': fc.get('max_penetration_depth_m'),
        'clearance_by_part_summary': {
            'second_joint': cp.get('second_joint'),
            'foot': cp.get('foot'),
        },
        'worst_second_joint': rep.get('worst_second_joint'),
        'worst_ground_clearance': rep.get('worst_ground_clearance'),
        'worst_inter_leg_distance': rep.get('worst_inter_leg_distance'),
    }


def command_second_difference_diagnostics(records):
    if len(records) < 3:
        return {
            'max_abs_second_diff_rad': 0.0,
            'max_abs_second_diff_deg': 0.0,
            'worst': None,
            'phase_summary': [],
        }
    n = len(records[0]['joint_command_rad'])
    worst = {'abs_second_diff_rad': -1.0}
    phase_acc = {}
    for i in range(1, len(records) - 1):
        q_prev = records[i - 1]['joint_command_rad']
        q = records[i]['joint_command_rad']
        q_next = records[i + 1]['joint_command_rad']
        phase = str(records[i].get('phase_name', 'unknown'))
        max_for_frame = 0.0
        max_joint = 0
        signed = 0.0
        for j in range(n):
            d2 = float(q_next[j]) - 2.0 * float(q[j]) + float(q_prev[j])
            ad2 = abs(d2)
            if ad2 > max_for_frame:
                max_for_frame = ad2
                signed = d2
                max_joint = j
        if max_for_frame > worst['abs_second_diff_rad']:
            worst = {
                'record_index': i,
                'frame_index': records[i].get('frame_index', i),
                'phase_name': phase,
                'roll_index': records[i].get('roll_index'),
                'joint_state_index': max_joint,
                'second_diff_rad': signed,
                'abs_second_diff_rad': max_for_frame,
                'second_diff_deg': math.degrees(signed),
                'abs_second_diff_deg': math.degrees(max_for_frame),
            }
        acc = phase_acc.setdefault(phase, {
            'phase_name': phase,
            'sample_count': 0,
            'max_abs_second_diff_rad': 0.0,
            'record_index': None,
            'joint_state_index': None,
        })
        acc['sample_count'] += 1
        if max_for_frame > acc['max_abs_second_diff_rad']:
            acc['max_abs_second_diff_rad'] = max_for_frame
            acc['record_index'] = i
            acc['joint_state_index'] = max_joint
    phases = []
    for k in sorted(phase_acc.keys()):
        a = dict(phase_acc[k])
        a['max_abs_second_diff_deg'] = math.degrees(a['max_abs_second_diff_rad'])
        phases.append(a)
    phases = sorted(phases, key=lambda x: x.get('max_abs_second_diff_rad', 0.0), reverse=True)
    return {
        'max_abs_second_diff_rad': 0.0 if worst['abs_second_diff_rad'] < 0.0 else worst['abs_second_diff_rad'],
        'max_abs_second_diff_deg': 0.0 if worst['abs_second_diff_rad'] < 0.0 else worst['abs_second_diff_deg'],
        'worst': None if worst['abs_second_diff_rad'] < 0.0 else worst,
        'phase_summary': phases,
    }


def command_smoothness_summary(records):
    diag = full_command_diagnostics(records)
    second = command_second_difference_diagnostics(records)
    phase_boundary = boundary_transition_diagnostics(records, segment_key='phase_name', top_joints=6)
    roll_boundary = boundary_transition_diagnostics(records, segment_key='roll_index', top_joints=6)
    return {
        'frame_count': len(records),
        'max_adjacent_delta_deg': diag.get('max_adjacent_delta_deg'),
        'worst_adjacent_transition': diag.get('worst_transition'),
        'max_second_diff_deg': second.get('max_abs_second_diff_deg'),
        'worst_second_diff': second.get('worst'),
        'second_diff_phase_summary': second.get('phase_summary', [])[:8],
        'phase_boundary_max_delta_deg': None if not phase_boundary.get('worst_boundary') else phase_boundary['worst_boundary'].get('max_abs_delta_deg'),
        'phase_boundary_worst': phase_boundary.get('worst_boundary'),
        'roll_boundary_max_delta_deg': None if not roll_boundary.get('worst_boundary') else roll_boundary['worst_boundary'].get('max_abs_delta_deg'),
        'roll_boundary_worst': roll_boundary.get('worst_boundary'),
    }


def _angle_delta_rad(a, b):
    d = float(b) - float(a)
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def _two_link_extension_ratio(q1, q2):
    """Return 1.0 when thigh/tibia are nearly straight."""
    L1 = THIGH_LENGTH_M
    L2 = TIBIA_LENGTH_M
    dist = math.sqrt(max(0.0, L1 * L1 + L2 * L2 + 2.0 * L1 * L2 * math.cos(float(q2))))
    return dist / (L1 + L2)


def _mount_to_foot_extension_ratio(q0, q1, q2):
    """Return mount-to-foot length normalized by coxa+thigh+tibia."""
    radial = COXA_LENGTH_M + THIGH_LENGTH_M * math.cos(float(q1)) + TIBIA_LENGTH_M * math.cos(float(q1) + float(q2))
    z = THIGH_LENGTH_M * math.sin(float(q1)) + TIBIA_LENGTH_M * math.sin(float(q1) + float(q2))
    dist = math.sqrt(radial * radial + z * z)
    return dist / (COXA_LENGTH_M + THIGH_LENGTH_M + TIBIA_LENGTH_M)


def compact_flip_extension_summary(records):
    """Detect support-leg stretch during visually sensitive flip phases.

    This is only a proxy.  It cannot replace Gazebo visual inspection.  It is
    included to avoid selecting candidates like the observed candidate_02 case,
    where a support leg stretched near a singular/roll-boundary flip.
    """
    watch_tokens = ['RF-2', 'RF-3', 'RF-4', 'RF-5', 'RF-6', 'RF-1']
    support_tokens = ['RF-2', 'RF-3', 'RF-4', 'RF-5']
    max_link = {'value': 0.0, 'worst': None}
    max_mount = {'value': 0.0, 'worst': None}
    max_support_link = {'value': 0.0, 'worst': None}
    max_flip_extension = {'value': 0.0, 'worst': None}
    max_boundary_flip_extension = {'value': 0.0, 'worst': None}

    for i, rec in enumerate(records):
        q = rec.get('joint_command_rad') or []
        phase = str(rec.get('phase_name', ''))
        is_support_phase = any(t in phase for t in support_tokens)
        for leg in range(8):
            j = leg * 3
            if j + 2 >= len(q):
                continue
            link_ratio = _two_link_extension_ratio(q[j + 1], q[j + 2])
            mount_ratio = _mount_to_foot_extension_ratio(q[j], q[j + 1], q[j + 2])
            base = {
                'record_index': i,
                'frame_index': rec.get('frame_index', i),
                'roll_index': rec.get('roll_index'),
                'phase_name': phase,
                'leg_index_in_joint_state_order': leg,
                'joint_base_index': j,
                'q0_deg': math.degrees(float(q[j])),
                'q1_deg': math.degrees(float(q[j + 1])),
                'q2_deg': math.degrees(float(q[j + 2])),
                'two_link_extension_ratio': link_ratio,
                'mount_to_foot_extension_ratio': mount_ratio,
            }
            if link_ratio > max_link['value']:
                max_link = {'value': link_ratio, 'worst': dict(base)}
            if mount_ratio > max_mount['value']:
                max_mount = {'value': mount_ratio, 'worst': dict(base)}
            if is_support_phase and link_ratio > max_support_link['value']:
                max_support_link = {'value': link_ratio, 'worst': dict(base)}

    for i in range(0, max(0, len(records) - 1)):
        rec = records[i]
        nxt = records[i + 1]
        phase_pair = '%s->%s' % (rec.get('phase_name', ''), nxt.get('phase_name', ''))
        watch = any(t in phase_pair for t in watch_tokens)
        boundary = (rec.get('phase_name') != nxt.get('phase_name')) or (rec.get('roll_index') != nxt.get('roll_index'))
        if not watch:
            continue
        q = rec.get('joint_command_rad') or []
        qn = nxt.get('joint_command_rad') or []
        for leg in range(8):
            j = leg * 3
            if j + 2 >= len(q) or j + 2 >= len(qn):
                continue
            yaw_delta_deg = abs(math.degrees(_angle_delta_rad(q[j], qn[j])))
            link_ratio = max(_two_link_extension_ratio(q[j + 1], q[j + 2]),
                             _two_link_extension_ratio(qn[j + 1], qn[j + 2]))
            mount_ratio = max(_mount_to_foot_extension_ratio(q[j], q[j + 1], q[j + 2]),
                              _mount_to_foot_extension_ratio(qn[j], qn[j + 1], qn[j + 2]))
            score = yaw_delta_deg * max(link_ratio, mount_ratio)
            detail = {
                'transition_index': i,
                'from_frame_index': rec.get('frame_index', i),
                'to_frame_index': nxt.get('frame_index', i + 1),
                'from_phase_name': rec.get('phase_name'),
                'to_phase_name': nxt.get('phase_name'),
                'from_roll_index': rec.get('roll_index'),
                'to_roll_index': nxt.get('roll_index'),
                'leg_index_in_joint_state_order': leg,
                'joint_base_index': j,
                'yaw_delta_deg': yaw_delta_deg,
                'two_link_extension_ratio': link_ratio,
                'mount_to_foot_extension_ratio': mount_ratio,
                'flip_extension_score': score,
                'is_phase_or_roll_boundary': boundary,
            }
            if score > max_flip_extension['value']:
                max_flip_extension = {'value': score, 'worst': dict(detail)}
            if boundary and score > max_boundary_flip_extension['value']:
                max_boundary_flip_extension = {'value': score, 'worst': dict(detail)}

    return {
        'note': 'Proxy only: low values are preferred, but final acceptance still requires Gazebo visual inspection. This metric was added after candidate_02 showed support-leg stretch despite acceptable numeric scores.',
        'max_two_link_extension_ratio': max_link['value'],
        'worst_two_link_extension': max_link['worst'],
        'max_mount_to_foot_extension_ratio': max_mount['value'],
        'worst_mount_to_foot_extension': max_mount['worst'],
        'max_support_phase_two_link_extension_ratio': max_support_link['value'],
        'worst_support_phase_two_link_extension': max_support_link['worst'],
        'max_flip_extension_score': max_flip_extension['value'],
        'worst_flip_extension': max_flip_extension['worst'],
        'max_boundary_flip_extension_score': max_boundary_flip_extension['value'],
        'worst_boundary_flip_extension': max_boundary_flip_extension['worst'],
    }


def visual_anchor_distance(case):
    weights = {
        'body_z': 50.0,
        'support_dist': 30.0,
        'move_dist': 30.0,
        'goal2_x_scale': 8.0,
        'goal2_pitch_scale': 8.0,
        'goal4_target_x': 12.0,
    }
    d = 0.0
    parts = {}
    for k, w in weights.items():
        if k not in case or k not in VISUAL_ANCHOR_CASE:
            continue
        v = float(case[k])
        a = float(VISUAL_ANCHOR_CASE[k])
        c = abs(v - a) * w
        parts[k] = c
        d += c
    return {'distance': d, 'parts': parts, 'anchor_case': dict(VISUAL_ANCHOR_CASE)}


def ratio(value, baseline, default=1.0):
    try:
        if baseline is None or abs(float(baseline)) < 1e-12:
            return default
        return float(value) / float(baseline)
    except Exception:
        return default


def get_nested(d, keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


def score_candidate(item, baseline, args):
    """Lower is better.  Heavy penalties reject candidates likely to break in replay."""
    if not item.get('candidate_completed'):
        return 1e99
    filt = item.get('filtered') or {}
    con = item.get('constraint_filtered') or {}
    sm = item.get('smoothness_filtered') or {}
    cf = item.get('compact_flip_filtered') or {}
    cf = item.get('compact_flip_filtered') or {}
    anchor = item.get('visual_anchor_distance') or {}
    base_f = baseline.get('filtered') or {}
    base_con = baseline.get('constraint_filtered') or {}
    base_sm = baseline.get('smoothness_filtered') or {}
    base_cf = baseline.get('compact_flip_filtered') or {}

    angle = filt.get('max_abs_angle_deg', 999.0)
    vf_rate = filt.get('violation_frame_rate', 1.0)
    vs_rate = filt.get('violation_sample_rate', 1.0)

    ground_pen = con.get('ground_penetration_count') or 0
    foot_pen = con.get('foot_penetration_count') or 0
    inter_col = con.get('inter_leg_collision_count') or 0
    inter_near = con.get('inter_leg_near_count') or 0
    house_col = con.get('inter_leg_joint_housing_collision_count') or 0
    house_near = con.get('inter_leg_joint_housing_near_count') or 0

    adj_ratio = ratio(sm.get('max_adjacent_delta_deg'), base_sm.get('max_adjacent_delta_deg'))
    d2_ratio = ratio(sm.get('max_second_diff_deg'), base_sm.get('max_second_diff_deg'))
    phase_b_ratio = ratio(sm.get('phase_boundary_max_delta_deg'), base_sm.get('phase_boundary_max_delta_deg'))
    roll_b_ratio = ratio(sm.get('roll_boundary_max_delta_deg'), base_sm.get('roll_boundary_max_delta_deg'))
    flip_ratio = ratio(cf.get('max_flip_extension_score'), base_cf.get('max_flip_extension_score'))
    boundary_flip_ratio = ratio(cf.get('max_boundary_flip_extension_score'), base_cf.get('max_boundary_flip_extension_score'))
    support_ext = cf.get('max_support_phase_two_link_extension_ratio') or 0.0
    anchor_distance = anchor.get('distance') or 0.0

    base_angle = base_f.get('max_abs_angle_deg') or angle
    improvement = max(0.0, float(base_angle) - float(angle))

    # Base score: angle and violation rate dominate but do not overrule hard regressions.
    score = float(angle) * 1000000.0
    score += float(vf_rate) * 3000000.0
    score += float(vs_rate) * 1000000.0

    # Reward genuine angle improvement, but not enough to hide physical/smoothness regressions.
    score -= improvement * 200000.0

    # Physical penalties.
    score += ground_pen * 200000.0
    score += foot_pen * 50000.0
    score += inter_col * 20000000.0
    score += inter_near * 2000000.0
    score += house_col * 20000000.0
    score += house_near * 2000000.0

    # Smoothness penalties versus baseline.  v3.0.42C keeps these softer
    # than v3.0.42B because candidate_03 had a boundary metric warning but
    # better visual behavior in Gazebo.
    for r, limit, weight in [
            (adj_ratio, args.max_adjacent_delta_ratio, 2500000.0),
            (d2_ratio, args.max_second_diff_ratio, 5000000.0),
            (phase_b_ratio, args.max_boundary_delta_ratio, 2000000.0),
            (roll_b_ratio, args.max_boundary_delta_ratio, 2000000.0),
            (flip_ratio, args.max_flip_extension_ratio, 5000000.0),
            (boundary_flip_ratio, args.max_boundary_flip_extension_ratio, 4000000.0)]:
        if r > limit:
            score += (r - limit) * weight

    # Visual-anchor penalty: prefer the candidate_03 neighborhood unless a
    # candidate gives a clearly better angle without stretch/flip penalties.
    score += anchor_distance * 1200000.0
    if support_ext > args.max_support_extension_ratio:
        score += (support_ext - args.max_support_extension_ratio) * 12000000.0

    return score


def gate_status(item, baseline, args):
    filt = item.get('filtered') or {}
    con = item.get('constraint_filtered') or {}
    sm = item.get('smoothness_filtered') or {}
    cf = item.get('compact_flip_filtered') or {}
    anchor = item.get('visual_anchor_distance') or {}
    base_f = baseline.get('filtered') or {}
    base_con = baseline.get('constraint_filtered') or {}
    base_sm = baseline.get('smoothness_filtered') or {}
    base_cf = baseline.get('compact_flip_filtered') or {}

    checks = []
    def add(name, ok, value=None, limit=None):
        checks.append({'name': name, 'ok': bool(ok), 'value': value, 'limit': limit})

    add('candidate_completed', item.get('candidate_completed'), item.get('candidate_completed'), True)
    add('angle_improves_vs_baseline', filt.get('max_abs_angle_deg', 999.0) < base_f.get('max_abs_angle_deg', -999.0), filt.get('max_abs_angle_deg'), base_f.get('max_abs_angle_deg'))
    add('angle_below_limit', filt.get('max_abs_angle_deg', 999.0) <= args.second_joint_abs_max_deg, filt.get('max_abs_angle_deg'), args.second_joint_abs_max_deg)
    add('no_inter_leg_collision', (con.get('inter_leg_collision_count') or 0) == 0, con.get('inter_leg_collision_count'), 0)
    add('no_joint_housing_collision', (con.get('inter_leg_joint_housing_collision_count') or 0) == 0, con.get('inter_leg_joint_housing_collision_count'), 0)
    add('no_inter_leg_near', (con.get('inter_leg_near_count') or 0) == 0, con.get('inter_leg_near_count'), 0)
    add('no_joint_housing_near', (con.get('inter_leg_joint_housing_near_count') or 0) == 0, con.get('inter_leg_joint_housing_near_count'), 0)

    base_ground = base_con.get('ground_penetration_count') or 0
    base_foot = base_con.get('foot_penetration_count') or 0
    add('ground_penetration_not_worse_than_baseline', (con.get('ground_penetration_count') or 0) <= base_ground, con.get('ground_penetration_count'), base_ground)
    add('foot_penetration_not_worse_than_baseline', (con.get('foot_penetration_count') or 0) <= base_foot, con.get('foot_penetration_count'), base_foot)

    add('adjacent_delta_ratio_ok', ratio(sm.get('max_adjacent_delta_deg'), base_sm.get('max_adjacent_delta_deg')) <= args.max_adjacent_delta_ratio, ratio(sm.get('max_adjacent_delta_deg'), base_sm.get('max_adjacent_delta_deg')), args.max_adjacent_delta_ratio)
    add('second_diff_ratio_ok', ratio(sm.get('max_second_diff_deg'), base_sm.get('max_second_diff_deg')) <= args.max_second_diff_ratio, ratio(sm.get('max_second_diff_deg'), base_sm.get('max_second_diff_deg')), args.max_second_diff_ratio)
    add('boundary_delta_ratio_ok', max(ratio(sm.get('phase_boundary_max_delta_deg'), base_sm.get('phase_boundary_max_delta_deg')), ratio(sm.get('roll_boundary_max_delta_deg'), base_sm.get('roll_boundary_max_delta_deg'))) <= args.max_boundary_delta_ratio, max(ratio(sm.get('phase_boundary_max_delta_deg'), base_sm.get('phase_boundary_max_delta_deg')), ratio(sm.get('roll_boundary_max_delta_deg'), base_sm.get('roll_boundary_max_delta_deg'))), args.max_boundary_delta_ratio)
    add('support_extension_ratio_ok', (cf.get('max_support_phase_two_link_extension_ratio') or 0.0) <= args.max_support_extension_ratio, cf.get('max_support_phase_two_link_extension_ratio'), args.max_support_extension_ratio)
    add('flip_extension_ratio_ok', ratio(cf.get('max_flip_extension_score'), base_cf.get('max_flip_extension_score')) <= args.max_flip_extension_ratio, ratio(cf.get('max_flip_extension_score'), base_cf.get('max_flip_extension_score')), args.max_flip_extension_ratio)
    add('boundary_flip_extension_ratio_ok', ratio(cf.get('max_boundary_flip_extension_score'), base_cf.get('max_boundary_flip_extension_score')) <= args.max_boundary_flip_extension_ratio, ratio(cf.get('max_boundary_flip_extension_score'), base_cf.get('max_boundary_flip_extension_score')), args.max_boundary_flip_extension_ratio)

    hard = [c for c in checks if c['name'] in (
        'candidate_completed', 'angle_improves_vs_baseline',
        'no_inter_leg_collision', 'no_joint_housing_collision',
        'second_diff_ratio_ok', 'support_extension_ratio_ok',
        'flip_extension_ratio_ok')]
    return {
        'checks': checks,
        'hard_gate_ok': all(c['ok'] for c in hard),
        'angle_limit_ok': [c for c in checks if c['name'] == 'angle_below_limit'][0]['ok'],
        'gazebo_replay_required': True,
        'note': 'hard_gate_ok is not final acceptance. v3.0.42C intentionally does not hard-reject boundary_delta_ratio because candidate_03 looked better in Gazebo despite a boundary warning. Strict replay and visual inspection are mandatory.',
    }


def evaluate_case(args, case, label, do_constraints=False):
    loc_args = make_loc_args(args, case)
    item = {'case': dict(case), 'case_key': make_case_key(case), 'label': label}
    raw_records = None
    filtered_records = None
    try:
        raw_records, completed, error = loc._make_legacy_repeated_records(loc_args)
        item['candidate_completed'] = completed
        item['generation_error'] = error
        raw_loc = loc._analyze_source(raw_records, 'raw', args.second_joint_abs_max_deg, args.boundary_window, args.top_n)
        item['raw'] = compact_loc(raw_loc)
        item['smoothness_raw'] = command_smoothness_summary(raw_records)
        item['compact_flip_raw'] = compact_flip_extension_summary(raw_records)

        segment_key = args.segment_key.strip() or None
        filtered_records = resample_command_records(raw_records, factor=args.resample_factor, segment_key=segment_key)
        filtered_records = moving_average_command_records(filtered_records, window=args.smooth_window, segment_key=segment_key)
        filtered_loc = loc._analyze_source(filtered_records, 'filtered', args.second_joint_abs_max_deg, args.boundary_window, args.top_n)
        item['filtered'] = compact_loc(filtered_loc)
        item['smoothness_filtered'] = command_smoothness_summary(filtered_records)
        item['compact_flip_filtered'] = compact_flip_extension_summary(filtered_records)
        item['visual_anchor_distance'] = visual_anchor_distance(case)

        if do_constraints:
            cargs = make_constraint_args(args, loc_args)
            if args.evaluate_raw_constraints:
                item['constraint_raw'] = compact_constraints(base_eval._evaluate_constraints(raw_records, cargs, 'raw'))
            item['constraint_filtered'] = compact_constraints(base_eval._evaluate_constraints(filtered_records, cargs, 'filtered_gazebo_command'))
        else:
            item['constraint_filtered'] = {'not_evaluated': True, 'reason': 'two-stage sweep: constraints are evaluated only for baseline and selected top candidates'}
    except Exception as e:
        item['candidate_completed'] = False
        item['generation_error'] = {'type': e.__class__.__name__, 'message': str(e)}
    return item, raw_records, filtered_records


def main():
    ap = argparse.ArgumentParser(description='v3.0.42C candidate_03 local refine: angle + constraints + command smoothness. Candidate generation only; Gazebo replay required before acceptance.')
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
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)

    # Small robust geometry sweep defaults.  Keep them conservative; this is
    # not a wide optimizer.
    ap.add_argument('--body-zs', default='0.39,0.40,0.41')
    ap.add_argument('--support-dists', default='0.73,0.75,0.77')
    ap.add_argument('--move-dists', default='0.38,0.40')
    ap.add_argument('--goal2-dist-fronts', default='0.40')
    ap.add_argument('--goal2-x-scales', default='0.95,1.0,1.05')
    ap.add_argument('--goal2-pitch-scales', default='0.85,0.90,0.95')
    ap.add_argument('--goal2-landing-zs', default='0.0')
    ap.add_argument('--goal3-lift-zs', default='0.05')
    ap.add_argument('--goal3-target-xs', default='0.2')
    ap.add_argument('--goal4-target-xs', default='0.04,0.05,0.06')
    ap.add_argument('--goal5-x-scales', default='1.0')
    ap.add_argument('--goal5-pitch-scales', default='1.0')

    ap.add_argument('--resample-factor', type=int, default=8)
    ap.add_argument('--smooth-window', type=int, default=40)
    ap.add_argument('--segment-key', default='', help='Leave empty for current baseline: do not split smoothing by roll_index.')
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
    ap.add_argument('--constraint-stride', type=int, default=128, help='Evaluate every Nth filtered frame for speed; final frame is always included. Use 4 or 8 only for final selected candidates.')
    ap.add_argument('--evaluate-raw-constraints', action='store_true')
    ap.add_argument('--constraint-top-n', type=int, default=3, help='Two-stage mode: run heavier FK constraints only for baseline and the top N preliminary candidates.')
    ap.add_argument('--no-constraints', action='store_true', help='Skip heavy FK constraint evaluation completely. Use only for quick smoke tests.')

    ap.add_argument('--max-adjacent-delta-ratio', type=float, default=1.10)
    ap.add_argument('--max-second-diff-ratio', type=float, default=1.20)
    ap.add_argument('--max-boundary-delta-ratio', type=float, default=1.25)
    ap.add_argument('--max-support-extension-ratio', type=float, default=0.99995)
    ap.add_argument('--max-flip-extension-ratio', type=float, default=1.10)
    ap.add_argument('--max-boundary-flip-extension-ratio', type=float, default=1.25)

    ap.add_argument('--output', default='testdata/v3_0_42c_candidate03_local_refine.json')
    ap.add_argument('--candidate-output-dir', default='testdata/v3_0_42c_candidates')
    ap.add_argument('--save-top-n', type=int, default=3)
    args = ap.parse_args()

    json_stdout = sys.stdout
    sys.stdout = sys.stderr

    lists = {
        'body_z': parse_float_list(args.body_zs),
        'support_dist': parse_float_list(args.support_dists),
        'move_dist': parse_float_list(args.move_dists),
        'goal2_dist_front': parse_float_list(args.goal2_dist_fronts),
        'goal2_x_scale': parse_float_list(args.goal2_x_scales),
        'goal2_pitch_scale': parse_float_list(args.goal2_pitch_scales),
        'goal2_landing_z': parse_float_list(args.goal2_landing_zs),
        'goal3_lift_z': parse_float_list(args.goal3_lift_zs),
        'goal3_target_x': parse_float_list(args.goal3_target_xs),
        'goal4_target_x': parse_float_list(args.goal4_target_xs),
        'goal5_x_scale': parse_float_list(args.goal5_x_scales),
        'goal5_pitch_scale': parse_float_list(args.goal5_pitch_scales),
    }
    keys = ['body_z', 'support_dist', 'move_dist', 'goal2_dist_front',
            'goal2_x_scale', 'goal2_pitch_scale', 'goal2_landing_z',
            'goal3_lift_z', 'goal3_target_x', 'goal4_target_x',
            'goal5_x_scale', 'goal5_pitch_scale']
    cases = [dict(zip(keys, vals)) for vals in itertools.product(*[lists[k] for k in keys])]

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
    }
    print('Evaluating baseline angle/smoothness...', file=sys.stderr)
    baseline, _, _ = evaluate_case(args, baseline_case, 'baseline', do_constraints=(not args.no_constraints and args.constraint_top_n > 0))

    results = []
    records_by_case_index = {}
    for idx, case in enumerate(cases):
        print('Evaluating preliminary case %d/%d: %s' % (idx + 1, len(cases), make_case_key(case)), file=sys.stderr)
        item, raw_records, filtered_records = evaluate_case(args, case, 'case_%d' % idx, do_constraints=False)
        item['case_index'] = idx
        item['preliminary_score'] = score_candidate(item, baseline, args)
        item['score'] = item['preliminary_score']
        item['gate_status'] = gate_status(item, baseline, args)
        results.append(item)
        records_by_case_index[idx] = (raw_records, filtered_records)

    preliminary_sorted = sorted(results, key=lambda x: x.get('preliminary_score', 1e99))

    if not args.no_constraints and args.constraint_top_n > 0:
        top_ids = set([x['case_index'] for x in preliminary_sorted[:args.constraint_top_n]])
        for item in results:
            if item.get('case_index') not in top_ids:
                continue
            print('Evaluating constraints for selected case %s: %s' % (item.get('case_index'), item.get('case_key')), file=sys.stderr)
            case = item['case']
            full_item, raw_records, filtered_records = evaluate_case(args, case, item.get('label', 'selected'), do_constraints=True)
            full_item['case_index'] = item['case_index']
            full_item['preliminary_score'] = item.get('preliminary_score')
            full_item['score'] = score_candidate(full_item, baseline, args)
            full_item['gate_status'] = gate_status(full_item, baseline, args)
            results[item['case_index']] = full_item
            records_by_case_index[item['case_index']] = (raw_records, filtered_records)

    sorted_results = sorted(results, key=lambda x: x.get('score', 1e99))
    saved = []
    if args.save_top_n > 0:
        ensure_dir(os.path.join(args.candidate_output_dir, 'dummy'))
        for rank, item in enumerate(sorted_results[:args.save_top_n], start=1):
            raw_records, filtered_records = records_by_case_index.get(item['case_index'], (None, None))
            if raw_records is None or filtered_records is None:
                continue
            raw_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_raw_commands.jsonl' % rank)
            filt_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_x8_sw40_commands.jsonl' % rank)
            meta_path = os.path.join(args.candidate_output_dir, 'candidate_%02d_metadata.json' % rank)
            write_jsonl(raw_records, raw_path)
            write_command_records(filtered_records, filt_path)
            meta = {
                'rank': rank,
                'case_index': item.get('case_index'),
                'case': item.get('case'),
                'score': item.get('score'),
                'filtered_max_abs_angle_deg': get_nested(item, ['filtered', 'max_abs_angle_deg']),
                'filtered_violation_frame_rate': get_nested(item, ['filtered', 'violation_frame_rate']),
                'gate_status': item.get('gate_status'),
                'compact_flip_filtered': item.get('compact_flip_filtered'),
                'visual_anchor_distance': item.get('visual_anchor_distance'),
                'raw_command_log': raw_path,
                'filtered_command_log': filt_path,
                'gazebo_replay_command': 'python run_v3_0_gazebo_replay.py --command-log %s --strict-command-log-input --rate 15 --hold-start-sec 2.0 --hold-end-sec 2.0 --diagnose-command-log' % filt_path,
            }
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=2, sort_keys=True)
            saved.append(meta)

    out = {
        'version_note': 'v3.0.42C candidate_03 local refine. Candidate selection uses angle + physical constraints + command smoothness. It does not prove Gazebo acceptance; saved filtered logs require strict replay and visual inspection.',
        'roadmap_position': {
            'phase': 'Phase 4',
            'name': 'v3.0.42C candidate_03 local refine',
            'purpose': 'Refine the visually preferred candidate_03 neighborhood while avoiding support-leg stretch during compact first-joint flip.',
            'next_phase': 'Phase 5/6: full constraint reevaluation and Gazebo strict replay of saved top candidates, starting from the most natural visual candidate rather than the angle-only winner.',
        },
        'baseline_case': baseline_case,
        'baseline': baseline,
        'sweep_parameters': {
            'case_count': len(cases),
            'body_zs': args.body_zs,
            'support_dists': args.support_dists,
            'move_dists': args.move_dists,
            'goal2_dist_fronts': args.goal2_dist_fronts,
            'goal2_x_scales': args.goal2_x_scales,
            'goal2_pitch_scales': args.goal2_pitch_scales,
            'constraint_stride': args.constraint_stride,
            'constraint_top_n': args.constraint_top_n,
            'no_constraints': args.no_constraints,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'segment_key': args.segment_key,
            'visual_anchor_case': VISUAL_ANCHOR_CASE,
            'max_support_extension_ratio': args.max_support_extension_ratio,
            'max_flip_extension_ratio': args.max_flip_extension_ratio,
            'max_boundary_flip_extension_ratio': args.max_boundary_flip_extension_ratio,
        },
        'scoring_note': 'Lower score is better. This v3.0.42C score is biased toward the candidate_03 neighborhood because Gazebo showed candidate_03 had the most natural compact flip. The sweep is two-stage: angle/smoothness/compact-flip proxies for all cases, heavier FK constraints for baseline and selected top candidates only by default. Hard gates still do not mean acceptance; Gazebo replay is mandatory.',
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
