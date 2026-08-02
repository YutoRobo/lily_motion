# -*- coding: utf-8 -*-
"""Constraint-aware parameter generation for legacy quarter-rolls.

v3.0.29 changes the search order: ``move_dist`` is not treated as a
free variable selected by the second-joint score.  For each
``support_dist`` / ``body_z`` / pitch-profile family, this module first
finds the ``move_dist`` that minimizes the quarter-roll periodicity error.
Only candidates satisfying the periodicity thresholds are passed to the
constraint and repeated-roll evaluators.
"""
from __future__ import division, print_function
import math

from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
from lily_motion_v3.repeated_roll_connection import connection_report

FORWARD_SEQUENCE = [1, 5, 6, 2, 1]

# Legacy pitch increment model in the vendored emulator:
#   RF-1: pi/4 * 0.1
#   RF-2: pi/4 * 0.9 * goal2_pitch_scale
#   RF-5: pi/4 * goal5_pitch_scale
# The helper below chooses goal5_pitch_scale so the one-quarter-roll total is
# pi/2 whenever possible.
def _goal5_for_goal2(goal2_scale):
    return (0.5 - 0.025 - 0.225 * float(goal2_scale)) / 0.25

PITCH_PROFILES = {
    'legacy': {
        'goal2_pitch_scale': 1.0,
        'goal5_pitch_scale': 1.0,
        'description': 'Original legacy pitch split.'
    },
    'balanced': {
        'goal2_pitch_scale': 0.90,
        'goal5_pitch_scale': _goal5_for_goal2(0.90),
        'description': 'Slightly reduced RF-2 pitch, compensated in RF-5 to preserve total quarter-roll pitch.'
    },
    'late_roll': {
        'goal2_pitch_scale': 0.80,
        'goal5_pitch_scale': _goal5_for_goal2(0.80),
        'description': 'More pitch delayed from RF-2 to RF-5; may improve RF-2 second-joint margin but must pass periodicity.'
    },
}


def parse_float_list(s):
    return [float(x) for x in str(s).split(',') if str(x).strip()]


def parse_profile_list(s):
    out = []
    for name in str(s).split(','):
        name = name.strip()
        if not name:
            continue
        if name not in PITCH_PROFILES:
            raise ValueError('unknown pitch profile: %s (known: %s)' % (name, ','.join(sorted(PITCH_PROFILES.keys()))))
        out.append(name)
    return out


def linspace(a, b, n):
    n = int(n)
    if n <= 1:
        return [float(a)]
    a = float(a); b = float(b)
    return [a + (b-a) * i / float(n-1) for i in range(n)]


def make_config(surface_id, support_dist, body_z, move_dist, max_step, profile_name,
                goal2_dist_front=0.3, goal2_x_scale=1.0, goal2_landing_z=0.0,
                goal3_lift_z=0.05, goal3_target_x=0.2, goal4_target_x=0.05,
                initialize_step=100):
    prof = PITCH_PROFILES[profile_name]
    return LegacyStateMachineConfig(
        move_dist=move_dist,
        support_dist=support_dist,
        max_step=max_step,
        surface_id=surface_id,
        z=body_z,
        initialize_step=initialize_step,
        include_initialize=False,
        goal2_dist_front=goal2_dist_front,
        goal2_x_scale=goal2_x_scale,
        goal2_pitch_scale=prof['goal2_pitch_scale'],
        goal2_landing_z=goal2_landing_z,
        goal3_lift_z=goal3_lift_z,
        goal3_target_x=goal3_target_x,
        goal4_target_x=goal4_target_x,
        goal5_x_scale=1.0,
        goal5_pitch_scale=prof['goal5_pitch_scale'],
    )


def generate_single_roll_records(surface_id, support_dist, body_z, move_dist, max_step, profile_name, **kwargs):
    cfg = make_config(surface_id, support_dist, body_z, move_dist, max_step, profile_name, **kwargs)
    emu = LegacyStateMachineEmulator(cfg)
    next_surface = {1:5, 5:6, 6:2, 2:1}[int(surface_id)]
    return emu.run_forward_repeated(surface_sequence=[surface_id, next_surface], include_initialize=False)


def periodicity_for_records(records, body_z):
    conn = connection_report(records, default_body_z=body_z)
    max_err = conn.get('max_body_to_next_support_center_xy_m')
    if max_err is None:
        max_err = 1e9
    # v3.0.29 uses the centroid connection metric as a first-order periodicity
    # gate.  The report keeps the note explicit because this is not a full
    # support-polygon proof.
    mean_err = 0.0
    boundaries = conn.get('boundaries') or []
    if boundaries:
        mean_err = sum(float(b.get('body_to_next_support_center_xy_m', 0.0)) for b in boundaries) / float(len(boundaries))
    else:
        mean_err = max_err
    return {
        'max_error_m': float(max_err),
        'mean_error_m': float(mean_err),
        'connection_report': conn,
        'metric_note': 'Uses repeated_roll_connection centroid metric as an admissibility gate; move_dist is chosen before second-joint scoring.'
    }


def find_periodic_move_dist(surface_id, support_dist, body_z, max_step, profile_name,
                            move_dist_candidates, max_error_limit, mean_error_limit, **kwargs):
    best = None
    for d in move_dist_candidates:
        try:
            records = generate_single_roll_records(surface_id, support_dist, body_z, d, max_step, profile_name, **kwargs)
            per = periodicity_for_records(records, body_z)
            err = per['max_error_m'] + per['mean_error_m']
            item = {'move_dist': float(d), 'periodicity': per, 'records': records, 'score': float(err)}
        except Exception as e:
            item = {'move_dist': float(d), 'periodicity': {'max_error_m': 1e9, 'mean_error_m': 1e9}, 'records': [], 'score': 1e9, 'error': str(e)}
        if best is None or item['score'] < best['score']:
            best = item
    if best is None:
        return None
    best['admissible'] = bool(best['periodicity']['max_error_m'] <= max_error_limit and best['periodicity']['mean_error_m'] <= mean_error_limit)
    return best


def evaluate_constraints(records, body_z, second_joint_limit_deg=95.0, inter_leg_limit=0.04, ground_tolerance=1e-4):
    ev = LegacyConstraintEvaluator(
        second_joint_limit_deg=second_joint_limit_deg,
        ground_tol=ground_tolerance,
        inter_leg_limit_m=inter_leg_limit,
        default_body_z=body_z,
    )
    return ev.evaluate(records)


def generate_repeated_records(surface_sequence, surface_id, support_dist, body_z, move_dist, max_step, profile_name, **kwargs):
    cfg = make_config(surface_id, support_dist, body_z, move_dist, max_step, profile_name, **kwargs)
    emu = LegacyStateMachineEmulator(cfg)
    return emu.run_forward_repeated(surface_sequence=surface_sequence, include_initialize=False)



def _is_finite_number(x):
    try:
        value = float(x)
    except Exception:
        return False
    return value == value and abs(value) != float('inf')


def is_valid_constrained_case(case):
    """Return True only for candidates that passed every required gate.

    v3.0.30: Periodicity alone is not enough.  A candidate must also
    produce a successful repeated-roll command sequence and a finite,
    error-free constraint report.  Penalty sentinel values such as 999 or
    999999 are treated as invalid rather than merely badly scored.
    """
    if not case or not case.get('admissible_by_periodicity', False):
        return False
    constraints = case.get('constraints') or {}
    repeated = case.get('repeated_roll') or {}
    if constraints.get('error'):
        return False
    if repeated.get('error'):
        return False
    if not repeated.get('candidate_completed', False):
        return False
    max_second = constraints.get('max_second_joint_deg')
    if not _is_finite_number(max_second) or float(max_second) >= 900.0:
        return False
    for key in ('second_joint_violation_count', 'ground_penetration_count', 'inter_leg_near_count'):
        value = constraints.get(key)
        try:
            if int(value) >= 999000:
                return False
        except Exception:
            return False
    return True


def score_valid_case(case):
    """Score only already-valid candidates. Lower is better."""
    periodicity = case.get('periodicity') or {}
    constraints = case.get('constraints') or {}
    repeated = case.get('repeated_roll') or {}
    # Completion has already been gated. Keep the score interpretable.
    return (float(periodicity.get('max_error_m', 1e9)) * 100000.0 +
            float(periodicity.get('mean_error_m', 1e9)) * 50000.0 +
            float(constraints.get('max_second_joint_deg', 999.0)) +
            int(constraints.get('second_joint_violation_count', 999999)) * 10000.0 +
            int(constraints.get('ground_penetration_count', 999999)) * 100000.0 +
            int(constraints.get('inter_leg_near_count', 999999)) * 10000.0)

def score_case(periodicity, constraints, repeated_report):
    # Periodicity is a gate, but score still preserves margins for ranking.
    max_second = float(constraints.get('max_second_joint_deg', 999.0))
    viol = int(constraints.get('second_joint_violation_count', 999999))
    ground = int(constraints.get('ground_penetration_count', 999999))
    near = int(constraints.get('inter_leg_near_count', 999999))
    rep_complete_penalty = 0 if repeated_report.get('candidate_completed') else 10000000
    return (periodicity['max_error_m'] * 100000.0 + periodicity['mean_error_m'] * 50000.0 +
            max_second + viol * 10000.0 + ground * 100000.0 + near * 10000.0 + rep_complete_penalty)
