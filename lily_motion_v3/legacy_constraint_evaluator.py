# -*- coding: utf-8 -*-
"""Constraint evaluation for vendored legacy roll command logs.

This module reads JSONL records with ``joint_command_rad`` and evaluates the
legacy-state-machine command trajectory with the same vendored legacy FK used by
v3.0.22.  It is intentionally diagnostic: it does not modify the gait.
"""
from __future__ import division, print_function
import math

try:
    range = xrange
except NameError:  # pragma: no cover
    pass

import numpy as np

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_runtime.util import Posture, TransformationRobotToABS

JOINT_NAMES = ['base_clause', 'thigh', 'tibia']


def _as_float3(matrix_like):
    return np.array([float(matrix_like[0]), float(matrix_like[1]), float(matrix_like[2])], dtype=float)


def _posture_from_record(rec, default_z=0.35):
    bp = rec.get('base_pose') or {}
    return Posture(
        x=float(bp.get('x', 0.0)),
        y=float(bp.get('y', 0.0)),
        z=float(bp.get('z', default_z)),
        roll=float(bp.get('roll', 0.0)),
        pitch=float(bp.get('pitch', 0.0)),
        yaw=float(bp.get('yaw', 0.0)),
    )


def _point_segment_distance(p, a, b):
    p = np.asarray(p, dtype=float); a = np.asarray(a, dtype=float); b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-16:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    q = a + t * ab
    return float(np.linalg.norm(p - q))


def _segment_distance(a0, a1, b0, b1):
    """Exact shortest distance between two 3D line segments.

    This is used for capsule-based inter-leg checks.  The previous
    endpoint-only proxy can miss closest points that occur in the interior of
    both segments, so v3.0.39 uses the standard closest-points formulation.
    """
    p1 = np.asarray(a0, dtype=float)
    q1 = np.asarray(a1, dtype=float)
    p2 = np.asarray(b0, dtype=float)
    q2 = np.asarray(b1, dtype=float)
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    eps = 1e-12

    if a <= eps and e <= eps:
        return float(np.linalg.norm(p1 - p2))
    if a <= eps:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
    else:
        c = float(np.dot(d1, r))
        if e <= eps:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            if denom != 0.0:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0
            tnom = b * s + f
            if tnom < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif tnom > e:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))
            else:
                t = tnom / e
    cp1 = p1 + d1 * s
    cp2 = p2 + d2 * t
    return float(np.linalg.norm(cp1 - cp2))


def _legacy_first_joint_pose_robot(leg_obj, theta):
    """Return the first/upper-link root point in robot coordinates.

    The vendored Leg exposes calcSecondJointPose() but not the coxa/upper-link
    root.  For diagnostics we obtain it by temporarily zeroing L2 and reusing
    the same FK expression.  The leg state is restored immediately.
    """
    try:
        L = getattr(leg_obj, '_Leg__L')
        old = L[2]
        L[2] = 0.0
        try:
            return leg_obj.calcSecondJointPose(theta)
        finally:
            L[2] = old
    except Exception:
        # Conservative fallback: use the robot/body origin.  This should not be
        # needed for the vendored legacy Leg, but keeps diagnostics robust.
        return np.asarray([[0.0], [0.0], [0.0]])


def _new_clearance_acc(point_name):
    return {
        'point_name': point_name,
        'min_clearance_m': None,
        'penetration_count': 0,
        'max_penetration_depth_m': 0.0,
        'worst': None,
    }


def _update_clearance_acc(acc, clearance, item_base, ground_tol):
    if acc['min_clearance_m'] is None or clearance < acc['min_clearance_m']:
        acc['min_clearance_m'] = clearance
        acc['worst'] = dict(item_base)
        acc['worst']['clearance_m'] = clearance
        acc['worst']['penetration_m'] = max(0.0, -clearance)
    if clearance < -ground_tol:
        acc['penetration_count'] += 1
        depth = -clearance
        if depth > acc['max_penetration_depth_m']:
            acc['max_penetration_depth_m'] = depth


class LegacyConstraintEvaluator(object):
    def __init__(self, second_joint_limit_deg=95.0, ground_z=0.0, ground_tol=0.0,
                 inter_leg_limit_m=0.04, default_body_z=0.35,
                 leg_radius_m=0.015, inter_leg_safety_margin_m=0.010,
                 joint_housing_radius_m=0.030, joint_housing_safety_margin_m=None):
        self.second_joint_limit_rad = math.radians(float(second_joint_limit_deg))
        self.second_joint_limit_deg = float(second_joint_limit_deg)
        self.ground_z = float(ground_z)
        self.ground_tol = float(ground_tol)
        self.inter_leg_limit_m = float(inter_leg_limit_m)
        self.leg_radius_m = float(leg_radius_m)
        self.inter_leg_safety_margin_m = float(inter_leg_safety_margin_m)
        self.joint_housing_radius_m = float(joint_housing_radius_m)
        if joint_housing_safety_margin_m is None:
            joint_housing_safety_margin_m = inter_leg_safety_margin_m
        self.joint_housing_safety_margin_m = float(joint_housing_safety_margin_m)
        self.inter_leg_collision_threshold_m = 2.0 * self.leg_radius_m
        self.inter_leg_required_clearance_m = max(
            self.inter_leg_limit_m,
            self.inter_leg_collision_threshold_m + self.inter_leg_safety_margin_m,
        )
        self.joint_housing_collision_threshold_m = self.joint_housing_radius_m + self.leg_radius_m
        self.joint_housing_required_clearance_m = self.joint_housing_collision_threshold_m + self.joint_housing_safety_margin_m
        self.default_body_z = float(default_body_z)
        cfg = LegacyStateMachineConfig(z=default_body_z)
        self._emu = LegacyStateMachineEmulator(cfg)

    def _apply_command_to_legacy_legs(self, q_rad):
        # Set the vendored ServoMotor states in external/Gazebo joint-state order.
        for idx, (legacy_leg_id, joint_index) in enumerate(JOINT_STATE_ORDER):
            leg_obj = self._emu.legs_by_legacy_id[legacy_leg_id]
            degs = leg_obj.getServosDeg()
            degs[joint_index][0] = math.degrees(float(q_rad[idx]))
            # Use Leg.setTargetDegree to keep ServoMotor state semantics.
            leg_obj.setTargetDegree([[float(degs[0][0])], [float(degs[1][0])], [float(degs[2][0])]])

    def _frame_geometry(self, rec):
        q_rad = rec['joint_command_rad']
        self._apply_command_to_legacy_legs(q_rad)
        posture = _posture_from_record(rec, default_z=self.default_body_z)
        out = {}
        for legacy_leg_id in range(8):
            leg_obj = self._emu.legs_by_legacy_id[legacy_leg_id]
            theta = np.array([[leg_obj.servos[0].getTargetTheta()], [leg_obj.servos[1].getTargetTheta()], [leg_obj.servos[2].getTargetTheta()]])
            foot_robot = leg_obj.calcPose(theta)
            knee_robot = leg_obj.calcSecondJointPose(theta)
            hip_robot = _legacy_first_joint_pose_robot(leg_obj, theta)
            foot_abs = _as_float3(TransformationRobotToABS(foot_robot, posture))
            knee_abs = _as_float3(TransformationRobotToABS(knee_robot, posture))
            hip_abs = _as_float3(TransformationRobotToABS(hip_robot, posture))
            out[legacy_leg_id] = {
                'leg_id': legacy_leg_id,
                'leg_name': LEG_NAMES_BY_ID[legacy_leg_id],
                'hip_abs': hip_abs,
                'knee_abs': knee_abs,
                'foot_abs': foot_abs,
                'upper_segment': (hip_abs, knee_abs),
                'lower_segment': (knee_abs, foot_abs),
                'tibia_segment': (knee_abs, foot_abs),  # backward-compatible alias
            }
        return out

    def evaluate(self, records, top_n=20):
        second_violations = []
        ground_violations = []
        near_violations = []
        collision_violations = []
        joint_housing_near_violations = []
        joint_housing_collision_violations = []
        worst_joint_housing = None
        worst_second = None
        worst_ground = None
        worst_near = None
        phase_acc = {}
        clearance_by_part = {
            'second_joint': _new_clearance_acc('second_joint'),
            'foot': _new_clearance_acc('foot'),
        }
        max_second_abs_rad = 0.0

        for fi, rec in enumerate(records):
            q = rec['joint_command_rad']
            phase = str(rec.get('phase_name', 'unknown'))
            phase_acc.setdefault(phase, {
                'phase_name': phase,
                'frame_count': 0,
                'second_joint_violation_count': 0,
                'ground_penetration_count': 0,
                'inter_leg_near_count': 0,
                'inter_leg_collision_count': 0,
                'joint_housing_near_count': 0,
                'joint_housing_collision_count': 0,
                'max_second_joint_deg': 0.0,
                'min_clearance_m': None,
                'min_inter_leg_distance_m': None,
                'min_joint_housing_distance_m': None,
                'clearance_by_part': {
                    'second_joint': _new_clearance_acc('second_joint'),
                    'foot': _new_clearance_acc('foot'),
                },
            })
            ph = phase_acc[phase]
            ph['frame_count'] += 1

            # second joint angle = joint_index 1 in each leg.
            for idx, (legacy_leg_id, joint_index) in enumerate(JOINT_STATE_ORDER):
                if joint_index != 1:
                    continue
                val = abs(float(q[idx]))
                if val > max_second_abs_rad:
                    max_second_abs_rad = val
                    worst_second = {
                        'frame_index': rec.get('frame_index', fi),
                        'phase_name': phase,
                        'leg_id': legacy_leg_id,
                        'leg_name': LEG_NAMES_BY_ID[legacy_leg_id],
                        'joint_index': joint_index,
                        'joint_name': 'thigh',
                        'abs_angle_rad': val,
                        'abs_angle_deg': math.degrees(val),
                    }
                ph['max_second_joint_deg'] = max(ph['max_second_joint_deg'], math.degrees(val))
                if val > self.second_joint_limit_rad:
                    item = {
                        'frame_index': rec.get('frame_index', fi),
                        'phase_name': phase,
                        'leg_id': legacy_leg_id,
                        'leg_name': LEG_NAMES_BY_ID[legacy_leg_id],
                        'abs_angle_deg': math.degrees(val),
                        'excess_deg': math.degrees(val - self.second_joint_limit_rad),
                    }
                    second_violations.append(item)
                    ph['second_joint_violation_count'] += 1

            geom = self._frame_geometry(rec)
            # ground: check second_joint/knee and foot separately.
            # The robot/body point is first transformed to world coordinates by
            # TransformationRobotToABS(..., posture), so body pitch is included.
            # The ground itself is the fixed Gazebo/world plane z = ground_z.
            for legacy_leg_id, g in geom.items():
                for point_name, part_name in (('knee_abs', 'second_joint'), ('foot_abs', 'foot')):
                    z = float(g[point_name][2])
                    clearance = z - self.ground_z
                    base_item = {
                        'frame_index': rec.get('frame_index', fi),
                        'phase_name': phase,
                        'roll_index': rec.get('roll_index'),
                        'phase_step_index': rec.get('phase_step_index'),
                        'leg_id': legacy_leg_id,
                        'leg_name': g['leg_name'],
                        'point': part_name,
                        'z_m': z,
                    }
                    _update_clearance_acc(clearance_by_part[part_name], clearance, base_item, self.ground_tol)
                    _update_clearance_acc(ph['clearance_by_part'][part_name], clearance, base_item, self.ground_tol)
                    if ph['min_clearance_m'] is None or clearance < ph['min_clearance_m']:
                        ph['min_clearance_m'] = clearance
                    if worst_ground is None or clearance < worst_ground['clearance_m']:
                        worst_ground = dict(base_item)
                        worst_ground['clearance_m'] = clearance
                    if clearance < -self.ground_tol:
                        item = dict(base_item)
                        item['penetration_m'] = -clearance
                        ground_violations.append(item)
                        ph['ground_penetration_count'] += 1

            # Inter-leg collision/near check.  Each link is approximated as
            # a capsule: segment distance must be >= 2*leg_radius for no
            # geometric contact, and >= 2*leg_radius+safety_margin for margin.
            ids = sorted(geom.keys())
            link_specs = (('upper', 'upper_segment'), ('lower', 'lower_segment'))
            for ai in range(len(ids)):
                for bi in range(ai + 1, len(ids)):
                    a = geom[ids[ai]]; b = geom[ids[bi]]
                    for link_a_name, link_a_key in link_specs:
                        for link_b_name, link_b_key in link_specs:
                            seg_a = a[link_a_key]
                            seg_b = b[link_b_key]
                            d = _segment_distance(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
                            if ph['min_inter_leg_distance_m'] is None or d < ph['min_inter_leg_distance_m']:
                                ph['min_inter_leg_distance_m'] = d
                            if worst_near is None or d < worst_near['distance_m']:
                                worst_near = {
                                    'frame_index': rec.get('frame_index', fi),
                                    'phase_name': phase,
                                    'roll_index': rec.get('roll_index'),
                                    'phase_step_index': rec.get('phase_step_index'),
                                    'leg_a_id': ids[ai], 'leg_a_name': a['leg_name'],
                                    'link_a': link_a_name,
                                    'leg_b_id': ids[bi], 'leg_b_name': b['leg_name'],
                                    'link_b': link_b_name,
                                    'distance_m': d,
                                    'collision_threshold_m': self.inter_leg_collision_threshold_m,
                                    'required_clearance_m': self.inter_leg_required_clearance_m,
                                    'clearance_margin_m': d - self.inter_leg_required_clearance_m,
                                }
                            base_item = {
                                'frame_index': rec.get('frame_index', fi),
                                'phase_name': phase,
                                'roll_index': rec.get('roll_index'),
                                'phase_step_index': rec.get('phase_step_index'),
                                'leg_a_id': ids[ai], 'leg_a_name': a['leg_name'], 'link_a': link_a_name,
                                'leg_b_id': ids[bi], 'leg_b_name': b['leg_name'], 'link_b': link_b_name,
                                'distance_m': d,
                                'collision_threshold_m': self.inter_leg_collision_threshold_m,
                                'required_clearance_m': self.inter_leg_required_clearance_m,
                                'clearance_margin_m': d - self.inter_leg_required_clearance_m,
                            }
                            if d < self.inter_leg_collision_threshold_m:
                                collision_violations.append(dict(base_item))
                                ph['inter_leg_collision_count'] += 1
                            if d < self.inter_leg_required_clearance_m:
                                near_violations.append(dict(base_item))
                                ph['inter_leg_near_count'] += 1

            # Inter-leg joint-housing check.  This detects cases where the
            # bulky knee / lower-link-root housing of one leg appears to hit
            # another leg's link, even when link centerline-to-centerline
            # capsule checks still have enough clearance.  The joint housing
            # is approximated as a sphere centered at the second joint
            # (knee_abs); other leg links are capsules around upper/lower
            # segments.  Same-leg pairs are excluded.
            for jid in ids:
                joint_leg = geom[jid]
                joint_center = joint_leg['knee_abs']
                for lid in ids:
                    if lid == jid:
                        continue
                    link_leg = geom[lid]
                    for link_name, link_key in link_specs:
                        seg = link_leg[link_key]
                        d = _point_segment_distance(joint_center, seg[0], seg[1])
                        if ph['min_joint_housing_distance_m'] is None or d < ph['min_joint_housing_distance_m']:
                            ph['min_joint_housing_distance_m'] = d
                        if worst_joint_housing is None or d < worst_joint_housing['distance_m']:
                            worst_joint_housing = {
                                'frame_index': rec.get('frame_index', fi),
                                'phase_name': phase,
                                'roll_index': rec.get('roll_index'),
                                'phase_step_index': rec.get('phase_step_index'),
                                'joint_leg_id': jid,
                                'joint_leg_name': joint_leg['leg_name'],
                                'joint_name': 'second_third_joint_housing',
                                'link_leg_id': lid,
                                'link_leg_name': link_leg['leg_name'],
                                'link_name': link_name,
                                'distance_m': d,
                                'collision_threshold_m': self.joint_housing_collision_threshold_m,
                                'required_clearance_m': self.joint_housing_required_clearance_m,
                                'clearance_margin_m': d - self.joint_housing_required_clearance_m,
                            }
                        base_item = {
                            'frame_index': rec.get('frame_index', fi),
                            'phase_name': phase,
                            'roll_index': rec.get('roll_index'),
                            'phase_step_index': rec.get('phase_step_index'),
                            'joint_leg_id': jid,
                            'joint_leg_name': joint_leg['leg_name'],
                            'joint_name': 'second_third_joint_housing',
                            'link_leg_id': lid,
                            'link_leg_name': link_leg['leg_name'],
                            'link_name': link_name,
                            'distance_m': d,
                            'collision_threshold_m': self.joint_housing_collision_threshold_m,
                            'required_clearance_m': self.joint_housing_required_clearance_m,
                            'clearance_margin_m': d - self.joint_housing_required_clearance_m,
                        }
                        if d < self.joint_housing_collision_threshold_m:
                            joint_housing_collision_violations.append(dict(base_item))
                            ph['joint_housing_collision_count'] += 1
                        if d < self.joint_housing_required_clearance_m:
                            joint_housing_near_violations.append(dict(base_item))
                            ph['joint_housing_near_count'] += 1

        phases = [phase_acc[k] for k in sorted(phase_acc.keys())]
        return {
            'frame_count': len(records),
            'second_joint_limit_deg': self.second_joint_limit_deg,
            'max_second_joint_deg': math.degrees(max_second_abs_rad),
            'second_joint_violation_count': len(second_violations),
            'ground_z_m': self.ground_z,
            'ground_tolerance_m': self.ground_tol,
            'ground_penetration_count': len(ground_violations),
            'clearance_by_part': clearance_by_part,
            'second_joint_clearance': clearance_by_part['second_joint'],
            'foot_clearance': clearance_by_part['foot'],
            'inter_leg_limit_m': self.inter_leg_limit_m,
            'inter_leg_link_radius_m': self.leg_radius_m,
            'inter_leg_safety_margin_m': self.inter_leg_safety_margin_m,
            'inter_leg_collision_threshold_m': self.inter_leg_collision_threshold_m,
            'inter_leg_required_clearance_m': self.inter_leg_required_clearance_m,
            'inter_leg_collision_count': len(collision_violations),
            'inter_leg_near_count': len(near_violations),
            'inter_leg_joint_housing_collision_count': len(joint_housing_collision_violations),
            'inter_leg_joint_housing_near_count': len(joint_housing_near_violations),
            'inter_leg_collision': {
                'method': 'capsule_segment_distance',
                'link_radius_m': self.leg_radius_m,
                'collision_threshold_m': self.inter_leg_collision_threshold_m,
                'safety_margin_m': self.inter_leg_safety_margin_m,
                'required_clearance_m': self.inter_leg_required_clearance_m,
                'collision_count': len(collision_violations),
                'near_count': len(near_violations),
                'min_distance_m': None if worst_near is None else worst_near['distance_m'],
                'worst': worst_near,
                'top_collisions': sorted(collision_violations, key=lambda x: x['distance_m'])[:top_n],
                'top_near': sorted(near_violations, key=lambda x: x['distance_m'])[:top_n],
            },
            'inter_leg_joint_housing_collision': {
                'method': 'joint_sphere_to_other_leg_link_capsule_distance',
                'joint_housing_radius_m': self.joint_housing_radius_m,
                'link_radius_m': self.leg_radius_m,
                'collision_threshold_m': self.joint_housing_collision_threshold_m,
                'safety_margin_m': self.joint_housing_safety_margin_m,
                'required_clearance_m': self.joint_housing_required_clearance_m,
                'collision_count': len(joint_housing_collision_violations),
                'near_count': len(joint_housing_near_violations),
                'min_distance_m': None if worst_joint_housing is None else worst_joint_housing['distance_m'],
                'worst': worst_joint_housing,
                'top_collisions': sorted(joint_housing_collision_violations, key=lambda x: x['distance_m'])[:top_n],
                'top_near': sorted(joint_housing_near_violations, key=lambda x: x['distance_m'])[:top_n],
            },
            'worst_second_joint': worst_second,
            'worst_ground_clearance': worst_ground,
            'worst_inter_leg_distance': worst_near,
            'phase_summary': phases,
            'top_second_joint_violations': sorted(second_violations, key=lambda x: x['excess_deg'], reverse=True)[:top_n],
            'top_ground_penetrations': sorted(ground_violations, key=lambda x: x['penetration_m'], reverse=True)[:top_n],
            'top_inter_leg_collisions': sorted(collision_violations, key=lambda x: x['distance_m'])[:top_n],
            'top_inter_leg_near': sorted(near_violations, key=lambda x: x['distance_m'])[:top_n],
            'top_inter_leg_joint_housing_collisions': sorted(joint_housing_collision_violations, key=lambda x: x['distance_m'])[:top_n],
            'top_inter_leg_joint_housing_near': sorted(joint_housing_near_violations, key=lambda x: x['distance_m'])[:top_n],
            'evaluator_note': 'Ground and inter-leg geometry use vendored legacy FK. Body pitch is included by transforming robot/body coordinates to world coordinates before comparing world_z against the fixed Gazebo ground plane. Inter-leg collision uses capsule approximation: upper/lower link segments are transformed to world coordinates and checked by exact segment-segment distance. collision if distance < 2*leg_radius; near if distance < max(inter_leg_limit, 2*leg_radius+safety_margin). v3.0.40 also checks second-third joint housing spheres against other-leg upper/lower link capsules, to catch bulky joint/link visual contacts that link-centerline segment checks can miss.',
        }
