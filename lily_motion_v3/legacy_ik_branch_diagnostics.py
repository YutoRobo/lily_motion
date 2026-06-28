# -*- coding: utf-8 -*-
"""Worst-frame IK branch diagnostics for vendored legacy roll commands.

This module answers a specific question: when the legacy state machine produces a
second-joint violation, is the violation forced by the foot target geometry, or
was a worse IK branch selected?

It intentionally uses the vendored legacy Leg FK/second-joint geometry and the
same analytic IK formulas as legacy_runtime/leg.py, but it does not update the
motion sequence.
"""
from __future__ import print_function, division
import math
import numpy as np
from sympy import Matrix

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_runtime.leg import getTransformationMatrixRobotToLeg
from lily_motion_v3.legacy_runtime.util import TransformationRobotToABS, Posture
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


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


def _command_for_leg(q_rad, legacy_leg_id):
    vals = [0.0, 0.0, 0.0]
    for idx, (lid, jid) in enumerate(JOINT_STATE_ORDER):
        if lid == legacy_leg_id:
            vals[jid] = float(q_rad[idx])
    return vals


def _set_legacy_leg_q(leg_obj, q):
    leg_obj.setTargetDegree([[math.degrees(q[0])], [math.degrees(q[1])], [math.degrees(q[2])]])


def role_for_phase(surface, phase_name, leg_id):
    phase = str(phase_name)
    surface = int(surface)
    support = []
    landing = []
    swing = []
    if phase.startswith('RF-1'):
        if surface == 1:
            swing=[5,7,4,6]; support=[1,3,0,2]
        elif surface == 5:
            swing=[1,3,5,7]; support=[0,2,4,6]
        elif surface == 6:
            swing=[0,2,1,3]; support=[4,6,5,7]
        elif surface == 2:
            swing=[4,6,0,2]; support=[5,7,1,3]
    elif phase.startswith('RF-2'):
        if surface == 1:
            support=[1,3,0,2]; landing=[4,6]
        elif surface == 5:
            support=[0,2,4,6]; landing=[5,7]
        elif surface == 6:
            support=[4,6,5,7]; landing=[1,3]
        elif surface == 2:
            support=[5,7,1,3]; landing=[0,2]
    elif phase.startswith('RF-3') or phase.startswith('RF-4'):
        if surface == 1:
            support=[1,3,4,6]; landing=[0,2]
        elif surface == 5:
            support=[0,2,5,7]; landing=[4,6]
        elif surface == 6:
            support=[4,6,1,3]; landing=[5,7]
        elif surface == 2:
            support=[0,2,5,7]; landing=[1,3]
    elif phase.startswith('RF-5') or phase.startswith('RF-6'):
        if surface == 1:
            support=[0,2,4,6]
        elif surface == 5:
            support=[4,6,5,7]
        elif surface == 6:
            support=[5,7,1,3]
        elif surface == 2:
            support=[1,3,0,2]
    if leg_id in support:
        return 'support'
    if leg_id in landing:
        return 'landing'
    if leg_id in swing:
        return 'swing_direct_degree'
    return 'inactive_or_unknown'


class LegacyIKBranchDiagnostics(object):
    def __init__(self, default_body_z=0.35, second_joint_limit_deg=95.0, ground_z=0.0):
        self.default_body_z = float(default_body_z)
        self.second_joint_limit_deg = float(second_joint_limit_deg)
        self.ground_z = float(ground_z)
        self._emu = LegacyStateMachineEmulator(LegacyStateMachineConfig(z=default_body_z))

    def _candidate_solutions(self, leg_obj, target_pose_robot, posture, prev_q):
        tar_pos = Matrix(target_pose_robot)
        abs_pos = tar_pos.row_insert(4, Matrix([[1]]))
        R_inv = getTransformationMatrixRobotToLeg(leg_obj._Leg__leg_type)
        arm = R_inv * abs_pos
        arm.row_del(3)
        vals = arm.evalf()
        z = float(vals[0]) - leg_obj._Leg__L[1]
        x = float(vals[1])
        y = float(vals[2])
        L2 = leg_obj._Leg__L[2]
        L3 = leg_obj._Leg__L[3]
        theta1_prev = float(prev_q[0]) + math.pi/2.0
        theta2_prev = float(prev_q[1]) + math.pi/2.0
        theta3_prev = float(prev_q[2])
        weight_1 = getattr(leg_obj, '_Leg__weight_1', 1.0)
        weight_2 = getattr(leg_obj, '_Leg__weight_2', 1.0)
        weight_3 = getattr(leg_obj, '_Leg__weight_3', 1.0)
        cval = (x*x+y*y+z*z-L2*L2-L3*L3) / (2.0*L2*L3)
        cval = max(-1.0, min(1.0, cval))
        out = []
        for solve_type in [0,1,2,3]:
            try:
                if solve_type in (0,2):
                    theta3 = math.acos(cval)
                else:
                    theta3 = -math.acos(cval)
                if solve_type in (0,1):
                    theta1_base = math.atan2(y, x)
                    theta2_tmp = math.atan2(-L3*math.sin(theta3)*math.sqrt(x*x+y*y) + (L2+L3*math.cos(theta3))*z,
                                            (L2+L3*math.cos(theta3))*math.sqrt(x*x+y*y) + L3*math.sin(theta3)*z)
                else:
                    theta1_base = math.atan2(-y, -x)
                    theta2_tmp = math.atan2(L3*math.sin(theta3)*math.sqrt(x*x+y*y) + (L2+L3*math.cos(theta3))*z,
                                            -(L2+L3*math.cos(theta3))*math.sqrt(x*x+y*y) + L3*math.sin(theta3)*z)
                theta1 = theta1_prev + min([theta1_base-theta1_prev, theta1_base+2*math.pi-theta1_prev, theta1_base-2*math.pi-theta1_prev], key=abs)
                theta2 = theta2_prev + min([theta2_tmp-theta2_prev, theta2_tmp+2*math.pi-theta2_prev, theta2_tmp-2*math.pi-theta2_prev], key=abs)
                # Same singular-passage correction as legacy Leg, for diagnostics.
                direction = getattr(leg_obj, '_Leg__dirction_at_rolling', 1)
                theta1_before_dir = theta1
                direction_correction = 0.0
                if abs(theta1 - theta1_prev) > math.pi/3.0 and direction == -1:
                    if theta1 - theta1_prev > 0:
                        theta1 -= 2*math.pi
                        direction_correction = -2*math.pi
                    else:
                        theta1 += 2*math.pi
                        direction_correction = 2*math.pi
                q = [theta1-math.pi/2.0, theta2-math.pi/2.0, theta3]
                foot_robot = leg_obj.calcPose(np.array([[q[0]], [q[1]], [q[2]]]))
                knee_robot = leg_obj.calcSecondJointPose(np.array([[q[0]], [q[1]], [q[2]]]))
                foot_abs = TransformationRobotToABS(foot_robot, posture)
                knee_abs = TransformationRobotToABS(knee_robot, posture)
                foot_err = math.sqrt(float((tar_pos[0]-foot_robot[0])**2 + (tar_pos[1]-foot_robot[1])**2 + (tar_pos[2]-foot_robot[2])**2))
                angle_jump = math.sqrt((theta1-theta1_prev)**2*weight_1 + (theta2-theta2_prev)**2*weight_2 + (theta3-theta3_prev)**2*weight_3)
                out.append({
                    'solve_type': solve_type,
                    'q_rad': q,
                    'q_deg': [math.degrees(v) for v in q],
                    'second_joint_abs_deg': abs(math.degrees(q[1])),
                    'second_joint_signed_deg': math.degrees(q[1]),
                    'second_joint_excess_deg': max(0.0, abs(math.degrees(q[1])) - self.second_joint_limit_deg),
                    'knee_world_z_m': float(knee_abs[2]),
                    'knee_clearance_m': float(knee_abs[2]) - self.ground_z,
                    'foot_world_z_m': float(foot_abs[2]),
                    'foot_error_m': foot_err,
                    'weighted_angle_jump_rad': angle_jump,
                    'theta1_direction_correction_rad': direction_correction,
                    'theta1_before_direction_correction_rad': theta1_before_dir,
                })
            except Exception as e:
                out.append({'solve_type': solve_type, 'error': str(e)})
        return out

    def diagnose_frame(self, records, frame_index=None, leg_id=None, phase_name=None, surface_id=None):
        if not records:
            raise ValueError('records is empty')
        if frame_index is None or leg_id is None:
            ev = LegacyConstraintEvaluator(second_joint_limit_deg=self.second_joint_limit_deg, default_body_z=self.default_body_z)
            report = ev.evaluate(records, top_n=1)
            ws = report.get('worst_second_joint') or {}
            frame_index = int(ws.get('frame_index', 0)) if frame_index is None else int(frame_index)
            leg_id = int(ws.get('leg_id', 0)) if leg_id is None else int(leg_id)
        # find record by frame_index or list index fallback
        rec = None
        rec_pos = None
        for i, r in enumerate(records):
            if int(r.get('frame_index', i)) == int(frame_index):
                rec = r; rec_pos = i; break
        if rec is None:
            rec_pos = max(0, min(len(records)-1, int(frame_index)))
            rec = records[rec_pos]
        prev = records[max(0, rec_pos-1)]
        phase = phase_name or rec.get('phase_name', 'unknown')
        surface = surface_id if surface_id is not None else rec.get('surface_start', None)
        if surface is None:
            surface = 1
        leg_obj = self._emu.legs_by_legacy_id[int(leg_id)]
        current_q = _command_for_leg(rec['joint_command_rad'], int(leg_id))
        prev_q = _command_for_leg(prev['joint_command_rad'], int(leg_id))
        _set_legacy_leg_q(leg_obj, current_q)
        target_pose_robot = leg_obj.getEndEffectorPose()
        posture = _posture_from_record(rec, default_z=self.default_body_z)
        candidates = self._candidate_solutions(leg_obj, target_pose_robot, posture, prev_q)
        selected_q_deg = [math.degrees(v) for v in current_q]
        selected_solve = None
        best_dist = None
        for c in candidates:
            if 'q_rad' not in c:
                continue
            d = sum((c['q_rad'][i]-current_q[i])**2 for i in range(3))
            if best_dist is None or d < best_dist:
                best_dist = d; selected_solve = c.get('solve_type')
        feasible = [c for c in candidates if 'q_rad' in c and c['second_joint_abs_deg'] <= self.second_joint_limit_deg and c['knee_clearance_m'] >= -1e-4]
        return {
            'frame_index': int(rec.get('frame_index', frame_index)),
            'record_position': rec_pos,
            'phase_name': phase,
            'surface_id': int(surface),
            'leg_id': int(leg_id),
            'leg_name': LEG_NAMES_BY_ID[int(leg_id)],
            'role': role_for_phase(surface, phase, int(leg_id)),
            'selected_q_deg': selected_q_deg,
            'selected_second_joint_abs_deg': abs(selected_q_deg[1]),
            'selected_solve_type_estimate': selected_solve,
            'has_candidate_within_second_joint_limit': bool(feasible),
            'candidate_count_within_second_joint_limit': len(feasible),
            'target_foot_position_robot_m': [float(target_pose_robot[0]), float(target_pose_robot[1]), float(target_pose_robot[2])],
            'base_pose': rec.get('base_pose'),
            'candidates': candidates,
            'diagnosis_hint': ('IK branch may be improvable' if feasible else 'No IK branch satisfies the second-joint limit for this foot target; change geometry/body/support parameters'),
        }
