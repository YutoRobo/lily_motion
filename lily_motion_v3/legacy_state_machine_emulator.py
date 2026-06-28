# -*- coding: utf-8 -*-
"""Vendored legacy roll state-machine emulator for v3.0.22.

This module does not import the user's ROS/catkin package.  It vendors the
small legacy runtime files supplied by the user and executes the old
LilyRobot/EndEfectorManager/Leg/Servo update pattern locally:

  supportMove / landingMove / swingMove / calcAnalyticalInverseKinematics

The purpose is not to design a new gait.  It is to reproduce the *command
sequence structure* of the legacy roll controller closely enough that Gazebo
can be used to compare against the original videos.
"""
from __future__ import division, print_function
import json
import math
import os

from sympy import Matrix

from lily_motion_v3.legacy_runtime import servo
from lily_motion_v3.legacy_runtime import leg as legacy_leg
from lily_motion_v3.legacy_runtime.end_efector_manager import EndEfectorManager
from lily_motion_v3.legacy_runtime.lily_robot import LilyRobot
from lily_motion_v3.legacy_runtime.util import Posture
from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID

LEGACY_ID_TO_NAME = {0:"BLF",1:"BLH",2:"BRF",3:"BRH",4:"TLF",5:"TLH",6:"TRF",7:"TRH"}
LEGACY_NAME_TO_ID = dict((v,k) for k,v in LEGACY_ID_TO_NAME.items())
FORWARD_NEXT_SURFACE = {1:5, 5:6, 6:2, 2:1}


class LegacyStateMachineConfig(object):
    def __init__(self, move_dist=0.4, support_dist=0.7, max_step=30,
                 surface_id=1, z=0.35, initialize_step=100,
                 include_initialize=False,
                 goal2_dist_front=0.4, goal2_x_scale=1.0,
                 goal2_pitch_scale=1.0, goal2_landing_z=0.0,
                 goal3_lift_z=0.05, goal3_target_x=0.2,
                 goal4_target_x=0.05,
                 middle_swing_y_escape=0.0,
                 middle_swing_y_escape_mode='none',
                 middle_swing_y_escape_apply_rf3=False,
                 middle_swing_y_escape_apply_rf4=False,
                 goal5_x_scale=1.0, goal5_pitch_scale=1.0,
                 rf1_current_angle_anchor=False):
        self.move_dist = float(move_dist)
        self.support_dist = float(support_dist)
        self.max_step = int(max_step)
        self.surface_id = int(surface_id)
        self.z = float(z)
        self.initialize_step = int(initialize_step)
        self.include_initialize = bool(include_initialize)
        # v3.0.25 diagnostic knobs.  Defaults reproduce the supplied legacy
        # controller.  They are intentionally narrow and phase-specific so the
        # original state-machine structure is not silently changed.
        self.goal2_dist_front = float(goal2_dist_front)
        self.goal2_x_scale = float(goal2_x_scale)
        self.goal2_pitch_scale = float(goal2_pitch_scale)
        self.goal2_landing_z = float(goal2_landing_z)
        self.goal3_lift_z = float(goal3_lift_z)
        self.goal3_target_x = float(goal3_target_x)
        self.goal4_target_x = float(goal4_target_x)
        # v3.0.43A: optional local Y escape for the middle-pair swing in
        # RF-3/RF-4.  Defaults must reproduce the legacy trajectory exactly.
        # This is intentionally limited to the central lift/land pair so that
        # any change can be attributed to the Y-escape experiment, not to a
        # broader gait redesign.
        self.middle_swing_y_escape = float(middle_swing_y_escape)
        self.middle_swing_y_escape_mode = str(middle_swing_y_escape_mode)
        self.middle_swing_y_escape_apply_rf3 = bool(middle_swing_y_escape_apply_rf3)
        self.middle_swing_y_escape_apply_rf4 = bool(middle_swing_y_escape_apply_rf4)
        self.goal5_x_scale = float(goal5_x_scale)
        self.goal5_pitch_scale = float(goal5_pitch_scale)
        # v3.0.36: When a repeated roll enters RF-1, optionally emit one
        # current-servo frame before setSwingLeg() starts moving toward the
        # RF-1 preswing target.  This does not average across surface
        # boundaries and does not preblend the previous roll.  It only makes
        # RF-1 begin from the actual terminal servo state.
        self.rf1_current_angle_anchor = bool(rf1_current_angle_anchor)


class LegacyStateMachineEmulator(object):
    def __init__(self, config=None):
        self.config = config or LegacyStateMachineConfig()
        self.servos_by_legacy_id = {}
        self.legs_by_legacy_id = {}
        self.managers_by_legacy_id = {}
        managers = []
        for legacy_id in range(8):
            name = LEGACY_ID_TO_NAME[legacy_id]
            ss, lg, mgr = self._make_leg(name)
            self.servos_by_legacy_id[legacy_id] = ss
            self.legs_by_legacy_id[legacy_id] = lg
            self.managers_by_legacy_id[legacy_id] = mgr
            managers.append(mgr)
        self.lily = LilyRobot(managers)
        self.lily.setRobotParam(body=0.3)
        self.lily.setPosture(Posture(z=self.config.z))
        self.lily.setDefaultSupportDist(self.config.support_dist)
        self._records = []
        self._frame_index = 0
        self._active_surface = self.config.surface_id
        self._active_roll_index = 0
        # v3.0.31: mimic LilyRobotController.__x / __pitch.
        # Earlier repeated-roll code generated each quarter roll with x=0 local
        # landing targets.  The supplied legacy controller uses cumulative
        # __x in RF-2/RF-3/RF-4 landing targets and then snaps lily.posture
        # to (__x, __pitch) after the stop-adjustment phase.  Without this,
        # the second and later quarter-rolls are not the same state machine as
        # the original program.
        self._controller_x = 0.0
        self._controller_pitch = 0.0
        self._boundary_summaries = []

    def _make_leg(self, leg_name):
        DEGREE_RANGE = 360 + 20
        DEGREE_RANGE2 = 270
        DEGREE_RANGE3 = 180
        ss = [servo.ServoMotor(), servo.ServoMotor(), servo.ServoMotor()]
        ss[0].setDegreeRange(-DEGREE_RANGE, DEGREE_RANGE)
        ss[1].setDegreeRange(-DEGREE_RANGE2, DEGREE_RANGE2)
        ss[2].setDegreeRange(-DEGREE_RANGE3, DEGREE_RANGE3)
        lg = legacy_leg.Leg(*ss)
        lg.setLegType(leg_name)
        lg.setLinkLength([0, 0.05, 0.3, 0.3])
        return ss, lg, EndEfectorManager(lg)

    def initialize(self, record=False):
        for legacy_id in range(8):
            if legacy_id <= 3:
                deg = [0.0, -45.0, 100.0]
            else:
                deg = [0.0, 45.0, -100.0]
            self.legs_by_legacy_id[legacy_id].setTargetDegree([[deg[0]], [deg[1]], [deg[2]]])
        step = self.config.initialize_step
        self.lily.setDefaultPose(max_step=step)
        for i in range(2 * step):
            self.lily.calcInverseKinematics(use_all_leg=True)
            if record:
                self._record("INIT_DefaultPose", i, 2 * step)

    def run_forward_roll(self):
        self.initialize(record=self.config.include_initialize)
        self._active_surface = self.config.surface_id
        self._active_roll_index = 0
        # v3.0.31: mimic LilyRobotController.__x / __pitch.
        # Earlier repeated-roll code generated each quarter roll with x=0 local
        # landing targets.  The supplied legacy controller uses cumulative
        # __x in RF-2/RF-3/RF-4 landing targets and then snaps lily.posture
        # to (__x, __pitch) after the stop-adjustment phase.  Without this,
        # the second and later quarter-rolls are not the same state machine as
        # the original program.
        self._controller_x = 0.0
        self._controller_pitch = 0.0
        self._boundary_summaries = []
        self._roll_forward_surface(self.config.surface_id)
        return list(self._records)

    def _record(self, phase_name, phase_step_index, phase_step_count):
        q_rad = self.current_command_rad_joint_state_order()
        q_deg = [math.degrees(v) for v in q_rad]
        rec = {
            "command_index": self._frame_index,
            "frame_index": self._frame_index,
            "profile": "legacy_state_machine",
            "command_source": "legacy_runtime_servo_state",
            "phase_name": str(phase_name),
            "phase_step_index": int(phase_step_index),
            "phase_step_count": int(phase_step_count),
            "surface_start": int(self._active_surface),
            "roll_index": int(self._active_roll_index),
            "joint_command_rad": q_rad,
            "joint_command_deg": q_deg,
            "base_pose": {
                "x": float(self.lily.posture.x),
                "y": float(self.lily.posture.y),
                "z": float(self.lily.posture.z),
                "roll": float(self.lily.posture.roll),
                "pitch": float(self.lily.posture.pitch),
                "yaw": float(self.lily.posture.yaw),
            },
        }
        self._records.append(rec)
        self._frame_index += 1

    def current_command_rad_joint_state_order(self):
        out = []
        for legacy_leg_id, joint_index in JOINT_STATE_ORDER:
            lg = self.legs_by_legacy_id[legacy_leg_id]
            deg = float(lg.getServosDeg()[joint_index][0])
            out.append(math.radians(deg))
        return out

    def _loop_support_swing(self, n, phase_name):
        for i in range(n):
            self.lily.suportMove()
            self.lily.calcAnalyticalInverseKinematics()
            self.lily.swingMove()
            self._record(phase_name, i, n)

    def _loop_support_landing(self, n, phase_name):
        for i in range(n):
            self.lily.suportMove()
            self.lily.landingMove()
            self.lily.calcAnalyticalInverseKinematics()
            self._record(phase_name, i, n)

    def _loop_support_only(self, n, phase_name):
        for i in range(n):
            self.lily.suportMove()
            self.lily.calcAnalyticalInverseKinematics()
            self._record(phase_name, i, n)

    def _middle_swing_y_targets(self, phase_name):
        """Return (+Y, -Y) targets for RF-3/RF-4 middle-pair motion.

        v3.0.43A keeps the legacy behavior as the exact default:
            +support_dist/2, -support_dist/2

        Non-zero escape is only applied when the corresponding RF phase is
        explicitly enabled.  The modes are intentionally simple so the sweep
        can isolate whether a lateral Y escape helps the observed middle-leg
        proximity / interference suspicion.
        """
        sd = float(self.config.support_dist)
        y_pos = sd / 2.0
        y_neg = -sd / 2.0

        esc = float(getattr(self.config, 'middle_swing_y_escape', 0.0))
        mode = str(getattr(self.config, 'middle_swing_y_escape_mode', 'none'))
        apply_rf3 = bool(getattr(self.config, 'middle_swing_y_escape_apply_rf3', False))
        apply_rf4 = bool(getattr(self.config, 'middle_swing_y_escape_apply_rf4', False))

        apply_escape = ((phase_name == 'RF-3' and apply_rf3) or
                        (phase_name == 'RF-4' and apply_rf4))
        if (not apply_escape) or abs(esc) < 1e-12 or mode == 'none':
            return y_pos, y_neg

        if mode == 'outward':
            return y_pos + esc, y_neg - esc
        if mode == 'inward':
            return y_pos - esc, y_neg + esc
        if mode == 'same_sign_plus':
            return y_pos + esc, y_neg + esc
        if mode == 'same_sign_minus':
            return y_pos - esc, y_neg - esc

        raise ValueError('unknown middle_swing_y_escape_mode: %s' % mode)

    def _roll_forward_surface(self, surface):
        if surface not in (1,5,6,2):
            raise ValueError("surface must be one of 1,5,6,2")
        # Legacy controller uses cumulative self.__x for absolute landing
        # targets.  This is essential for repeated rolls.
        x = float(self._controller_x)
        md = self.config.move_dist
        sd = self.config.support_dist
        max_step = self.config.max_step

        # Goal 1: upper legs slight change + small support move.
        # v3.0.36: emit a current-angle anchor at the start of RF-1 when
        # requested.  The important point is placement: this happens before
        # setSwingLeg()/setSupportMove() create the RF-1 interpolation queue,
        # so the first RF-1 sample can no longer be the already-moved target.
        # This is not boundary smoothing; it is an explicit RF-1 start sample.
        if self.config.rf1_current_angle_anchor:
            self._record("RF-1_Goal1_UpperLegPreSwing_CurrentAngleAnchor", 0, 1)
        self.lily.setWeight(weight_1=0.1, weight_2=1, weight_3=1)
        rate = 0.1
        if surface == 1:
            self.lily.setSwingLeg(swing_leg=[5,7,4,6], swing_leg_type=[0,0,1,1], max_step=int(max_step*rate))
            self.lily.setSupportLeg(support_leg=[1,3,0,2])
        elif surface == 5:
            self.lily.setSwingLeg(swing_leg=[1,3,5,7], swing_leg_type=[5,6,0,0], max_step=int(max_step*rate))
            self.lily.setSupportLeg(support_leg=[0,2,4,6])
        elif surface == 6:
            self.lily.setSwingLeg(swing_leg=[0,2,1,3], swing_leg_type=[1,1,5,6], max_step=int(max_step*rate))
            self.lily.setSupportLeg(support_leg=[4,6,5,7])
        elif surface == 2:
            self.lily.setSwingLeg(swing_leg=[4,6,0,2], swing_leg_type=[1,1,1,1], max_step=int(max_step*rate))
            self.lily.setSupportLeg(support_leg=[5,7,1,3])
        self.lily.setSupportMove(Posture(x=md/2*rate, z=0.0, pitch=math.pi/4*rate), int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_swing(int(max_step*rate), "RF-1_Goal1_UpperLegPreSwing")

        # Goal 2: upper pair landing while rolling.
        dist_front = self.config.goal2_dist_front
        self.lily.setWeight(weight_1=1, weight_2=1, weight_3=1)
        rate = 1.0-rate
        if surface == 1:
            self.lily.setSupportLeg(support_leg=[1,3,0,2]); landing=[4,6]
        elif surface == 5:
            self.lily.setSupportLeg(support_leg=[0,2,4,6]); landing=[5,7]
        elif surface == 6:
            self.lily.setSupportLeg(support_leg=[4,6,5,7]); landing=[1,3]
        elif surface == 2:
            self.lily.setSupportLeg(support_leg=[5,7,1,3]); landing=[0,2]
        self.lily.setLandingMove([
            Matrix([[x+sd/2+dist_front],[sd/2],[self.config.goal2_landing_z]]),
            Matrix([[x+sd/2+dist_front],[-sd/2],[self.config.goal2_landing_z]])],
            landing_leg=landing, lending_leg_type=[-1,-1], max_step=int(max_step*rate))
        self.lily.setSupportMove(Posture(x=md/2*rate*self.config.goal2_x_scale,
                                          z=0.0,
                                          pitch=math.pi/4*rate*self.config.goal2_pitch_scale),
                                int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_landing(int(max_step*rate), "RF-2_Goal2_UpperLegLanding")

        # Goal 3: lift middle pair among six grounded legs.
        rate = 0.1
        if surface == 1:
            self.lily.setSupportLeg(support_leg=[1,3,4,6]); landing=[0,2]
        elif surface == 5:
            self.lily.setSupportLeg(support_leg=[0,2,5,7]); landing=[4,6]
        elif surface == 6:
            self.lily.setSupportLeg(support_leg=[4,6,1,3]); landing=[5,7]
        elif surface == 2:
            self.lily.setSupportLeg(support_leg=[0,2,5,7]); landing=[1,3]
        y_pos, y_neg = self._middle_swing_y_targets('RF-3')
        self.lily.setLandingMove([
            Matrix([[x+self.config.goal3_target_x],[y_pos],[self.config.goal3_lift_z]]),
            Matrix([[x+self.config.goal3_target_x],[y_neg],[self.config.goal3_lift_z]])],
            landing_leg=landing, lending_leg_type=[-1,-1], max_step=int(max_step*rate))
        self.lily.setSupportMove(Posture(x=0,z=0.0,pitch=0.0), int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_landing(int(max_step*rate), "RF-3_Goal3_LiftMiddlePair")

        # Goal 4: land lifted middle pair.
        self.lily.setWeight(weight_1=10, weight_2=1, weight_3=1)
        rate = 0.1
        # support and landing same as Goal 3.
        y_pos, y_neg = self._middle_swing_y_targets('RF-4')
        self.lily.setLandingMove([
            Matrix([[x+self.config.goal4_target_x],[y_pos],[0.0]]),
            Matrix([[x+self.config.goal4_target_x],[y_neg],[0.0]])],
            landing_leg=landing, lending_leg_type=[-1,-1], max_step=int(max_step*rate))
        self.lily.setSupportMove(Posture(x=0,z=0.0,pitch=0.0), int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_landing(int(max_step*rate), "RF-4_Goal4_LandMiddlePair")

        # Goal 5: main body roll with next support set.
        rate = 1.0
        if surface == 1:
            self.lily.setSupportLeg(support_leg=[0,2,4,6])
        elif surface == 5:
            self.lily.setSupportLeg(support_leg=[4,6,5,7])
        elif surface == 6:
            self.lily.setSupportLeg(support_leg=[5,7,1,3])
        elif surface == 2:
            self.lily.setSupportLeg(support_leg=[1,3,0,2])
        self.lily.setSupportMove(Posture(x=md/2*rate*self.config.goal5_x_scale, z=0.0, pitch=math.pi/4*rate*self.config.goal5_pitch_scale), int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_only(int(max_step*rate), "RF-5_Goal5_MainBodyRoll")

        # Stop adjustment.
        rate = 0.1
        self.lily.setSupportMove(Posture(), int(max_step*rate), support_solve_type=[-1]*8)
        self._loop_support_only(int(max_step*rate), "RF-6_StopAdjustment")

        # Match the supplied LilyRobotController.roll() terminal state update:
        #   self.__x += move_dist
        #   self.__pitch += pi/2
        #   self.lily.posture = Posture(x=self.__x, pitch=self.__pitch, z=...)
        #   self.surface.move(direction)
        # The exact snap happens *after* the stop-adjustment frames are
        # published, so it mainly affects the next quarter roll.
        self._controller_x = x + md
        self._controller_pitch = float(self._controller_pitch) + math.pi/2.0
        self.lily.posture = Posture(x=self._controller_x,
                                    pitch=self._controller_pitch,
                                    z=self.lily.posture.z)
        self._boundary_summaries.append({
            'roll_index': int(self._active_roll_index),
            'surface_start': int(surface),
            'surface_after': int(FORWARD_NEXT_SURFACE.get(int(surface), -1)),
            'controller_x': float(self._controller_x),
            'controller_pitch': float(self._controller_pitch),
            'lily_posture': {
                'x': float(self.lily.posture.x),
                'y': float(self.lily.posture.y),
                'z': float(self.lily.posture.z),
                'roll': float(self.lily.posture.roll),
                'pitch': float(self.lily.posture.pitch),
                'yaw': float(self.lily.posture.yaw),
            },
            'frame_index_after_roll': int(self._frame_index - 1),
        })


    def run_forward_repeated(self, surface_sequence=None, include_initialize=None):
        """Run repeated quarter-rolls on one vendored legacy runtime instance.

        ``surface_sequence`` is a list such as [1, 5, 6, 2, 1].  Each adjacent
        pair represents one quarter roll.  The last item is only the terminal
        surface and is not executed as a start surface.
        """
        if surface_sequence is None:
            surface_sequence = [1, 5, 6, 2, 1]
        surface_sequence = [int(x) for x in surface_sequence]
        if len(surface_sequence) < 2:
            raise ValueError("surface_sequence must contain at least two surfaces")
        if include_initialize is None:
            include_initialize = self.config.include_initialize
        self._controller_x = 0.0
        self._controller_pitch = 0.0
        self._boundary_summaries = []
        self.initialize(record=bool(include_initialize))
        for i, surface in enumerate(surface_sequence[:-1]):
            expected_next = FORWARD_NEXT_SURFACE.get(int(surface))
            if expected_next != int(surface_sequence[i + 1]):
                raise ValueError("invalid forward surface transition: %s -> %s" % (surface, surface_sequence[i + 1]))
            self._active_surface = int(surface)
            self._active_roll_index = int(i)
            start_index = self._frame_index
            self._roll_forward_surface(int(surface))
            # Mark the generated records with terminal surface and within-roll index.
            for rec in self._records[start_index:]:
                rec['surface_after'] = int(surface_sequence[i + 1])
                rec['roll_surface_transition'] = "%s->%s" % (surface, surface_sequence[i + 1])
        return list(self._records)


def command_diagnostics(records):
    if not records:
        return {}
    n = len(records[0]["joint_command_rad"])
    mins = [min(r["joint_command_rad"][i] for r in records) for i in range(n)]
    maxs = [max(r["joint_command_rad"][i] for r in records) for i in range(n)]
    deltas = [maxs[i]-mins[i] for i in range(n)]
    return {
        "frame_count": len(records),
        "nonzero_joint_count": sum(1 for d in deltas if abs(d) > 1e-9),
        "max_delta_rad": max(deltas) if deltas else 0.0,
        "max_delta_deg": math.degrees(max(deltas)) if deltas else 0.0,
        "deltas_rad": deltas,
        "deltas_deg": [math.degrees(d) for d in deltas],
    }


def write_jsonl(records, path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, 'w') as f:
        for r in records:
            f.write(json.dumps(r, sort_keys=True))
            f.write('\n')
