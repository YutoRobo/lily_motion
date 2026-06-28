# -*- coding: utf-8 -*-
"""Spec-level reproduction of the legacy roll() sequence in v3-core.

v3.0.21 intentionally follows the old controller's *procedure* rather than the
previous qualitative legacy-style scaffold.  It still does not import the old
project; the five forward-roll goals are represented as tables extracted from
lily_controller.py and simulated through the common V3RollCandidate schema.

Limitations:
  * Kinematics still use the v3 project-contained model, not the huge symbolic
    old leg.py formulas.
  * The legacy IK branch-selection policy is approximated by v3 IK continuity.
  * Direct swing-angle commands are represented explicitly in the raw q stream.
"""
from __future__ import division
import math

from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.roll_candidate import V3MotionFrame, V3RollCandidate
from lily_motion_v3.motion_evaluation_report import MotionEvaluationReport
from lily_motion_v3.contact_state import ContactState
from lily_motion_v3.phase_spec import PhaseSpec
from lily_motion_v3 import leg_role as R
from lily_motion_v3.transforms import vec_add, vec_sub, mat_vec_mul, mat_transpose


LEGACY_ID_TO_NAME = {
    0: "BLF", 1: "BLH", 2: "BRF", 3: "BRH",
    4: "TLF", 5: "TLH", 6: "TRF", 7: "TRH",
}

FORWARD_NEXT_SURFACE = {1: 5, 5: 6, 6: 2, 2: 1}


class LegacyRollSpecGenerationConfig(object):
    def __init__(self, move_dist=0.4, support_dist=0.7, max_step=30,
                 surface_id=1, z=0.35, direction="forward",
                 ground_z=0.0, second_joint_abs_max_deg=95.0,
                 support_solve_type=-1, landing_solve_type=-1):
        self.move_dist = float(move_dist)
        self.support_dist = float(support_dist)
        self.max_step = int(max_step)
        self.surface_id = int(surface_id)
        self.z = float(z)
        self.direction = str(direction)
        self.ground_z = float(ground_z)
        self.second_joint_abs_max_deg = float(second_joint_abs_max_deg)
        self.support_solve_type = int(support_solve_type)
        self.landing_solve_type = int(landing_solve_type)


class LegacyRollSpecCandidateGenerator(object):
    def __init__(self, robot_model=None, config=None):
        self.robot_model = robot_model or RobotModel()
        self.config = config or LegacyRollSpecGenerationConfig()
        self._v3_id_by_legacy_id = {}
        for legacy_id, name in LEGACY_ID_TO_NAME.items():
            self._v3_id_by_legacy_id[legacy_id] = self.robot_model.leg_id(name)

    def _lid(self, legacy_id):
        return self._v3_id_by_legacy_id[int(legacy_id)]

    def _lids(self, legacy_ids):
        return [self._lid(i) for i in legacy_ids]

    def default_initial_joint_map(self):
        # From LilyRobotController.initialize(): bottom legacy ids 0..3 use
        # [0,-45,100] deg; top legacy ids 4..7 use [0,45,-100] deg.
        out = {}
        for legacy_id in range(8):
            if legacy_id <= 3:
                deg = [0.0, -45.0, 100.0]
            else:
                deg = [0.0, 45.0, -100.0]
            out[self._lid(legacy_id)] = [math.radians(v) for v in deg]
        return out

    def base_pose0(self):
        return {"x": 0.0, "y": 0.0, "z": self.config.z, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def generate_forward_one_roll(self, initial_joint_map=None, surface_id=None):
        if surface_id is None:
            surface_id = self.config.surface_id
        surface_id = int(surface_id)
        if surface_id not in (1, 5, 6, 2):
            raise ValueError("legacy roll spec currently supports surface 1,5,6,2")
        joint_map = self._copy_joint_map(initial_joint_map or self.default_initial_joint_map())
        base_pose = self.base_pose0()
        x_state = 0.0
        foot_world = dict((lid, self.robot_model.foot_position_world(lid, q, base_pose)) for lid, q in joint_map.items())
        frames = []
        phases = []
        frame_index = 0
        report = MotionEvaluationReport()
        ik_failures = []
        ground_penetrations = []
        max_second_joint_deg = 0.0
        max_joint_delta_deg = 0.0

        steps = self._legacy_forward_steps(surface_id, x_state)
        for phase_index, step in enumerate(steps):
            step_count = max(1, int(round(self.config.max_step * step["rate"])))
            cs = ContactState(
                surface_id,
                support_legs=self._lids(step.get("support", [])),
                candidate_support_legs=self._lids(step.get("landing", [])),
                lift_legs=self._lids(step.get("landing", [])) if step["kind"] in ("lift_middle", "land_middle") else [],
                clearance_legs=self._lids(step.get("swing", [])),
            )
            phase = PhaseSpec(step["name"], step["description"], cs, [], [])
            phases.append(phase)
            start_base = dict(base_pose)
            start_foot_world = self._copy_targets(foot_world)
            landing_start_world = self._copy_targets(foot_world)
            landing_goals = dict((self._lid(k), v) for k, v in step.get("landing_targets", {}).items())
            direct_swing_targets = dict((self._lid(k), v) for k, v in step.get("swing_targets_deg", {}).items())
            direct_swing_start = self._copy_joint_map(joint_map)

            # Legacy setSupportMove stores absolute positions at phase start.
            support_start_world = self._copy_targets(foot_world)
            for phase_step_index in range(step_count):
                alpha = float(phase_step_index + 1) / float(step_count)
                prev_joint_map = self._copy_joint_map(joint_map)
                # Legacy suportMove: posture is incremented first.
                base_pose = dict(start_base)
                base_pose["x"] += step.get("support_dx", 0.0) * alpha
                base_pose["z"] += step.get("support_dz", 0.0) * alpha
                base_pose["pitch"] += step.get("support_dpitch", 0.0) * alpha

                leg_roles = self._roles_for_step(step)
                body_targets = {}
                world_targets_frame = self._copy_targets(foot_world)

                # Support feet keep the phase-start absolute/world point.
                for lid in self._lids(step.get("support", [])):
                    world_targets_frame[lid] = list(support_start_world[lid])
                    body_targets[lid] = self.robot_model.world_point_to_body(world_targets_frame[lid], base_pose)

                # Landing legs linearly move in absolute/world coordinates.
                for lid, goal in landing_goals.items():
                    start = landing_start_world[lid]
                    world_targets_frame[lid] = [start[i] + (goal[i] - start[i]) * alpha for i in range(3)]
                    body_targets[lid] = self.robot_model.world_point_to_body(world_targets_frame[lid], base_pose)

                # Other legs retain current body configuration unless overwritten.
                for lid, q in joint_map.items():
                    if lid not in body_targets:
                        body_targets[lid] = self.robot_model.foot_position_body(lid, q)
                        world_targets_frame[lid] = self.robot_model.body_point_to_world(body_targets[lid], base_pose)

                # IK update for support and landing legs.
                frame_ik_failures = []
                for lid in sorted(set(self._lids(step.get("support", []))) | set(landing_goals.keys())):
                    selected = self.robot_model.select_ik_body(lid, body_targets[lid], previous_q=joint_map.get(lid))
                    if selected is None:
                        rec = {
                            "frame_index": frame_index,
                            "phase_name": step["name"],
                            "phase_step_index": phase_step_index,
                            "leg_id": lid,
                            "leg_name": self.robot_model.leg_name(lid),
                            "role": leg_roles.get(lid, R.OTHER),
                            "target_body": list(body_targets[lid]),
                            "target_world": list(world_targets_frame[lid]),
                            "base_pose": dict(base_pose),
                        }
                        ik_failures.append(rec)
                        frame_ik_failures.append(rec)
                    else:
                        joint_map[lid] = list(selected.q)

                # Direct swing angle targets are raw joint commands, like setTargetDegree.
                for lid, target_deg in direct_swing_targets.items():
                    start_q = direct_swing_start[lid]
                    target_q = [math.radians(v) for v in target_deg]
                    joint_map[lid] = [start_q[i] + (target_q[i] - start_q[i]) * alpha for i in range(3)]

                # Refresh body/world targets from actual joint map for frame storage.
                body_targets = dict((lid, self.robot_model.foot_position_body(lid, q)) for lid, q in joint_map.items())
                world_targets_frame = dict((lid, self.robot_model.body_point_to_world(p, base_pose)) for lid, p in body_targets.items())
                foot_world = self._copy_targets(world_targets_frame)

                frame_diag = {"ik_failures": frame_ik_failures}
                ground_diag = self._ground_diag(joint_map, base_pose, step["name"], phase_step_index, frame_index)
                frame_diag["ground_clearance"] = ground_diag
                if ground_diag["penetrating"]:
                    ground_penetrations.append(ground_diag)
                for lid, q in joint_map.items():
                    max_second_joint_deg = max(max_second_joint_deg, abs(math.degrees(q[1])))
                    prev = prev_joint_map.get(lid)
                    if prev is not None:
                        local = max(abs(math.degrees(self._angle_delta(q[i], prev[i]))) for i in range(3))
                        max_joint_delta_deg = max(max_joint_delta_deg, local)

                frames.append(V3MotionFrame(
                    frame_index=frame_index,
                    phase_index=phase_index,
                    phase_name=step["name"],
                    phase_step_index=phase_step_index,
                    phase_step_count=step_count,
                    contact_state=cs,
                    base_pose=dict(base_pose),
                    leg_roles=leg_roles,
                    foot_targets_body=body_targets,
                    foot_targets_world=world_targets_frame,
                    joint_angles=self._copy_joint_map(joint_map),
                    diagnostics=frame_diag,
                ))
                frame_index += 1

        # Legacy state update after roll.
        final_surface = FORWARD_NEXT_SURFACE.get(surface_id, surface_id)
        report.task_success = {
            "completed": len(ik_failures) == 0 and len(ground_penetrations) == 0,
            "direction": "forward",
            "profile": "legacy_roll_spec",
            "surface_start": surface_id,
            "surface_after": final_surface,
            "planned_phase_count": len(phases),
            "frame_count": len(frames),
            "legacy_dependency": False,
            "legacy_spec_source": "lily_controller.roll(Direction.FORWARD) five-goal sequence",
            "pitch_only_transform": True,
            "move_dist": self.config.move_dist,
            "support_dist": self.config.support_dist,
            "max_step": self.config.max_step,
        }
        report.ik_reachability = {"ik_failure_count": len(ik_failures), "top_failure_records": ik_failures[:20]}
        report.ground_clearance = {"ground_z": self.config.ground_z, "penetration_count": len(ground_penetrations), "top_penetration_records": ground_penetrations[:20]}
        report.joint_limit = {"second_joint_abs_max_deg": self.config.second_joint_abs_max_deg, "max_abs_second_joint_deg": max_second_joint_deg, "second_joint_limit_ok": max_second_joint_deg <= self.config.second_joint_abs_max_deg + 1e-9}
        report.motion_discontinuity = {"max_joint_delta_deg": max_joint_delta_deg, "warn_threshold_deg": 120.0, "discontinuity_count": 0, "top_records": []}
        report.notes.append("v3.0.21: legacy_roll_spec follows the old roll() five-goal tables; old code is not imported.")
        return V3RollCandidate("forward", phases, frames, report)

    def _legacy_forward_steps(self, surface, x_state):
        md = self.config.move_dist
        sd = self.config.support_dist
        dist_front = 0.4
        steps = []
        # Goal 1.
        p1 = {
            1: ([5, 7, 4, 6], [0, 0, 1, 1], [1, 3, 0, 2]),
            5: ([1, 3, 5, 7], [5, 6, 0, 0], [0, 2, 4, 6]),
            6: ([0, 2, 1, 3], [1, 1, 5, 6], [4, 6, 5, 7]),
            2: ([4, 6, 0, 2], [1, 1, 1, 1], [5, 7, 1, 3]),
        }[surface]
        steps.append(self._step("RF-1_Goal1_UpperLegPreSwing", "目標1: 上の脚を少し変化", 0.1, p1[2], swing=p1[0], swing_types=p1[1], support_dx=md/2*0.1, support_dpitch=math.pi/4*0.1))
        # Goal 2.
        land2 = {1: [4, 6], 5: [5, 7], 6: [1, 3], 2: [0, 2]}[surface]
        support2 = {1: [1,3,0,2], 5: [0,2,4,6], 6: [4,6,5,7], 2: [5,7,1,3]}[surface]
        targets2 = self._pair_targets(land2, x_state + sd/2 + dist_front, sd/2, 0.0)
        steps.append(self._step("RF-2_Goal2_UpperLegLanding", "目標2: 上の脚を地面まで振り下ろす", 0.9, support2, landing=land2, landing_targets=targets2, support_dx=md/2*0.9, support_dpitch=math.pi/4*0.9))
        # Goal 3.
        support3 = {1: [1,3,4,6], 5: [0,2,5,7], 6: [4,6,1,3], 2: [0,2,5,7]}[surface]
        land3 = {1: [0,2], 5: [4,6], 6: [5,7], 2: [1,3]}[surface]
        targets3 = self._pair_targets(land3, x_state + 0.2, sd/2, 0.05)
        steps.append(self._step("RF-3_Goal3_LiftMiddlePair", "目標3: 接地6脚中の真ん中2脚を振り上げる", 0.1, support3, landing=land3, landing_targets=targets3))
        # Goal 4.
        targets4 = self._pair_targets(land3, x_state + 0.05, sd/2, 0.0)
        steps.append(self._step("RF-4_Goal4_LandMiddlePair", "目標4: 振り上げた2脚を接地させる", 0.1, support3, landing=land3, landing_targets=targets4))
        # Goal 5.
        support5 = {1: [0,2,4,6], 5: [4,6,5,7], 6: [5,7,1,3], 2: [1,3,0,2]}[surface]
        steps.append(self._step("RF-5_Goal5_MainBodyRoll", "目標5: 後ろ2脚を半分まで振り上げながら胴体を回す", 1.0, support5, support_dx=md/2, support_dpitch=math.pi/4))
        # Stop adjustment.
        steps.append(self._step("RF-6_StopAdjustment", "停止後の調整用: 姿勢差ゼロでIKを再計算", 0.1, support5))
        return steps

    def _step(self, name, description, rate, support, landing=None, landing_targets=None,
              swing=None, swing_types=None, support_dx=0.0, support_dz=0.0, support_dpitch=0.0):
        swing_targets = {}
        if swing and swing_types:
            for lid, typ in zip(swing, swing_types):
                swing_targets[int(lid)] = self._swing_type_to_deg(typ)
        return {
            "name": name,
            "description": description,
            "rate": float(rate),
            "support": list(support or []),
            "landing": list(landing or []),
            "landing_targets": dict(landing_targets or {}),
            "swing": list(swing or []),
            "swing_targets_deg": swing_targets,
            "support_dx": float(support_dx),
            "support_dz": float(support_dz),
            "support_dpitch": float(support_dpitch),
            "kind": "lift_middle" if "Goal3" in name else ("land_middle" if "Goal4" in name else "generic"),
        }

    def _pair_targets(self, legacy_ids, x, half_y, z):
        out = {}
        if len(legacy_ids) >= 1:
            out[int(legacy_ids[0])] = [float(x), float(half_y), float(z)]
        if len(legacy_ids) >= 2:
            out[int(legacy_ids[1])] = [float(x), -float(half_y), float(z)]
        return out

    def _swing_type_to_deg(self, typ):
        DEG2 = 60.0
        DEG3 = 120.0
        if typ == 0:
            return [0.0, DEG2, -DEG3]
        if typ == 1:
            return [0.0, -DEG2, DEG3]
        if typ == 3:
            return [-180.0, DEG2, -DEG3]
        if typ == 4:
            return [180.0, DEG2, -DEG3]
        if typ == 5:
            return [180.0, -DEG2, DEG3]
        if typ == 6:
            return [-180.0, -DEG2, DEG3]
        # -1 means normalize theta1 by +/-180 in the old code; keep current.
        return [0.0, 0.0, 0.0]

    def _roles_for_step(self, step):
        roles = dict((m.leg_id, R.OTHER) for m in self.robot_model.mounts)
        for lid in self._lids(step.get("support", [])):
            roles[lid] = R.SUPPORT
        for lid in self._lids(step.get("landing", [])):
            roles[lid] = R.CANDIDATE_SUPPORT
        for lid in self._lids(step.get("swing", [])):
            roles[lid] = R.CLEARANCE
        return roles

    def _ground_diag(self, joint_map, base_pose, phase_name, phase_step_index, frame_index):
        min_clearance = None
        penetrating = False
        worst = None
        for lid, q in joint_map.items():
            pts = self.robot_model.link_positions_world(lid, q, base_pose)
            for point_name, p in pts.items():
                c = p[2] - self.config.ground_z
                if min_clearance is None or c < min_clearance:
                    min_clearance = c
                    worst = {"leg_id": lid, "leg_name": self.robot_model.leg_name(lid), "point": point_name, "clearance_m": c, "position_world": list(p)}
                if c < -1e-9:
                    penetrating = True
        return {"phase_name": phase_name, "phase_step_index": phase_step_index, "frame_index": frame_index, "min_clearance_m": min_clearance, "penetrating": penetrating, "worst_point": worst}

    def _copy_joint_map(self, m):
        return dict((int(k), list(v)) for k, v in m.items())

    def _copy_targets(self, m):
        return dict((int(k), list(v)) for k, v in m.items())

    def _angle_delta(self, a, b):
        return (float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi
