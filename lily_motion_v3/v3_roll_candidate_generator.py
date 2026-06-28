# -*- coding: utf-8 -*-
"""Project-contained v3 one-roll candidate generator.

v3.0.3 adds an explicit base-pose trajectory.  During ConstrainedBodyRoll the
body pitch is changed while foot targets are represented in world coordinates;
active support feet therefore remain fixed in world and are converted back into
body-frame IK targets at each frame.  This is still a concept generator, not a
final gait.
"""
from __future__ import division
import math

from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.v3_roll_concept_generator import build_forward_roll_concept
from lily_motion_v3.roll_candidate import V3MotionFrame, V3RollCandidate
from lily_motion_v3.motion_evaluation_report import MotionEvaluationReport
from lily_motion_v3.role_utils import roles_from_contact_state
from lily_motion_v3 import leg_role as R
from lily_motion_v3.transforms import vec_add, angle_delta
from lily_motion_v3.foot_target_candidate import CandidateFootTargetConfig, CandidateFootTargetGenerator
from lily_motion_v3.geometry import segment_segment_distance


class V3RollGenerationConfig(object):
    def __init__(self, steps_per_phase=8, lift_height=0.08,
                 clearance_height=0.06, candidate_support_shift_x=0.04,
                 candidate_support_drop_z=-0.02,
                 body_roll_pitch_rad=math.pi / 2.0,
                 body_roll_x_shift=0.0,
                 body_roll_z_shift=0.0,
                 enable_body_roll_pose_search=True,
                 body_roll_search_x_offsets=None,
                 body_roll_search_z_offsets=None,
                 max_joint_delta_deg=120.0,
                 min_inter_leg_clearance_m=0.05,
                 min_target_point_clearance_m=0.04,
                 ground_z=0.0, auto_align_initial_ground=True,
                 enable_contact_lock_generation=True,
                 contact_plan_variant="default"):
        self.steps_per_phase = int(steps_per_phase)
        self.lift_height = float(lift_height)
        self.clearance_height = float(clearance_height)
        self.candidate_support_shift_x = float(candidate_support_shift_x)
        self.candidate_support_drop_z = float(candidate_support_drop_z)
        self.body_roll_pitch_rad = float(body_roll_pitch_rad)
        self.body_roll_x_shift = float(body_roll_x_shift)
        self.body_roll_z_shift = float(body_roll_z_shift)
        self.enable_body_roll_pose_search = bool(enable_body_roll_pose_search)
        self.body_roll_search_x_offsets = list(body_roll_search_x_offsets or [-0.20, -0.10, 0.0, 0.10, 0.20])
        self.body_roll_search_z_offsets = list(body_roll_search_z_offsets or [-0.10, 0.0, 0.10, 0.20, 0.30, 0.40])
        self.max_joint_delta_deg = float(max_joint_delta_deg)
        self.min_inter_leg_clearance_m = float(min_inter_leg_clearance_m)
        self.min_target_point_clearance_m = float(min_target_point_clearance_m)
        self.ground_z = float(ground_z)
        self.auto_align_initial_ground = bool(auto_align_initial_ground)
        self.enable_contact_lock_generation = bool(enable_contact_lock_generation)
        self.contact_plan_variant = str(contact_plan_variant or "default")


class V3RollCandidateGenerator(object):
    def __init__(self, robot_model=None, config=None):
        self.robot_model = robot_model or RobotModel()
        self.config = config or V3RollGenerationConfig()
        self.foot_target_generator = CandidateFootTargetGenerator(
            self.robot_model,
            CandidateFootTargetConfig(
                lift_height=self.config.lift_height,
                clearance_height=self.config.clearance_height,
                candidate_support_shift_x=self.config.candidate_support_shift_x,
                candidate_support_drop_z=self.config.candidate_support_drop_z,
                min_point_clearance_m=self.config.min_target_point_clearance_m,
            )
        )

    def default_initial_joint_map(self):
        """Return a conservative folded-but-reachable default pose for all legs."""
        return dict((m.leg_id, [0.0, -0.45, 0.90]) for m in self.robot_model.mounts)

    def generate_forward_one_roll(self, initial_joint_map=None, surface_id=1):
        phases = build_forward_roll_concept(surface_id, self.config.contact_plan_variant)
        joint_map = self._copy_joint_map(initial_joint_map or self.default_initial_joint_map())
        previous_joint_map = self._copy_joint_map(joint_map)
        self._roll_base_z0 = self._compute_initial_base_z0(joint_map)
        self._post_roll_base_pose = None
        self._active_contact_locks = {}
        initial_base_pose = self._base_pose_at_roll_progress(0.0)
        foot_targets_world = dict(
            (leg_id, self.robot_model.foot_position_world(leg_id, q, initial_base_pose))
            for leg_id, q in joint_map.items())

        frames = []
        frame_index = 0
        report = MotionEvaluationReport()
        ik_failure_count = 0
        max_second_joint_deg = 0.0
        max_joint_delta_deg = 0.0
        discontinuity_records = []
        ik_failure_records = []
        target_selection_records = []
        target_selection_failure_records = []
        max_candidate_count = 0
        min_inter_leg_distance = None
        inter_leg_near_records = []
        min_ground_clearance = None
        ground_penetration_records = []
        max_abs_base_pitch_deg = 0.0
        base_pose_search_records = []
        base_pose_search_failure_count = 0
        final_surface_id = surface_id

        for phase_index, phase in enumerate(phases):
            step_count = max(1, self.config.steps_per_phase)
            phase_start_world_targets = self._copy_targets(foot_targets_world)
            phase_start_base_pose = self._base_pose_for_phase(phase.name, 0.0)
            phase_goal_world_targets = self._phase_goal_world_targets(
                phase, phase_start_world_targets, phase_start_base_pose)

            for phase_step_index in range(step_count):
                alpha = 1.0 if step_count == 1 else float(phase_step_index) / float(step_count - 1)
                leg_roles = roles_from_contact_state(
                    phase.contact_state,
                    [m.leg_id for m in self.robot_model.mounts]
                )
                nominal_world_targets = self._interpolate_targets(
                    phase_start_world_targets, phase_goal_world_targets, alpha)
                nominal_world_targets, contact_lock_diag = self._apply_contact_locks(
                    nominal_world_targets, previous_joint_map, leg_roles,
                    phase.name, phase_step_index, frame_index)
                base_pose, pose_search_diag = self._choose_base_pose_for_frame(
                    phase.name, alpha, nominal_world_targets, previous_joint_map, leg_roles,
                    phase_step_index, frame_index)
                base_pose_search_records.append(pose_search_diag)
                if not pose_search_diag.get("selected_feasible", False):
                    base_pose_search_failure_count += 1
                pitch_deg = abs(math.degrees(base_pose.get("pitch", 0.0)))
                if pitch_deg > max_abs_base_pitch_deg:
                    max_abs_base_pitch_deg = pitch_deg
                nominal_body_targets = self._world_targets_to_body(nominal_world_targets, base_pose)
                frame_body_targets, selection_diag = self._select_frame_targets(
                    nominal_body_targets, previous_joint_map, leg_roles,
                    phase.name, phase_step_index, frame_index)
                frame_world_targets = self._body_targets_to_world(frame_body_targets, base_pose)
                selection_diag["base_pose_search"] = pose_search_diag

                target_selection_records.extend(selection_diag["selection_records"])
                target_selection_failure_records.extend(selection_diag["failure_records"])
                if selection_diag["max_candidate_count"] > max_candidate_count:
                    max_candidate_count = selection_diag["max_candidate_count"]

                frame_diag = {"ik_failures": [], "target_selection": selection_diag, "contact_lock_generation": contact_lock_diag}
                new_joint_map = {}
                for leg_id in sorted(frame_body_targets.keys()):
                    selected = self.robot_model.select_ik_body(
                        leg_id,
                        frame_body_targets[leg_id],
                        previous_q=previous_joint_map.get(leg_id)
                    )
                    if selected is None:
                        ik_failure_count += 1
                        rec = {
                            "frame_index": frame_index,
                            "phase_name": phase.name,
                            "phase_step_index": phase_step_index,
                            "leg_id": leg_id,
                            "leg_name": self.robot_model.leg_name(leg_id),
                            "role": leg_roles.get(leg_id, R.OTHER),
                            "target_body": list(frame_body_targets[leg_id]),
                            "target_world": list(frame_world_targets[leg_id]),
                            "base_pose": dict(base_pose),
                        }
                        ik_failure_records.append(rec)
                        frame_diag["ik_failures"].append(rec)
                        new_joint_map[leg_id] = list(previous_joint_map[leg_id])
                        continue
                    q = list(selected.q)
                    new_joint_map[leg_id] = q
                    second_deg = abs(math.degrees(q[1]))
                    if second_deg > max_second_joint_deg:
                        max_second_joint_deg = second_deg
                    prev_q = previous_joint_map.get(leg_id)
                    if prev_q is not None:
                        deltas = [abs(math.degrees(angle_delta(q[i], prev_q[i]))) for i in range(3)]
                        local_max = max(deltas)
                        if local_max > max_joint_delta_deg:
                            max_joint_delta_deg = local_max
                        if local_max > self.config.max_joint_delta_deg:
                            discontinuity_records.append({
                                "frame_index": frame_index,
                                "phase_name": phase.name,
                                "phase_step_index": phase_step_index,
                                "leg_id": leg_id,
                                "leg_name": self.robot_model.leg_name(leg_id),
                                "role": leg_roles.get(leg_id, R.OTHER),
                                "delta_deg": deltas,
                                "max_delta_deg": local_max,
                                "base_pose": dict(base_pose),
                            })

                previous_joint_map = self._copy_joint_map(new_joint_map)
                foot_targets_world = frame_world_targets
                if phase.name == "ConstrainedBodyRoll":
                    # v3.0.5: Preserve the actual selected terminal base pose.
                    # Earlier versions recomputed SupportTransfer/PostureNormalization
                    # from nominal progress=1.0, which reset x/z and broke the
                    # world/body consistency achieved by the body-roll pose search.
                    self._post_roll_base_pose = dict(base_pose)

                inter_leg_diag = self._evaluate_inter_leg_clearance(
                    new_joint_map, leg_roles, phase.name, phase_step_index, frame_index, base_pose)
                frame_diag["inter_leg_clearance"] = inter_leg_diag
                if inter_leg_diag["min_distance_m"] is not None:
                    if min_inter_leg_distance is None or inter_leg_diag["min_distance_m"] < min_inter_leg_distance:
                        min_inter_leg_distance = inter_leg_diag["min_distance_m"]
                    if inter_leg_diag["below_threshold"]:
                        inter_leg_near_records.append(inter_leg_diag)

                ground_diag = self._evaluate_ground_clearance(
                    new_joint_map, leg_roles, phase.name, phase_step_index, frame_index, base_pose)
                frame_diag["ground_clearance"] = ground_diag
                if ground_diag["min_clearance_m"] is not None:
                    if min_ground_clearance is None or ground_diag["min_clearance_m"] < min_ground_clearance:
                        min_ground_clearance = ground_diag["min_clearance_m"]
                    if ground_diag["penetrating"]:
                        ground_penetration_records.append(ground_diag)

                frames.append(V3MotionFrame(
                    frame_index=frame_index,
                    phase_index=phase_index,
                    phase_name=phase.name,
                    phase_step_index=phase_step_index,
                    phase_step_count=step_count,
                    contact_state=phase.contact_state,
                    base_pose=base_pose,
                    leg_roles=leg_roles,
                    foot_targets_body=frame_body_targets,
                    foot_targets_world=frame_world_targets,
                    joint_angles=new_joint_map,
                    diagnostics=frame_diag,
                ))
                frame_index += 1

            if phase.name in ("SupportTransfer", "PostureNormalization"):
                final_surface_id = self._next_forward_surface(surface_id)

        report.task_success = {
            "completed": ik_failure_count == 0 and len(ground_penetration_records) == 0 and base_pose_search_failure_count == 0,
            "direction": "forward",
            "surface_start": surface_id,
            "surface_after": final_surface_id,
            "planned_phase_count": len(phases),
            "frame_count": len(frames),
            "legacy_dependency": False,
            "base_pose_enabled": True,
            "auto_align_initial_ground": self.config.auto_align_initial_ground,
            "initial_base_z": self._roll_base_z0,
            "max_abs_base_pitch_deg": max_abs_base_pitch_deg,
            "base_pose_search_enabled": self.config.enable_body_roll_pose_search,
            "base_pose_search_failure_count": base_pose_search_failure_count,
            "contact_lock_generation_enabled": self.config.enable_contact_lock_generation,
            "contact_plan_variant": self.config.contact_plan_variant,
        }
        report.joint_limit = {
            "second_joint_abs_max_deg": self.robot_model.leg_config.second_joint_abs_max_deg,
            "max_abs_second_joint_deg": max_second_joint_deg,
            "second_joint_limit_ok": max_second_joint_deg <= self.robot_model.leg_config.second_joint_abs_max_deg + 1e-9,
        }
        report.motion_discontinuity = {
            "max_joint_delta_deg": max_joint_delta_deg,
            "warn_threshold_deg": self.config.max_joint_delta_deg,
            "discontinuity_count": len(discontinuity_records),
            "top_records": discontinuity_records[:20],
        }
        report.ik_reachability = {
            "ik_failure_count": ik_failure_count,
            "top_failure_records": ik_failure_records[:20],
        }
        report.ground_clearance = {
            "ground_z": self.config.ground_z,
            "min_clearance_m": min_ground_clearance,
            "penetration_count": len(ground_penetration_records),
            "top_penetration_records": ground_penetration_records[:20],
        }
        report.inter_leg_clearance = {
            "threshold_m": self.config.min_inter_leg_clearance_m,
            "min_distance_m": min_inter_leg_distance,
            "near_count": len(inter_leg_near_records),
            "top_near_records": inter_leg_near_records[:20],
        }
        report.base_pose_search = {
            "enabled": self.config.enable_body_roll_pose_search,
            "failure_count": base_pose_search_failure_count,
            "top_records": base_pose_search_records[:20],
            "top_failure_records": [r for r in base_pose_search_records if not r.get("selected_feasible", False)][:20],
        }
        report.support_consistency = {
            "target_selection_failure_count": len(target_selection_failure_records),
            "max_candidate_count_per_leg": max_candidate_count,
            "top_selection_failures": target_selection_failure_records[:20],
            "top_selection_records": target_selection_records[:20],
        }
        report.notes.append("v3.0.9: support-foot contact locks are now applied during generation; SUPPORT foot_world targets remain fixed until release.")
        return V3RollCandidate("forward", phases, frames, report)


    def _apply_contact_locks(self, nominal_world_targets, previous_joint_map, leg_roles,
                             phase_name, phase_step_index, frame_index):
        """Override SUPPORT foot targets with persistent world contact locks.

        A lock is created when a leg first enters SUPPORT and released when it
        leaves SUPPORT.  The generator still may fail IK later; the point is
        that the target itself no longer drifts silently frame by frame.
        """
        if not self.config.enable_contact_lock_generation:
            return self._copy_targets(nominal_world_targets), {
                "enabled": False,
                "active_locks": {},
                "created_locks": [],
                "released_locks": [],
            }
        out = self._copy_targets(nominal_world_targets)
        active_support = set(int(k) for k, role in leg_roles.items() if role == R.SUPPORT)
        created = []
        released = []
        for leg_id in list(self._active_contact_locks.keys()):
            if leg_id not in active_support:
                released.append({
                    "leg_id": leg_id,
                    "leg_name": self.robot_model.leg_name(leg_id),
                    "lock_point_world": list(self._active_contact_locks[leg_id]["lock_point_world"]),
                    "release_frame_index": frame_index,
                    "release_phase_name": phase_name,
                    "release_phase_step_index": phase_step_index,
                })
                del self._active_contact_locks[leg_id]
        for leg_id in sorted(active_support):
            if leg_id not in self._active_contact_locks:
                self._active_contact_locks[leg_id] = {
                    "lock_point_world": list(out[leg_id]),
                    "start_frame_index": frame_index,
                    "start_phase_name": phase_name,
                    "start_phase_step_index": phase_step_index,
                }
                created.append({
                    "leg_id": leg_id,
                    "leg_name": self.robot_model.leg_name(leg_id),
                    "lock_point_world": list(out[leg_id]),
                    "start_frame_index": frame_index,
                    "start_phase_name": phase_name,
                    "start_phase_step_index": phase_step_index,
                })
            out[leg_id] = list(self._active_contact_locks[leg_id]["lock_point_world"])
        return out, {
            "enabled": True,
            "created_locks": created,
            "released_locks": released,
            "active_locks": dict((str(k), {
                "lock_point_world": list(v["lock_point_world"]),
                "start_frame_index": v["start_frame_index"],
                "start_phase_name": v["start_phase_name"],
                "start_phase_step_index": v["start_phase_step_index"],
            }) for k, v in sorted(self._active_contact_locks.items())),
        }


    def _choose_base_pose_for_frame(self, phase_name, alpha, nominal_world_targets, previous_joint_map,
                                    leg_roles, phase_step_index, frame_index):
        nominal_pose = self._base_pose_for_phase(phase_name, alpha)
        if phase_name != "ConstrainedBodyRoll" or not self.config.enable_body_roll_pose_search:
            diag = self._score_base_pose(nominal_pose, nominal_world_targets, previous_joint_map, leg_roles,
                                         phase_name, phase_step_index, frame_index, candidate_index=0)
            diag["candidate_count"] = 1
            diag["selected_pose"] = dict(nominal_pose)
            diag["selected_feasible"] = diag.get("feasible", False)
            return nominal_pose, diag

        candidates = []
        idx = 0
        for dx in self.config.body_roll_search_x_offsets:
            for dz in self.config.body_roll_search_z_offsets:
                pose = dict(nominal_pose)
                pose["x"] = nominal_pose.get("x", 0.0) + float(dx)
                pose["z"] = nominal_pose.get("z", 0.0) + float(dz)
                rec = self._score_base_pose(pose, nominal_world_targets, previous_joint_map, leg_roles,
                                            phase_name, phase_step_index, frame_index, candidate_index=idx)
                candidates.append(rec)
                idx += 1
        candidates.sort(key=lambda r: r["score"])
        best = candidates[0] if candidates else self._score_base_pose(nominal_pose, nominal_world_targets, previous_joint_map, leg_roles,
                                                                      phase_name, phase_step_index, frame_index, candidate_index=0)
        out = dict(best)
        out["candidate_count"] = len(candidates)
        out["selected_pose"] = dict(best["base_pose"])
        out["selected_feasible"] = best.get("feasible", False)
        out["top_candidates"] = candidates[:5]
        return dict(best["base_pose"]), out

    def _score_base_pose(self, base_pose, nominal_world_targets, previous_joint_map, leg_roles,
                         phase_name, phase_step_index, frame_index, candidate_index=0):
        body_targets = self._world_targets_to_body(nominal_world_targets, base_pose)
        ik_failures = 0
        second_joint_violation = 0
        max_second_joint_deg = 0.0
        max_joint_delta_deg = 0.0
        joint_map = {}
        for leg_id in sorted(body_targets.keys()):
            selected = self.robot_model.select_ik_body(leg_id, body_targets[leg_id], previous_q=previous_joint_map.get(leg_id))
            if selected is None:
                ik_failures += 1
                joint_map[leg_id] = list(previous_joint_map[leg_id])
                continue
            q = list(selected.q)
            joint_map[leg_id] = q
            second_deg = abs(math.degrees(q[1]))
            if second_deg > max_second_joint_deg:
                max_second_joint_deg = second_deg
            if second_deg > self.robot_model.leg_config.second_joint_abs_max_deg:
                second_joint_violation += 1
            prev_q = previous_joint_map.get(leg_id)
            if prev_q is not None:
                local_max = max(abs(math.degrees(angle_delta(q[i], prev_q[i]))) for i in range(3))
                if local_max > max_joint_delta_deg:
                    max_joint_delta_deg = local_max

        min_clearance = None
        penetration_count = 0
        for leg_id, q in joint_map.items():
            pts = self.robot_model.link_positions_world(leg_id, q, base_pose)
            for p in pts.values():
                c = p[2] - self.config.ground_z
                if min_clearance is None or c < min_clearance:
                    min_clearance = c
                if c < -1e-9:
                    penetration_count += 1

        min_inter_leg_distance = None
        segments = []
        for leg_id, q in joint_map.items():
            segments.extend(self.robot_model.leg_segments_world(leg_id, q, base_pose))
        for i in range(len(segments)):
            for j in range(i + 1, len(segments)):
                if segments[i]["leg_id"] == segments[j]["leg_id"]:
                    continue
                d, _, _ = segment_segment_distance(segments[i]["a"], segments[i]["b"], segments[j]["a"], segments[j]["b"])
                if min_inter_leg_distance is None or d < min_inter_leg_distance:
                    min_inter_leg_distance = d
        inter_leg_violation = 0
        if min_inter_leg_distance is not None and min_inter_leg_distance < self.config.min_inter_leg_clearance_m:
            inter_leg_violation = 1

        # Cost is intentionally lexicographic-like: feasibility first, smoothness second.
        clearance_penalty = 0.0 if min_clearance is None else max(0.0, -min_clearance)
        inter_leg_penalty = 0.0 if min_inter_leg_distance is None else max(0.0, self.config.min_inter_leg_clearance_m - min_inter_leg_distance)
        score = (ik_failures * 100000.0 +
                 penetration_count * 20000.0 +
                 second_joint_violation * 10000.0 +
                 inter_leg_violation * 5000.0 +
                 clearance_penalty * 10000.0 +
                 inter_leg_penalty * 10000.0 +
                 max_joint_delta_deg * 2.0 +
                 abs(base_pose.get("x", 0.0) - self._base_pose_for_phase(phase_name, 0.0).get("x", 0.0)) * 10.0 +
                 abs(base_pose.get("z", 0.0) - getattr(self, "_roll_base_z0", 0.0)) * 10.0)
        feasible = (ik_failures == 0 and penetration_count == 0 and second_joint_violation == 0 and inter_leg_violation == 0)
        return {
            "frame_index": frame_index,
            "phase_name": phase_name,
            "phase_step_index": phase_step_index,
            "candidate_index": candidate_index,
            "base_pose": dict(base_pose),
            "score": score,
            "feasible": feasible,
            "ik_failures": ik_failures,
            "penetration_count": penetration_count,
            "min_clearance_m": min_clearance,
            "second_joint_violation_count": second_joint_violation,
            "max_abs_second_joint_deg": max_second_joint_deg,
            "max_joint_delta_deg": max_joint_delta_deg,
            "min_inter_leg_distance_m": min_inter_leg_distance,
            "inter_leg_violation": bool(inter_leg_violation),
        }

    def _phase_goal_world_targets(self, phase, start_world_targets, base_pose):
        # During body roll, support targets are intentionally fixed in world.
        if phase.name == "ConstrainedBodyRoll":
            return self._copy_targets(start_world_targets)
        start_body = self._world_targets_to_body(start_world_targets, base_pose)
        goal_body = self._phase_goal_body_targets(phase, start_body)
        return self._body_targets_to_world(goal_body, base_pose)

    def _phase_goal_body_targets(self, phase, start_targets):
        out = self._copy_targets(start_targets)
        cs = phase.contact_state
        for leg_id in cs.clearance_legs:
            out[int(leg_id)] = vec_add(out[int(leg_id)], [0.0, 0.0, self.config.clearance_height])
        for leg_id in cs.candidate_support_legs:
            sign = 1.0 if int(leg_id) % 2 == 0 else -1.0
            out[int(leg_id)] = vec_add(out[int(leg_id)], [self.config.candidate_support_shift_x * sign, 0.0, self.config.candidate_support_drop_z])
        for leg_id in cs.lift_legs:
            out[int(leg_id)] = vec_add(out[int(leg_id)], [0.0, 0.0, self.config.lift_height])
        return out

    def _select_frame_targets(self, nominal_targets, previous_joint_map, leg_roles,
                              phase_name, phase_step_index, frame_index):
        out = {}
        selection_records = []
        failure_records = []
        max_candidate_count = 0
        for leg_id in sorted(nominal_targets.keys()):
            role = leg_roles.get(leg_id, R.OTHER)
            best, evaluated = self.foot_target_generator.choose_target(
                leg_id,
                role,
                nominal_targets[leg_id],
                previous_q=previous_joint_map.get(leg_id),
                other_current_targets=nominal_targets,
            )
            max_candidate_count = max(max_candidate_count, len(evaluated))
            if best is None:
                out[leg_id] = list(nominal_targets[leg_id])
                rec = {
                    "frame_index": frame_index,
                    "phase_name": phase_name,
                    "phase_step_index": phase_step_index,
                    "leg_id": leg_id,
                    "leg_name": self.robot_model.leg_name(leg_id),
                    "role": role,
                    "candidate_count": len(evaluated),
                    "nominal_target": list(nominal_targets[leg_id]),
                    "first_candidates": [c.to_dict() for c in evaluated[:5]],
                }
                failure_records.append(rec)
                selection_records.append(rec)
            else:
                out[leg_id] = list(best.target)
                selection_records.append({
                    "frame_index": frame_index,
                    "phase_name": phase_name,
                    "phase_step_index": phase_step_index,
                    "leg_id": leg_id,
                    "leg_name": self.robot_model.leg_name(leg_id),
                    "role": role,
                    "candidate_count": len(evaluated),
                    "selected_reason": best.reason,
                    "selected_target": list(best.target),
                    "selected_score": best.score,
                    "selected_metrics": dict(best.metrics),
                })
        return out, {
            "selection_records": selection_records,
            "failure_records": failure_records,
            "max_candidate_count": max_candidate_count,
        }

    def _evaluate_inter_leg_clearance(self, joint_map, leg_roles, phase_name,
                                      phase_step_index, frame_index, base_pose):
        segments = []
        for leg_id, q in joint_map.items():
            segments.extend(self.robot_model.leg_segments_world(leg_id, q, base_pose))
        min_rec = None
        for i in range(len(segments)):
            for j in range(i + 1, len(segments)):
                a = segments[i]
                b = segments[j]
                if a["leg_id"] == b["leg_id"]:
                    continue
                d, ca, cb = segment_segment_distance(a["a"], a["b"], b["a"], b["b"])
                if min_rec is None or d < min_rec["distance_m"]:
                    min_rec = {
                        "distance_m": d,
                        "closest_point_a": ca,
                        "closest_point_b": cb,
                        "segment_a": {"leg_id": a["leg_id"], "leg_name": a["leg_name"], "segment_name": a["segment_name"], "role": leg_roles.get(a["leg_id"], R.OTHER)},
                        "segment_b": {"leg_id": b["leg_id"], "leg_name": b["leg_name"], "segment_name": b["segment_name"], "role": leg_roles.get(b["leg_id"], R.OTHER)},
                    }
        if min_rec is None:
            return {
                "frame_index": frame_index,
                "phase_name": phase_name,
                "phase_step_index": phase_step_index,
                "threshold_m": self.config.min_inter_leg_clearance_m,
                "min_distance_m": None,
                "below_threshold": False,
            }
        min_rec.update({
            "frame_index": frame_index,
            "phase_name": phase_name,
            "phase_step_index": phase_step_index,
            "threshold_m": self.config.min_inter_leg_clearance_m,
            "min_distance_m": min_rec["distance_m"],
            "clearance_margin_m": min_rec["distance_m"] - self.config.min_inter_leg_clearance_m,
            "below_threshold": min_rec["distance_m"] < self.config.min_inter_leg_clearance_m,
        })
        return min_rec

    def _evaluate_ground_clearance(self, joint_map, leg_roles, phase_name,
                                   phase_step_index, frame_index, base_pose):
        min_rec = None
        for leg_id, q in joint_map.items():
            pts = self.robot_model.link_positions_world(leg_id, q, base_pose)
            # Check representative joint points.  This is still a coarse geometric
            # proxy, not a Gazebo collision mesh check.
            for point_name, p in pts.items():
                clearance = p[2] - self.config.ground_z
                if min_rec is None or clearance < min_rec["clearance_m"]:
                    min_rec = {
                        "clearance_m": clearance,
                        "point_world": list(p),
                        "point_name": point_name,
                        "leg_id": int(leg_id),
                        "leg_name": self.robot_model.leg_name(leg_id),
                        "role": leg_roles.get(int(leg_id), R.OTHER),
                    }
        if min_rec is None:
            return {"frame_index": frame_index, "phase_name": phase_name, "phase_step_index": phase_step_index, "min_clearance_m": None, "penetrating": False}
        min_rec.update({
            "frame_index": frame_index,
            "phase_name": phase_name,
            "phase_step_index": phase_step_index,
            "ground_z": self.config.ground_z,
            "min_clearance_m": min_rec["clearance_m"],
            "penetrating": min_rec["clearance_m"] < -1e-9,
        })
        return min_rec

    def _base_pose_for_phase(self, phase_name, alpha):
        if phase_name == "ConstrainedBodyRoll":
            progress = alpha
        elif phase_name in ("SupportTransfer", "PostureNormalization"):
            if getattr(self, "_post_roll_base_pose", None) is not None:
                return dict(self._post_roll_base_pose)
            progress = 1.0
        else:
            progress = 0.0
        return self._base_pose_at_roll_progress(progress)


    def _compute_initial_base_z0(self, joint_map):
        if not self.config.auto_align_initial_ground:
            return 0.0
        min_z = None
        zero_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        for leg_id, q in joint_map.items():
            pts = self.robot_model.link_positions_world(leg_id, q, zero_pose)
            for p in pts.values():
                if min_z is None or p[2] < min_z:
                    min_z = p[2]
        if min_z is None:
            return 0.0
        return self.config.ground_z - min_z

    def _base_pose_at_roll_progress(self, progress):
        return {
            "x": self.config.body_roll_x_shift * progress,
            "y": 0.0,
            "z": getattr(self, "_roll_base_z0", 0.0) + self.config.body_roll_z_shift * progress,
            "roll": 0.0,
            "pitch": self.config.body_roll_pitch_rad * progress,
            "yaw": 0.0,
        }

    @staticmethod
    def _next_forward_surface(surface_id):
        # v2 observed forward sequence begins 1 -> 5 -> 6 -> 2 -> 1.
        seq = [1, 5, 6, 2]
        if surface_id in seq:
            return seq[(seq.index(surface_id) + 1) % len(seq)]
        return surface_id

    def _world_targets_to_body(self, targets_world, base_pose):
        return dict((int(k), self.robot_model.world_point_to_body(v, base_pose))
                    for k, v in targets_world.items())

    def _body_targets_to_world(self, targets_body, base_pose):
        return dict((int(k), self.robot_model.body_point_to_world(v, base_pose))
                    for k, v in targets_body.items())

    @staticmethod
    def _interpolate_targets(a, b, alpha):
        out = {}
        for k in a.keys():
            out[k] = [a[k][i] + (b[k][i] - a[k][i]) * alpha for i in range(3)]
        return out

    @staticmethod
    def _copy_targets(targets):
        return dict((int(k), list(v)) for k, v in targets.items())

    @staticmethod
    def _copy_joint_map(joint_map):
        return dict((int(k), list(v)) for k, v in joint_map.items())
