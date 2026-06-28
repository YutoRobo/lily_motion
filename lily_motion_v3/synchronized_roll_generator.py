# -*- coding: utf-8 -*-
"""Progress-synchronized v3 roll generator.

This generator is intentionally different from the older phase-local generator:
all quantities are driven by one roll progress variable s in [0, 1].  Body
pitch, support roles, lift trajectories, and candidate-support placement are
therefore evaluated at every step together.  It is still a search scaffold, not
a final gait.
"""
from __future__ import division
import math

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.v3_roll_concept_generator import build_forward_roll_concept, _variant_sets
from lily_motion_v3.roll_candidate import V3MotionFrame, V3RollCandidate
from lily_motion_v3.motion_evaluation_report import MotionEvaluationReport
from lily_motion_v3.contact_state import ContactState
from lily_motion_v3 import leg_role as R
from lily_motion_v3.transforms import vec_add, angle_delta


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def smoothstep(x):
    x = clamp(x)
    return x * x * (3.0 - 2.0 * x)


class SynchronizedRollGenerationConfig(V3RollGenerationConfig):
    def __init__(self, synchronized_steps=72, roll_start_s=0.35, roll_end_s=0.85, *args, **kwargs):
        V3RollGenerationConfig.__init__(self, *args, **kwargs)
        self.synchronized_steps = int(synchronized_steps)
        self.roll_start_s = float(roll_start_s)
        self.roll_end_s = float(roll_end_s)


class SynchronizedRollCandidateGenerator(V3RollCandidateGenerator):
    """Generate one roll with a single progress variable s.

    Design intent:
      - support roles and body pitch are synchronized by s;
      - SUPPORT legs still create persistent contact locks;
      - non-support legs receive time-varying lift/clearance/landing targets;
      - body pose is searched at each roll-progress frame, not only inside a
        hard-coded phase local alpha.
    """

    def __init__(self, robot_model=None, config=None):
        if config is None:
            config = SynchronizedRollGenerationConfig()
        V3RollCandidateGenerator.__init__(self, robot_model=robot_model, config=config)
        if not hasattr(self.config, "synchronized_steps"):
            self.config.synchronized_steps = max(24, self.config.steps_per_phase * 7)
            self.config.roll_start_s = 0.35
            self.config.roll_end_s = 0.85

    def generate_forward_one_roll(self, initial_joint_map=None, surface_id=1):
        phases = build_forward_roll_concept(surface_id, self.config.contact_plan_variant)
        sets = _variant_sets(self.config.contact_plan_variant)
        joint_map = self._copy_joint_map(initial_joint_map or self.default_initial_joint_map())
        previous_joint_map = self._copy_joint_map(joint_map)
        self._roll_base_z0 = self._compute_initial_base_z0(joint_map)
        self._post_roll_base_pose = None
        self._active_contact_locks = {}

        zero_pose = self._base_pose_at_roll_progress(0.0)
        initial_targets_world = dict(
            (leg_id, self.robot_model.foot_position_world(leg_id, q, zero_pose))
            for leg_id, q in joint_map.items())
        current_targets_world = self._copy_targets(initial_targets_world)
        landing_targets_world = self._compute_landing_targets(initial_targets_world, sets)

        frames = []
        report = MotionEvaluationReport()
        ik_failure_count = 0
        ik_failure_records = []
        max_second_joint_deg = 0.0
        max_joint_delta_deg = 0.0
        discontinuity_records = []
        ground_penetration_records = []
        inter_leg_near_records = []
        base_pose_search_records = []
        base_pose_search_failure_count = 0
        min_ground_clearance = None
        min_inter_leg_distance = None
        target_selection_records = []
        target_selection_failure_records = []
        max_candidate_count = 0
        max_abs_base_pitch_deg = 0.0

        n = max(2, int(self.config.synchronized_steps))
        for frame_index in range(n):
            s = 0.0 if n == 1 else float(frame_index) / float(n - 1)
            phase_name, phase_step_index, phase_step_count = self._phase_from_s(s, n)
            leg_roles = self._roles_from_s(s, sets)
            contact_state = self._contact_state_from_roles(surface_id, leg_roles)
            nominal_targets = self._targets_from_s(s, initial_targets_world, landing_targets_world, current_targets_world, sets)
            nominal_targets, contact_lock_diag = self._apply_contact_locks(
                nominal_targets, previous_joint_map, leg_roles, phase_name, phase_step_index, frame_index)
            roll_progress = self._roll_progress_from_s(s)
            nominal_pose = self._base_pose_at_roll_progress(roll_progress)
            base_pose, pose_search_diag = self._choose_base_pose_for_s(
                nominal_pose, nominal_targets, previous_joint_map, leg_roles,
                phase_name, phase_step_index, frame_index)
            base_pose_search_records.append(pose_search_diag)
            if not pose_search_diag.get("selected_feasible", False):
                base_pose_search_failure_count += 1
            max_abs_base_pitch_deg = max(max_abs_base_pitch_deg, abs(math.degrees(base_pose.get("pitch", 0.0))))

            body_targets = self._world_targets_to_body(nominal_targets, base_pose)
            frame_body_targets, selection_diag = self._select_frame_targets(
                body_targets, previous_joint_map, leg_roles, phase_name, phase_step_index, frame_index)
            frame_world_targets = self._body_targets_to_world(frame_body_targets, base_pose)
            selection_diag["base_pose_search"] = pose_search_diag
            target_selection_records.extend(selection_diag["selection_records"])
            target_selection_failure_records.extend(selection_diag["failure_records"])
            max_candidate_count = max(max_candidate_count, selection_diag["max_candidate_count"])

            frame_diag = {"ik_failures": [], "target_selection": selection_diag, "contact_lock_generation": contact_lock_diag, "roll_progress_s": s, "body_roll_progress": roll_progress}
            new_joint_map = {}
            for leg_id in sorted(frame_body_targets.keys()):
                selected = self.robot_model.select_ik_body(leg_id, frame_body_targets[leg_id], previous_q=previous_joint_map.get(leg_id))
                if selected is None:
                    ik_failure_count += 1
                    rec = {
                        "frame_index": frame_index,
                        "phase_name": phase_name,
                        "phase_step_index": phase_step_index,
                        "roll_progress_s": s,
                        "body_roll_progress": roll_progress,
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
                max_second_joint_deg = max(max_second_joint_deg, second_deg)
                prev_q = previous_joint_map.get(leg_id)
                if prev_q is not None:
                    deltas = [abs(math.degrees(angle_delta(q[i], prev_q[i]))) for i in range(3)]
                    local_max = max(deltas)
                    max_joint_delta_deg = max(max_joint_delta_deg, local_max)
                    if local_max > self.config.max_joint_delta_deg:
                        discontinuity_records.append({
                            "frame_index": frame_index,
                            "phase_name": phase_name,
                            "phase_step_index": phase_step_index,
                            "roll_progress_s": s,
                            "body_roll_progress": roll_progress,
                            "leg_id": leg_id,
                            "leg_name": self.robot_model.leg_name(leg_id),
                            "role": leg_roles.get(leg_id, R.OTHER),
                            "delta_deg": deltas,
                            "max_delta_deg": local_max,
                            "base_pose": dict(base_pose),
                        })

            inter_leg_diag = self._evaluate_inter_leg_clearance(new_joint_map, leg_roles, phase_name, phase_step_index, frame_index, base_pose)
            ground_diag = self._evaluate_ground_clearance(new_joint_map, leg_roles, phase_name, phase_step_index, frame_index, base_pose)
            frame_diag["inter_leg_clearance"] = inter_leg_diag
            frame_diag["ground_clearance"] = ground_diag
            if inter_leg_diag.get("min_distance_m") is not None:
                min_inter_leg_distance = inter_leg_diag["min_distance_m"] if min_inter_leg_distance is None else min(min_inter_leg_distance, inter_leg_diag["min_distance_m"])
                if inter_leg_diag.get("below_threshold"):
                    inter_leg_near_records.append(inter_leg_diag)
            if ground_diag.get("min_clearance_m") is not None:
                min_ground_clearance = ground_diag["min_clearance_m"] if min_ground_clearance is None else min(min_ground_clearance, ground_diag["min_clearance_m"])
                if ground_diag.get("penetrating"):
                    ground_penetration_records.append(ground_diag)

            frames.append(V3MotionFrame(
                frame_index=frame_index,
                phase_index=self._phase_index_from_name(phase_name),
                phase_name=phase_name,
                phase_step_index=phase_step_index,
                phase_step_count=phase_step_count,
                contact_state=contact_state,
                base_pose=base_pose,
                leg_roles=leg_roles,
                foot_targets_body=frame_body_targets,
                foot_targets_world=frame_world_targets,
                joint_angles=new_joint_map,
                diagnostics=frame_diag,
            ))
            previous_joint_map = self._copy_joint_map(new_joint_map)
            current_targets_world = frame_world_targets

        completed = (ik_failure_count == 0 and len(ground_penetration_records) == 0 and base_pose_search_failure_count == 0)
        report.task_success = {
            "completed": completed,
            "direction": "forward",
            "surface_start": surface_id,
            "surface_after": self._next_forward_surface(surface_id),
            "planned_phase_count": len(phases),
            "frame_count": len(frames),
            "legacy_dependency": False,
            "base_pose_enabled": True,
            "trajectory_mode": "synchronized_progress",
            "roll_progress_synchronized": True,
            "auto_align_initial_ground": self.config.auto_align_initial_ground,
            "initial_base_z": self._roll_base_z0,
            "max_abs_base_pitch_deg": max_abs_base_pitch_deg,
            "base_pose_search_enabled": self.config.enable_body_roll_pose_search,
            "base_pose_search_failure_count": base_pose_search_failure_count,
            "contact_lock_generation_enabled": self.config.enable_contact_lock_generation,
            "contact_plan_variant": self.config.contact_plan_variant,
            "synchronized_steps": n,
            "roll_start_s": self.config.roll_start_s,
            "roll_end_s": self.config.roll_end_s,
        }
        report.joint_limit = {"second_joint_abs_max_deg": self.robot_model.leg_config.second_joint_abs_max_deg, "max_abs_second_joint_deg": max_second_joint_deg, "second_joint_limit_ok": max_second_joint_deg <= self.robot_model.leg_config.second_joint_abs_max_deg + 1e-9}
        report.motion_discontinuity = {"max_joint_delta_deg": max_joint_delta_deg, "warn_threshold_deg": self.config.max_joint_delta_deg, "discontinuity_count": len(discontinuity_records), "top_records": discontinuity_records[:20]}
        report.ik_reachability = {"ik_failure_count": ik_failure_count, "top_failure_records": ik_failure_records[:20]}
        report.ground_clearance = {"ground_z": self.config.ground_z, "min_clearance_m": min_ground_clearance, "penetration_count": len(ground_penetration_records), "top_penetration_records": ground_penetration_records[:20]}
        report.inter_leg_clearance = {"threshold_m": self.config.min_inter_leg_clearance_m, "min_distance_m": min_inter_leg_distance, "near_count": len(inter_leg_near_records), "top_near_records": inter_leg_near_records[:20]}
        report.base_pose_search = {"enabled": self.config.enable_body_roll_pose_search, "failure_count": base_pose_search_failure_count, "top_records": base_pose_search_records[:20], "top_failure_records": [r for r in base_pose_search_records if not r.get("selected_feasible", False)][:20]}
        report.support_consistency = {"target_selection_failure_count": len(target_selection_failure_records), "max_candidate_count_per_leg": max_candidate_count, "top_selection_failures": target_selection_failure_records[:20], "top_selection_records": target_selection_records[:20]}
        report.notes.append("v3.0.14: synchronized-progress generator drives base pose, leg roles, and foot targets from the same roll progress s.")
        return V3RollCandidate("forward", phases, frames, report)

    def _compute_landing_targets(self, initial_targets_world, sets):
        out = self._copy_targets(initial_targets_world)
        next_like = set(sets.get("candidate_support", [])) | set(sets.get("transfer_support", [])) | set(sets.get("posture_support", []))
        for leg_id in next_like:
            sign = 1.0 if int(leg_id) % 2 == 0 else -1.0
            out[int(leg_id)] = vec_add(out[int(leg_id)], [self.config.candidate_support_shift_x * sign, 0.0, self.config.candidate_support_drop_z])
        return out

    def _roll_progress_from_s(self, s):
        denom = max(1e-9, self.config.roll_end_s - self.config.roll_start_s)
        return smoothstep((s - self.config.roll_start_s) / denom)

    def _targets_from_s(self, s, initial_targets, landing_targets, current_targets, sets):
        out = self._copy_targets(initial_targets)
        # Candidate support placement is completed before the main body roll.
        place = smoothstep((s - 0.12) / 0.22)
        for leg_id, target in landing_targets.items():
            if leg_id in set(sets.get("candidate_support", [])) | set(sets.get("transfer_support", [])) | set(sets.get("posture_support", [])):
                out[leg_id] = [initial_targets[leg_id][i] + (target[i] - initial_targets[leg_id][i]) * place for i in range(3)]
        # Lift/retract legs that are not meant to remain as roll support.
        lift_up = smoothstep((s - 0.22) / 0.22)
        lift_down = smoothstep((s - 0.82) / 0.12)
        lift_profile = max(0.0, lift_up * (1.0 - 0.5 * lift_down))
        for leg_id in sets.get("lift_legs", []):
            leg_id = int(leg_id)
            out[leg_id] = list(out.get(leg_id, initial_targets[leg_id]))
            out[leg_id][2] += self.config.lift_height * lift_profile
        # Clearance legs are lifted earlier and kept away longer.
        clear_profile = smoothstep((s - 0.05) / 0.25) * (1.0 - 0.3 * smoothstep((s - 0.80) / 0.20))
        for leg_id in sets.get("clearance", []):
            leg_id = int(leg_id)
            if leg_id in sets.get("roll_support", []):
                continue
            out[leg_id] = list(out.get(leg_id, initial_targets[leg_id]))
            out[leg_id][2] += self.config.clearance_height * clear_profile
        return out

    def _roles_from_s(self, s, sets):
        roles = dict((m.leg_id, R.OTHER) for m in self.robot_model.mounts)
        if s < 0.18:
            support = sets.get("initial_support", [])
            clearance = sets.get("clearance", [])
            candidate = []
            lift = []
        elif s < 0.34:
            support = sets.get("initial_support", [])
            clearance = []
            candidate = sets.get("candidate_support", [])
            lift = []
        elif s < self.config.roll_start_s:
            support = sets.get("lift_support", sets.get("initial_support", []))
            clearance = []
            candidate = []
            lift = sets.get("lift_legs", [])
        elif s < self.config.roll_end_s:
            support = sets.get("roll_support", [])
            clearance = []
            candidate = []
            lift = [i for i in sets.get("lift_legs", []) if i not in support]
        elif s < 0.95:
            support = sets.get("transfer_support", [])
            clearance = []
            candidate = []
            lift = []
            for i in sets.get("transfer_legs", []):
                roles[int(i)] = R.TRANSFER
        else:
            support = sets.get("posture_support", sets.get("transfer_support", []))
            clearance = []
            candidate = []
            lift = []
        for i in support:
            roles[int(i)] = R.SUPPORT
        for i in clearance:
            if roles[int(i)] == R.OTHER:
                roles[int(i)] = R.CLEARANCE
        for i in candidate:
            if roles[int(i)] == R.OTHER:
                roles[int(i)] = R.CANDIDATE_SUPPORT
        for i in lift:
            if roles[int(i)] == R.OTHER:
                roles[int(i)] = R.LIFT
        return roles

    def _contact_state_from_roles(self, surface_id, roles):
        return ContactState(
            surface_id,
            support_legs=[i for i, r in roles.items() if r == R.SUPPORT],
            candidate_support_legs=[i for i, r in roles.items() if r == R.CANDIDATE_SUPPORT],
            lift_legs=[i for i, r in roles.items() if r == R.LIFT],
            clearance_legs=[i for i, r in roles.items() if r == R.CLEARANCE],
            transfer_legs=[i for i, r in roles.items() if r == R.TRANSFER],
        )

    def _phase_from_s(self, s, n):
        if s < 0.12:
            name = "StableInitialContact"
        elif s < 0.22:
            name = "ClearancePreparation"
        elif s < 0.34:
            name = "EstablishNextSupportCandidates"
        elif s < self.config.roll_start_s:
            name = "LiftTransitionLegs"
        elif s < self.config.roll_end_s:
            name = "ConstrainedBodyRoll"
        elif s < 0.95:
            name = "SupportTransfer"
        else:
            name = "PostureNormalization"
        # phase_step_index is approximate but useful for logs.
        return name, int(round(s * max(1, n - 1))), n

    def _phase_index_from_name(self, name):
        names = ["StableInitialContact", "ClearancePreparation", "EstablishNextSupportCandidates", "LiftTransitionLegs", "ConstrainedBodyRoll", "SupportTransfer", "PostureNormalization"]
        return names.index(name) if name in names else 0

    def _choose_base_pose_for_s(self, nominal_pose, nominal_world_targets, previous_joint_map, leg_roles, phase_name, phase_step_index, frame_index):
        if not self.config.enable_body_roll_pose_search:
            diag = self._score_base_pose(nominal_pose, nominal_world_targets, previous_joint_map, leg_roles, phase_name, phase_step_index, frame_index, candidate_index=0)
            diag["candidate_count"] = 1
            diag["selected_pose"] = dict(nominal_pose)
            diag["selected_feasible"] = diag.get("feasible", False)
            return nominal_pose, diag
        # Search is useful throughout the synchronized trajectory, not only in
        # ConstrainedBodyRoll, because x/z must remain continuous as legs move.
        candidates = []
        idx = 0
        for dx in self.config.body_roll_search_x_offsets:
            for dz in self.config.body_roll_search_z_offsets:
                pose = dict(nominal_pose)
                pose["x"] = nominal_pose.get("x", 0.0) + float(dx)
                pose["z"] = nominal_pose.get("z", 0.0) + float(dz)
                rec = self._score_base_pose(pose, nominal_world_targets, previous_joint_map, leg_roles, phase_name, phase_step_index, frame_index, candidate_index=idx)
                candidates.append(rec)
                idx += 1
        candidates.sort(key=lambda r: r["score"])
        best = candidates[0]
        out = dict(best)
        out["candidate_count"] = len(candidates)
        out["selected_pose"] = dict(best["base_pose"])
        out["selected_feasible"] = best.get("feasible", False)
        out["top_candidates"] = candidates[:5]
        return dict(best["base_pose"]), out
