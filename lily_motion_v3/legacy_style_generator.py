# -*- coding: utf-8 -*-
"""Legacy-style roll adapter for v3-core.

This module intentionally does NOT import or call the old project.  It maps the
legacy qualitative idea (six contacts, middle-leg step-over, singular/flip-prone
roll, moving-average smoothing downstream) into the common v3 candidate format.

It is a compatibility scaffold, not an exact numerical reproduction of the old
RF implementation.  The exact legacy adapter can later replace the profile
rules here while still returning the same V3RollCandidate object.
"""
from __future__ import division
import math

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.transforms import vec_add
from lily_motion_v3 import leg_role as R


class LegacyStyleRollGenerationConfig(V3RollGenerationConfig):
    def __init__(self, legacy_step_scale=1.5, legacy_splited_num=10,
                 legacy_rf2_pitch_scale=1.0, legacy_rf2_x_scale=1.0,
                 *args, **kwargs):
        kwargs.setdefault("contact_plan_variant", "legacy_rf_six_middle_roll")
        kwargs.setdefault("steps_per_phase", int(legacy_splited_num))
        V3RollGenerationConfig.__init__(self, *args, **kwargs)
        self.legacy_step_scale = float(legacy_step_scale)
        self.legacy_splited_num = int(legacy_splited_num)
        self.legacy_rf2_pitch_scale = float(legacy_rf2_pitch_scale)
        self.legacy_rf2_x_scale = float(legacy_rf2_x_scale)
        # Keep the legacy qualitative parameter values visible in reports.
        self.contact_plan_variant = "legacy_rf_six_middle_roll"
        self.steps_per_phase = int(legacy_splited_num)


class LegacyStyleRollCandidateGenerator(V3RollCandidateGenerator):
    """v3-core generator with legacy-style parameter hooks.

    Current mapping:
      * splited_num -> steps_per_phase
      * step_scale -> body x progression scale and candidate support shift scale
      * rf2_pitch_scale -> early support-candidate/base pre-shaping scale
      * rf2_x_scale -> early x pre-shaping scale

    This keeps the old-project dependency out of v3 while making legacy-like
    parameters sweepable through the common evaluator.
    """
    def __init__(self, robot_model=None, config=None):
        if config is None:
            config = LegacyStyleRollGenerationConfig()
        V3RollCandidateGenerator.__init__(self, robot_model=robot_model, config=config)
        self.profile_name = "legacy_style_six_contact_middle_step"


    def generate_forward_one_roll(self, initial_joint_map=None, surface_id=1):
        cand = V3RollCandidateGenerator.generate_forward_one_roll(self, initial_joint_map=initial_joint_map, surface_id=surface_id)
        if cand.report is not None:
            cand.report.task_success["profile"] = "legacy_style"
            cand.report.task_success["contact_plan_variant"] = "legacy_six_middle_roll"
            cand.report.task_success["internal_contact_plan_variant"] = "legacy_rf_six_middle_roll"
            cand.report.task_success["legacy_style_fidelity"] = "rf_named_six_contact_middle_step"
            cand.report.task_success["legacy_dependency"] = False
            cand.report.task_success["legacy_params"] = {
                "step_scale": self.config.legacy_step_scale,
                "splited_num": self.config.legacy_splited_num,
                "rf2_pitch_scale": self.config.legacy_rf2_pitch_scale,
                "rf2_x_scale": self.config.legacy_rf2_x_scale,
            }
            cand.report.notes.append(
                "v3.0.19: legacy-style adapter uses RF-1..RF-6 named phases and a six-contact middle-pair step scaffold; no old project calls."
            )
        return cand

    def _legacy_scale(self):
        # Step scale is centered around the historically used 1.5.  The adapter
        # should be conservative when a user provides another value.
        return self.config.legacy_step_scale / 1.5 if abs(1.5) > 1e-12 else 1.0

    def _base_pose_for_phase(self, phase_name, alpha):
        # RF-2 pre-shapes the body mildly before the main roll.  RF-4 performs
        # the main legacy-style roll.  RF-5/RF-6 inherit the terminal roll pose.
        # The old implementation used implicit surface-dependent values; this
        # v3-contained adapter keeps them explicit and sweepable.
        if phase_name in ("EstablishNextSupportCandidates", "RF-2_NextSurfacePreShape"):
            pre = max(0.0, min(1.0, alpha))
            progress = 0.18 * self.config.legacy_rf2_pitch_scale * pre
            pose = self._base_pose_at_roll_progress(progress)
            pose["x"] += 0.12 * self.config.legacy_rf2_x_scale * self._legacy_scale() * pre
            return pose
        if phase_name in ("ConstrainedBodyRoll", "RF-4_BodyRollThroughSingular"):
            return self._base_pose_at_roll_progress(max(0.0, min(1.0, alpha)))
        if phase_name in ("SupportTransfer", "PostureNormalization", "RF-5_SupportTransfer", "RF-6_PostureNormalization"):
            if getattr(self, "_post_roll_base_pose", None) is not None:
                return dict(self._post_roll_base_pose)
            return self._base_pose_at_roll_progress(1.0)
        return V3RollCandidateGenerator._base_pose_for_phase(self, phase_name, alpha)

    def _phase_goal_body_targets(self, phase, start_targets):
        # Start from the native rule and add legacy-style scaling to the parts
        # that correspond to old RF2/RF3 step-over behavior.
        out = V3RollCandidateGenerator._phase_goal_body_targets(self, phase, start_targets)
        if phase.name in ("EstablishNextSupportCandidates", "RF-2_NextSurfacePreShape"):
            # Amplify/dampen candidate support x shift and z drop according to
            # the explicit RF2-style knobs.  We apply a delta relative to start
            # to avoid double counting the base class defaults.
            for leg_id in phase.contact_state.candidate_support_legs:
                leg_id = int(leg_id)
                if leg_id not in out:
                    continue
                base = list(start_targets[leg_id])
                sign = 1.0 if leg_id % 2 == 0 else -1.0
                dx = self.config.candidate_support_shift_x * sign * self.config.legacy_rf2_x_scale * self._legacy_scale()
                dz = self.config.candidate_support_drop_z
                out[leg_id] = vec_add(base, [dx, 0.0, dz])
        if phase.name in ("LiftTransitionLegs", "RF-3_LiftMiddlePair"):
            # The old motion intentionally lifted/stepped the middle pair while
            # the body had already been pre-shaped.  Give those lift legs a bit
            # more clearance as step_scale grows, but do not alter support locks.
            extra = max(0.0, self._legacy_scale() - 1.0) * 0.04
            for leg_id in phase.contact_state.lift_legs:
                leg_id = int(leg_id)
                if leg_id in out:
                    out[leg_id][2] += extra
        return out
