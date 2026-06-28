# -*- coding: utf-8 -*-
from __future__ import division
import math
import unittest

from lily_motion_v3.leg_config import LegKinematicConfig, default_octpus_mounts
from lily_motion_v3.leg_kinematics import LegKinematics
from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.v3_roll_concept_generator import build_forward_roll_concept


class TestV30Kinematics(unittest.TestCase):
    def assertVecAlmostEqual(self, a, b, places=6):
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=places)

    def test_fk_ik_round_trip_leg_frame(self):
        kin = LegKinematics(LegKinematicConfig())
        q = [0.25, -0.35, 0.9]
        p = kin.forward_kinematics(q)
        cands = kin.inverse_kinematics_candidates(p)
        self.assertEqual(len(cands), 2)
        selected = kin.select_candidate(cands, previous_q=q)
        self.assertIsNotNone(selected)
        p2 = kin.forward_kinematics(selected.q)
        self.assertVecAlmostEqual(p, p2, places=6)

    def test_unreachable_candidate_is_marked(self):
        kin = LegKinematics(LegKinematicConfig())
        cands = kin.inverse_kinematics_candidates([10.0, 0.0, 0.0])
        self.assertEqual(len(cands), 2)
        self.assertFalse(cands[0].reachable)
        self.assertFalse(cands[1].reachable)
        self.assertIsNone(kin.select_candidate(cands))

    def test_second_joint_limit_selection(self):
        cfg = LegKinematicConfig(second_joint_abs_max_deg=20.0)
        kin = LegKinematics(cfg)
        # High target tends to require a large positive thigh pitch.
        cands = kin.inverse_kinematics_candidates([0.2, 0.0, 0.45])
        selected = kin.select_candidate(cands)
        if selected is not None:
            self.assertLessEqual(abs(math.degrees(selected.q[1])), 20.0 + 1e-6)

    def test_robot_model_body_fk_ik(self):
        model = RobotModel(mounts=default_octpus_mounts())
        q = [0.1, -0.2, 0.7]
        foot_body = model.foot_position_body(0, q)
        selected = model.select_ik_body(0, foot_body, previous_q=q)
        self.assertIsNotNone(selected)
        foot2 = model.foot_position_body(0, selected.q)
        self.assertVecAlmostEqual(foot_body, foot2, places=6)

    def test_v3_roll_concept_has_role_phases(self):
        phases = build_forward_roll_concept(1)
        names = [p.name for p in phases]
        self.assertIn("LiftTransitionLegs", names)
        self.assertIn("ConstrainedBodyRoll", names)
        for p in phases:
            d = p.to_dict()
            self.assertTrue(d["purpose"])
            self.assertIsNotNone(d["contact_state"])


if __name__ == "__main__":
    unittest.main()

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig


class TestV301RollCandidate(unittest.TestCase):
    def test_v3_one_roll_candidate_is_project_contained(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=3))
        cand = gen.generate_forward_one_roll(surface_id=1)
        d = cand.to_dict()
        self.assertEqual(d["direction"], "forward")
        self.assertEqual(d["phase_count"], 7)
        self.assertEqual(d["frame_count"], 21)
        self.assertFalse(d["report"]["task_success"]["legacy_dependency"])
        self.assertIn("joint_limit", d["report"])
        self.assertIn("ik_reachability", d["report"])

    def test_v3_one_roll_candidate_has_lift_roles(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        lift_frames = [f for f in cand.frames if f.phase_name == "LiftTransitionLegs"]
        self.assertTrue(lift_frames)
        roles = lift_frames[-1].leg_roles
        self.assertEqual(roles[0], "LIFT")
        self.assertEqual(roles[2], "LIFT")

    def test_v3_run_summary_structure(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=1))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = cand.report.to_dict()
        self.assertIn("completed", report["task_success"])
        self.assertIn("max_abs_second_joint_deg", report["joint_limit"])
        self.assertIn("max_joint_delta_deg", report["motion_discontinuity"])

from lily_motion_v3.foot_target_candidate import CandidateFootTargetGenerator, CandidateFootTargetConfig


class TestV302CandidateTargets(unittest.TestCase):
    def test_candidate_foot_target_generator_returns_multiple_lift_candidates(self):
        model = RobotModel()
        gen = CandidateFootTargetGenerator(model, CandidateFootTargetConfig(lift_height=0.08))
        current = model.foot_position_body(0, [0.0, -0.45, 0.90])
        cands = gen.generate_candidates(0, "LIFT", current)
        self.assertGreater(len(cands), 1)
        self.assertTrue(any(c.target[2] > current[2] for c in cands))

    def test_v3_roll_candidate_reports_candidate_selection_and_clearance(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = cand.report.to_dict()
        self.assertIn("target_selection_failure_count", report["support_consistency"])
        self.assertIn("min_distance_m", report["inter_leg_clearance"])
        self.assertIn("top_selection_records", report["support_consistency"])

class TestV303BasePoseRoll(unittest.TestCase):
    def test_v3_roll_candidate_has_nonzero_base_pitch(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=3))
        cand = gen.generate_forward_one_roll(surface_id=1)
        pitches = [abs(f.base_pose.get("pitch", 0.0)) for f in cand.frames]
        self.assertGreater(max(pitches), 1.0)
        report = cand.report.to_dict()
        self.assertTrue(report["task_success"]["base_pose_enabled"])
        self.assertIn("surface_after", report["task_success"])
        self.assertIn("min_clearance_m", report["ground_clearance"])

class TestV304BasePoseSearch(unittest.TestCase):
    def test_v3_roll_candidate_reports_base_pose_search(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=3,
            enable_body_roll_pose_search=True,
            body_roll_search_x_offsets=[0.0],
            body_roll_search_z_offsets=[0.0, 0.1],
        ))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = cand.report.to_dict()
        self.assertIn("base_pose_search", report)
        self.assertTrue(report["base_pose_search"]["enabled"])
        self.assertIn("failure_count", report["base_pose_search"])
        self.assertTrue(report["task_success"]["base_pose_search_enabled"])

class TestV305BasePoseHandoff(unittest.TestCase):
    def test_support_transfer_inherits_terminal_body_roll_pose(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=4,
            enable_body_roll_pose_search=True,
            body_roll_search_x_offsets=[0.0, 0.1],
            body_roll_search_z_offsets=[0.0, 0.2],
        ))
        cand = gen.generate_forward_one_roll(surface_id=1)
        roll_frames = [f for f in cand.frames if f.phase_name == "ConstrainedBodyRoll"]
        transfer_frames = [f for f in cand.frames if f.phase_name == "SupportTransfer"]
        self.assertTrue(roll_frames)
        self.assertTrue(transfer_frames)
        terminal = roll_frames[-1].base_pose
        first_transfer = transfer_frames[0].base_pose
        self.assertAlmostEqual(first_transfer["x"], terminal["x"])
        self.assertAlmostEqual(first_transfer["z"], terminal["z"])
        self.assertAlmostEqual(first_transfer["pitch"], terminal["pitch"])

from lily_motion_v3.gazebo_export import V3GazeboCommandExporter, frames_until_invalid


class TestV306GazeboExport(unittest.TestCase):
    def test_v3_frame_exports_to_24_joint_command(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=1))
        cand = gen.generate_forward_one_roll(surface_id=1)
        exporter = V3GazeboCommandExporter(gen.robot_model)
        cmd = exporter.frame_to_joint_state_order(cand.frames[0])
        self.assertEqual(len(cmd), 24)
        # First three commands are BRF in existing Gazebo order, while v3 BRF is leg 2.
        self.assertEqual(cmd[0], cand.frames[0].joint_angles[2][0])
        self.assertEqual(cmd[1], cand.frames[0].joint_angles[2][1])
        self.assertEqual(cmd[2], cand.frames[0].joint_angles[2][2])

    def test_frames_until_invalid_stops_before_first_failure(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=8))
        cand = gen.generate_forward_one_roll(surface_id=1)
        frames, first_invalid = frames_until_invalid(cand.frames, include_invalid=False)
        self.assertIsNotNone(first_invalid)
        self.assertLess(len(frames), len(cand.frames))
        if frames:
            self.assertLess(frames[-1].frame_index, first_invalid["frame_index"])

from lily_motion_v3.command_filter import filter_joint_trajectory, filter_joint_trajectory_contact_reproject, max_joint_step_deg
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig
from lily_motion_v3.contact_lock import ContactLockTracker


class TestV308WholeRollEvaluation(unittest.TestCase):
    def test_filter_preserves_frame_count(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        filtered = filter_joint_trajectory(cand.frames, window=3)
        self.assertEqual(len(filtered), len(cand.frames))
        self.assertEqual(sorted(filtered[0].keys()), sorted(cand.frames[0].joint_angles.keys()))

    def test_whole_roll_evaluator_reports_filtered_and_contact_lock(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(filter_window=3)).evaluate(cand)
        self.assertIn("raw_command", report)
        self.assertIn("filtered_command", report)
        self.assertIn("contact_lock", report)
        self.assertIn("max_contact_drift_m", report["contact_lock"])
        self.assertIn("whole_roll_success_by_filtered_geometry", report)

from lily_motion_v3.v3_roll_concept_generator import CONTACT_PLAN_VARIANTS


class TestV309ContactLockedGeneration(unittest.TestCase):
    def test_contact_plan_variants_are_buildable(self):
        for variant in CONTACT_PLAN_VARIANTS:
            phases = build_forward_roll_concept(1, contact_plan_variant=variant)
            self.assertEqual(len(phases), 7)
            self.assertEqual(phases[4].name, "ConstrainedBodyRoll")

    def test_v3_frames_record_generation_contact_locks(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        first = cand.frames[0].diagnostics.get("contact_lock_generation")
        self.assertTrue(first["enabled"])
        self.assertTrue(first["created_locks"])
        report = cand.report.to_dict()
        self.assertTrue(report["task_success"]["contact_lock_generation_enabled"])
        self.assertEqual(report["task_success"]["contact_plan_variant"], "default")

    def test_next_only_contact_plan_changes_support_set(self):
        phases = build_forward_roll_concept(1, contact_plan_variant="next_only_roll")
        roll_support = phases[4].contact_state.support_legs
        self.assertEqual(sorted(roll_support), [4, 6])


class TestV310ContactPreservingFilter(unittest.TestCase):
    def test_contact_preserving_filter_reports_projection(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=2, contact_plan_variant="front_pair_roll"))
        cand = gen.generate_forward_one_roll(surface_id=1)
        filtered, diag = filter_joint_trajectory_contact_reproject(cand.frames, gen.robot_model, window=3)
        self.assertEqual(len(filtered), len(cand.frames))
        self.assertTrue(diag["enabled"])
        self.assertIn("projection_failure_count", diag)
        self.assertIn("projected_count", diag)

    def test_whole_roll_evaluator_accepts_contact_preserving_filter(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=2, contact_plan_variant="front_pair_roll"))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
            filter_window=3, contact_preserving_filter=True)).evaluate(cand)
        self.assertEqual(report["filter"]["type"], "moving_average_unwrapped_angles_contact_reproject")
        self.assertTrue(report["filter"]["contact_preserving_projection"]["enabled"])

class TestV311SoftContactDrift(unittest.TestCase):
    def test_contact_lock_tracker_reports_soft_and_hard_limits(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=2, contact_plan_variant="front_pair_roll"))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
            filter_window=3,
            contact_drift_soft_limit_m=0.02,
            contact_drift_hard_limit_m=0.15,
        )).evaluate(cand)
        contact = report["contact_lock"]
        self.assertIn("contact_drift_soft_violation_count", contact)
        self.assertIn("contact_drift_hard_violation_count", contact)
        self.assertIn("contact_drift_soft_excess_sum_m", contact)
        self.assertIn("contact_drift_hard_excess_sum_m", contact)

    def test_success_uses_hard_contact_drift_limit_not_soft_limit(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(
            steps_per_phase=2, contact_plan_variant="front_pair_roll"))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
            filter_window=3,
            contact_drift_soft_limit_m=0.0,
            contact_drift_hard_limit_m=10.0,
        )).evaluate(cand)
        self.assertEqual(report["filter"]["contact_drift_hard_limit_m"], 10.0)
        self.assertEqual(report["contact_lock"]["contact_drift_hard_violation_count"], 0)

from lily_motion_v3.failure_diagnosis import summarize_failure_diagnosis


class TestV315CoreIndependenceAndDiagnosis(unittest.TestCase):
    def test_failure_diagnosis_reports_histograms(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        report = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(filter_window=3)).evaluate(cand)
        diag = report["failure_diagnosis"]
        self.assertIn("dominant_failure_category", diag)
        self.assertIn("generator_ik_failure", diag["categories"])
        self.assertIn("by_phase", diag["categories"]["generator_ik_failure"]["histogram"])

    def test_gazebo_export_does_not_require_legacy_package_constants(self):
        import lily_motion_v3.gazebo_export as ge
        self.assertTrue(hasattr(ge, "V3GazeboCommandExporter"))

class TestV317LegacyStyleAdapter(unittest.TestCase):
    def test_legacy_style_adapter_is_project_contained(self):
        from lily_motion_v3.legacy_style_generator import LegacyStyleRollCandidateGenerator, LegacyStyleRollGenerationConfig
        gen = LegacyStyleRollCandidateGenerator(config=LegacyStyleRollGenerationConfig(legacy_splited_num=3))
        cand = gen.generate_forward_one_roll(surface_id=1)
        self.assertFalse(cand.report.task_success.get("legacy_dependency"))
        self.assertEqual(cand.report.task_success.get("contact_plan_variant"), "legacy_six_middle_roll")
        self.assertGreater(len(cand.frames), 0)

    def test_legacy_six_middle_roll_variant_is_buildable(self):
        from lily_motion_v3.v3_roll_concept_generator import build_forward_roll_concept, CONTACT_PLAN_VARIANTS
        self.assertIn("legacy_six_middle_roll", CONTACT_PLAN_VARIANTS)
        phases = build_forward_roll_concept(1, "legacy_six_middle_roll")
        self.assertEqual(phases[3].name, "LiftTransitionLegs")
        self.assertEqual(set(phases[3].contact_state.lift_legs), set([0, 2]))
