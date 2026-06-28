# -*- coding: utf-8 -*-
from __future__ import division
import os
import shutil
import tempfile
import unittest

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig
from lily_motion_v3.visualizer import select_key_frame_indices, visualize_candidate


class V3VisualizerTest(unittest.TestCase):
    def test_select_key_frames_returns_existing_indices(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=3))
        cand = gen.generate_forward_one_roll(surface_id=1)
        whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig()).evaluate(cand)
        indices = select_key_frame_indices(cand, whole, max_frames=10)
        self.assertTrue(indices)
        self.assertEqual(indices, sorted(set(indices)))
        self.assertGreaterEqual(indices[0], 0)
        self.assertLess(indices[-1], len(cand.frames))

    def test_visualize_candidate_writes_manifest(self):
        gen = V3RollCandidateGenerator(config=V3RollGenerationConfig(steps_per_phase=2))
        cand = gen.generate_forward_one_roll(surface_id=1)
        whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(filter_window=1)).evaluate(cand)
        tmp = tempfile.mkdtemp(prefix='v3vis_')
        try:
            manifest = visualize_candidate(
                gen.robot_model, cand, whole, tmp,
                command_source='filtered', filter_window=1,
                frame_indices=[0], max_frames=1, ground_z=0.0)
            self.assertTrue(os.path.exists(manifest['manifest_path']))
            self.assertTrue(os.path.exists(manifest['html_path']))
            self.assertEqual(manifest['rendered_frame_count'], 1)
        finally:
            shutil.rmtree(tmp)


if __name__ == '__main__':
    unittest.main()
