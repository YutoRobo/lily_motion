# -*- coding: utf-8 -*-
from __future__ import division
import json
import os
import tempfile
import unittest

from lily_motion_v3.reference_importer import candidate_from_reference_file, candidate_to_json_file, candidate_from_json_file
from lily_motion_v3.gazebo_export import V3GazeboCommandExporter
from lily_motion_v3.robot_model import RobotModel


class TestV3ReferenceImporter(unittest.TestCase):
    def test_import_jsonl_joint_command_rad(self):
        tmpdir = tempfile.mkdtemp()
        path = os.path.join(tmpdir, 'cmd.jsonl')
        cmd0 = [0.0] * 24
        cmd1 = [0.01 * i for i in range(24)]
        with open(path, 'w') as f:
            f.write(json.dumps({'frame_index': 0, 'phase_name': 'RF-1', 'joint_command_rad': cmd0}) + '\n')
            f.write(json.dumps({'frame_index': 1, 'phase_name': 'RF-1', 'joint_command_rad': cmd1}) + '\n')
        cand = candidate_from_reference_file(path)
        self.assertEqual(len(cand.frames), 2)
        self.assertEqual(cand.frames[0].phase_name, 'RF-1')
        self.assertTrue(cand.report.task_success.get('legacy_dependency') is False)
        exporter = V3GazeboCommandExporter(RobotModel())
        out = exporter.frame_to_joint_state_order(cand.frames[1])
        self.assertEqual(len(out), 24)
        self.assertAlmostEqual(out[0], cmd1[0])

    def test_candidate_json_roundtrip(self):
        tmpdir = tempfile.mkdtemp()
        in_path = os.path.join(tmpdir, 'cmd.jsonl')
        out_path = os.path.join(tmpdir, 'candidate.json')
        with open(in_path, 'w') as f:
            f.write(json.dumps({'joint_command_rad': [0.0] * 24}) + '\n')
        cand = candidate_from_reference_file(in_path)
        candidate_to_json_file(cand, out_path)
        robot_model, loaded = candidate_from_json_file(out_path)
        self.assertEqual(len(loaded.frames), 1)
        self.assertEqual(robot_model.leg_name(0), 'TRF')


if __name__ == '__main__':
    unittest.main()
