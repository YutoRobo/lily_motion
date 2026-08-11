#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import imp
import json
import os
import shutil
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
quarter = imp.load_source('build_roll_quarter_stages', os.path.join(ROOT, 'tools', 'build_roll_quarter_stages.py'))
runner = imp.load_source('run_roll_quarter_stage', os.path.join(ROOT, 'tools', 'run_roll_quarter_stage.py'))


def record(roll_index, value=0.0):
    return {
        'roll_index': roll_index,
        'joint_command_rad': [float(value)] * 24,
    }


class QuarterStageBuilderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='lily_quarter_stage_')
        self.source = os.path.join(self.tmp, 'commands.jsonl')
        self.out = os.path.join(self.tmp, 'staged')

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def write(self, records):
        with open(self.source, 'w') as f:
            for item in records:
                f.write(json.dumps(item, sort_keys=True) + '\n')

    def count_lines(self, path):
        with open(path) as f:
            return len([line for line in f if line.strip()])

    def test_unequal_semantic_blocks_generate_cumulative_prefixes(self):
        records = []
        lengths = [2, 3, 1, 4]
        for ri, length in enumerate(lengths):
            for i in range(length):
                records.append(record(ri, len(records)))
        self.write(records)
        manifest = quarter.build(self.source, self.out, 4, False)
        self.assertEqual([b['frame_count'] for b in manifest['roll_blocks']], lengths)
        self.assertEqual([s['frame_count'] for s in manifest['stages']], [2, 5, 6, 10])
        for stage, expected in zip(manifest['stages'], [2, 5, 6, 10]):
            self.assertEqual(self.count_lines(os.path.join(self.out, stage['path'])), expected)

    def test_last_stage_is_complete_source_sequence(self):
        self.write([record(10), record(20), record(30), record(40)])
        manifest = quarter.build(self.source, self.out, 4, False)
        last = os.path.join(self.out, manifest['stages'][-1]['path'])
        with open(self.source, 'rb') as a, open(last, 'rb') as b:
            self.assertEqual(a.read(), b.read())

    def test_reappearing_roll_index_is_rejected(self):
        self.write([record(0), record(1), record(0), record(2), record(3)])
        with self.assertRaises(ValueError):
            quarter.build(self.source, self.out, 4, True)

    def test_wrong_roll_count_is_rejected(self):
        self.write([record(0), record(1), record(2)])
        with self.assertRaises(ValueError):
            quarter.build(self.source, self.out, 4, True)

    def test_missing_roll_index_is_rejected(self):
        item = {'joint_command_rad': [0.0] * 24}
        self.write([item])
        with self.assertRaises(ValueError):
            quarter.build(self.source, self.out, 4, True)

    def test_wrong_position_length_is_rejected(self):
        item = {'roll_index': 0, 'joint_command_rad': [0.0] * 23}
        self.write([item])
        with self.assertRaises(ValueError):
            quarter.build(self.source, self.out, 4, True)

    def test_stage_parser_accepts_human_quarter_notation(self):
        self.assertEqual(runner.normalize_stage('2'), 2)
        self.assertEqual(runner.normalize_stage('2/4'), 2)
        self.assertEqual(runner.normalize_stage('roll-to-2of4'), 2)
        with self.assertRaises(ValueError):
            runner.normalize_stage('5/4')


if __name__ == '__main__':
    unittest.main()
