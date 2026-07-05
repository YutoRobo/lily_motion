#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dry-run adapter from v3 command logs to /cmdForJetson JointState(position[24]).

This script does not import rospy, does not open can0, and does not transmit.
It validates a command log against hardware_limit_v2, records the joint order
mapping expected by the referenced CAN state machine, and writes preview files
that show the JointState position vector and CAN float payloads that would be
formed downstream.
"""
from __future__ import print_function

import argparse
import csv
import json
import math
import os
import struct
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID


JOINT_NAMES = ['base_clause', 'thigh', 'tibia']
HARDWARE_LIMIT_V2_DEG = {
    'base_clause': (-360.0, 360.0),
    'thigh': (-95.0, 95.0),
    'tibia': (-150.0, 150.0),
}
BASE_SOFT_THRESHOLDS_DEG = [330.0, 340.0]


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def write_json(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write('\n')


def write_csv(path, fields, rows):
    with open(path, 'w') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def load_command_records(path):
    records = []
    with open(path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            rec = json.loads(line)
            rec = dict(rec)
            rec.setdefault('frame_index', rec.get('command_index', i))
            if 'joint_command_rad' not in rec:
                if 'joint_command_deg' not in rec:
                    raise ValueError('record %d has no joint_command_rad or joint_command_deg: %s' % (i, path))
                rec['joint_command_rad'] = [math.radians(float(v)) for v in rec['joint_command_deg']]
            if 'joint_command_deg' not in rec:
                rec['joint_command_deg'] = [math.degrees(float(v)) for v in rec['joint_command_rad']]
            if len(rec['joint_command_rad']) != 24:
                raise ValueError('record %d expected 24 joint_command_rad values, got %d' % (i, len(rec['joint_command_rad'])))
            records.append(rec)
    return records


def mapping_rows():
    rows = []
    for idx, (leg_id, joint_index) in enumerate(JOINT_STATE_ORDER):
        rows.append({
            'position_index': idx,
            'can_id_hex': '0x%03X' % (0x400 + idx),
            'can_id_dec': 0x400 + idx,
            'leg_id': leg_id,
            'leg_name': LEG_NAMES_BY_ID[leg_id],
            'joint_index': joint_index,
            'joint_name': JOINT_NAMES[joint_index],
            'unit': 'rad',
            'source_field': 'joint_command_rad[%d]' % idx,
        })
    return rows


def evaluate_hardware_limits(records):
    violations = []
    minmax = {}
    soft_counts = dict((str(t), 0) for t in BASE_SOFT_THRESHOLDS_DEG)
    for ri, rec in enumerate(records):
        for idx, (leg_id, joint_index) in enumerate(JOINT_STATE_ORDER):
            leg_name = LEG_NAMES_BY_ID[leg_id]
            joint_name = JOINT_NAMES[joint_index]
            value_deg = float(rec['joint_command_deg'][idx])
            key = (leg_name, joint_name)
            stat = minmax.setdefault(key, {
                'leg_name': leg_name,
                'joint_name': joint_name,
                'min_deg': value_deg,
                'max_deg': value_deg,
                'max_abs_deg': abs(value_deg),
                'hard_violation_count': 0,
            })
            stat['min_deg'] = min(stat['min_deg'], value_deg)
            stat['max_deg'] = max(stat['max_deg'], value_deg)
            stat['max_abs_deg'] = max(stat['max_abs_deg'], abs(value_deg))
            lo, hi = HARDWARE_LIMIT_V2_DEG[joint_name]
            side = None
            excess = 0.0
            if value_deg < lo:
                side = 'below_min'
                excess = lo - value_deg
            elif value_deg > hi:
                side = 'above_max'
                excess = value_deg - hi
            if excess > 0.0:
                stat['hard_violation_count'] += 1
                violations.append({
                    'record_index': ri,
                    'frame_index': rec.get('frame_index', ri),
                    'roll_index': rec.get('roll_index'),
                    'phase_name': rec.get('phase_name'),
                    'position_index': idx,
                    'leg_name': leg_name,
                    'joint_name': joint_name,
                    'value_deg': value_deg,
                    'limit_min_deg': lo,
                    'limit_max_deg': hi,
                    'side': side,
                    'excess_deg': excess,
                })
            if joint_name == 'base_clause':
                for threshold in BASE_SOFT_THRESHOLDS_DEG:
                    if abs(value_deg) > threshold:
                        soft_counts[str(threshold)] += 1
    return {
        'hard_violation_count': len(violations),
        'base_soft_margin_counts': soft_counts,
        'minmax': [minmax[k] for k in sorted(minmax)],
        'violations': violations,
    }


def float_payload_bytes(value):
    packed = struct.pack('<f', float(value))
    return [b if isinstance(b, int) else ord(b) for b in packed]


def write_previews(records, out_dir, max_preview_frames):
    jointstate_path = os.path.join(out_dir, 'jointstate_preview.jsonl')
    can_path = os.path.join(out_dir, 'can_frame_preview.jsonl')
    preview_count = min(len(records), int(max_preview_frames))
    with open(jointstate_path, 'w') as js, open(can_path, 'w') as cf:
        for preview_index, rec in enumerate(records[:preview_count]):
            positions = [float(v) for v in rec['joint_command_rad']]
            context = {
                'preview_index': preview_index,
                'frame_index': rec.get('frame_index', preview_index),
                'roll_index': rec.get('roll_index'),
                'phase_name': rec.get('phase_name'),
                'phase_step_index': rec.get('phase_step_index'),
            }
            js_rec = dict(context)
            js_rec.update({
                'topic': '/cmdForJetson',
                'message_type': 'sensor_msgs/JointState',
                'position_unit': 'rad',
                'position': positions,
                'position_length': len(positions),
                'name': [
                    '%s_%s' % (LEG_NAMES_BY_ID[leg_id], JOINT_NAMES[joint_index])
                    for leg_id, joint_index in JOINT_STATE_ORDER
                ],
            })
            js.write(json.dumps(js_rec, sort_keys=True))
            js.write('\n')
            for idx, value in enumerate(positions):
                leg_id, joint_index = JOINT_STATE_ORDER[idx]
                cf_rec = dict(context)
                cf_rec.update({
                    'position_index': idx,
                    'can_id_hex': '0x%03X' % (0x400 + idx),
                    'can_id_dec': 0x400 + idx,
                    'leg_id': leg_id,
                    'leg_name': LEG_NAMES_BY_ID[leg_id],
                    'joint_index': joint_index,
                    'joint_name': JOINT_NAMES[joint_index],
                    'value_rad': value,
                    'value_deg': math.degrees(value),
                    'payload_bytes_dec': [0, 0, 0, 0] + float_payload_bytes(value),
                    'payload_format': '[0,0,0,0] + little_endian_float32(position[index])',
                })
                cf.write(json.dumps(cf_rec, sort_keys=True))
                cf.write('\n')
    return jointstate_path, can_path, preview_count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--command-log', default='data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl')
    ap.add_argument('--output-dir', default='testdata/can_jointstate_dry_run_candidate02_softlimit_94p8')
    ap.add_argument('--max-preview-frames', type=int, default=5)
    args = ap.parse_args()

    ensure_dir(args.output_dir)
    records = load_command_records(args.command_log)
    limit_report = evaluate_hardware_limits(records)
    jointstate_path, can_path, preview_count = write_previews(records, args.output_dir, args.max_preview_frames)
    mapping = mapping_rows()

    write_csv(
        os.path.join(args.output_dir, 'joint_order_mapping.csv'),
        ['position_index', 'can_id_hex', 'can_id_dec', 'leg_id', 'leg_name', 'joint_index', 'joint_name', 'unit', 'source_field'],
        mapping,
    )
    write_csv(
        os.path.join(args.output_dir, 'hardware_limit_violations.csv'),
        ['record_index', 'frame_index', 'roll_index', 'phase_name', 'position_index', 'leg_name', 'joint_name', 'value_deg', 'limit_min_deg', 'limit_max_deg', 'side', 'excess_deg'],
        limit_report['violations'],
    )
    write_json(os.path.join(args.output_dir, 'hardware_limit_v2_report.json'), {
        'policy_name': 'hardware_limit_v2',
        'hard_limits_deg': dict((k, {'min': v[0], 'max': v[1]}) for k, v in HARDWARE_LIMIT_V2_DEG.items()),
        'base_soft_margin_thresholds_deg': BASE_SOFT_THRESHOLDS_DEG,
        'hard_violation_count': limit_report['hard_violation_count'],
        'base_soft_margin_counts': limit_report['base_soft_margin_counts'],
        'minmax': limit_report['minmax'],
        'pass': limit_report['hard_violation_count'] == 0,
    })

    summary = {
        'dry_run': True,
        'opened_can0': False,
        'sent_to_hardware': False,
        'command_log': args.command_log,
        'frame_count': len(records),
        'preview_frame_count': preview_count,
        'topic': '/cmdForJetson',
        'message_type': 'sensor_msgs/JointState',
        'position_length': 24,
        'position_unit': 'rad',
        'joint_order': 'lily_motion_v3.interface_config.JOINT_STATE_ORDER',
        'can_program_expectation': {
            'source': 'tools/can_interface/statemachine/state_machine.py',
            'subscriber': '/cmdForJetson JointState',
            'minimum_position_length': 24,
            'can_id_rule': '0x400 + position_index',
            'payload_rule': '[0,0,0,0] + struct.pack(\"<f\", position[index])',
            'run_mode_required_by_can_program': True,
        },
        'hardware_limit_v2': {
            'pass': limit_report['hard_violation_count'] == 0,
            'hard_violation_count': limit_report['hard_violation_count'],
            'base_soft_margin_counts': limit_report['base_soft_margin_counts'],
        },
        'outputs': {
            'joint_order_mapping_csv': os.path.join(args.output_dir, 'joint_order_mapping.csv'),
            'jointstate_preview_jsonl': jointstate_path,
            'can_frame_preview_jsonl': can_path,
            'hardware_limit_v2_report_json': os.path.join(args.output_dir, 'hardware_limit_v2_report.json'),
            'hardware_limit_violations_csv': os.path.join(args.output_dir, 'hardware_limit_violations.csv'),
        },
    }
    write_json(os.path.join(args.output_dir, 'summary.json'), summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
