#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Set the touchdown hold posture in Gazebo and sample z-offset variants.

This is a Gazebo-only visual/check helper.  It does not import or use the CAN
interface, does not open can0, and does not publish Jetson/CAN commands.
"""
from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
from lily_motion_v3.ros_bridge import GazeboCommandPublisher, GazeboLinkStateLogger, split_csv


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def write_json(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write('\n')


def parse_float_list(text):
    out = []
    for part in str(text).split(','):
        part = part.strip()
        if part:
            out.append(float(part))
    return out


def load_first_record(path):
    with open(path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            rec = json.loads(line)
            if 'joint_command_rad' not in rec:
                raise ValueError('record %d has no joint_command_rad: %s' % (i, path))
            return rec
    raise ValueError('empty command log: %s' % path)


def offset_record_base_z(rec, offset_m):
    out = dict(rec)
    bp = dict(out.get('base_pose') or {})
    bp['z'] = float(bp.get('z', 0.4)) + float(offset_m)
    out['base_pose'] = bp
    out['touchdown_offset_m'] = float(offset_m)
    return out


def fk_touchdown_report(rec, offset_m, top_n=20):
    ev = LegacyConstraintEvaluator(
        second_joint_limit_deg=95.0,
        ground_z=0.0,
        ground_tol=1e-4,
        inter_leg_limit_m=0.04,
        default_body_z=0.4,
        leg_radius_m=0.015,
        inter_leg_safety_margin_m=0.010,
        joint_housing_radius_m=0.030,
        joint_housing_safety_margin_m=0.005,
    )
    rep = ev.evaluate([offset_record_base_z(rec, offset_m)], top_n=top_n)
    return {
        'foot_min_clearance_m': rep.get('foot_clearance', {}).get('min_clearance_m'),
        'penetration_count': rep.get('ground_penetration_count'),
        'second_joint_min_clearance_m': rep.get('second_joint_clearance', {}).get('min_clearance_m'),
        'inter_leg_collision_count': rep.get('inter_leg_collision_count'),
        'inter_leg_near_count': rep.get('inter_leg_near_count'),
        'housing_collision_count': rep.get('inter_leg_joint_housing_collision_count'),
        'housing_near_count': rep.get('inter_leg_joint_housing_near_count'),
        'worst_foot': rep.get('foot_clearance', {}).get('worst'),
        'worst_second_joint': rep.get('second_joint_clearance', {}).get('worst'),
    }


def model_pose_to_dict(pose):
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w,
        },
    }


def clone_pose(pose):
    from geometry_msgs.msg import Pose
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    out.orientation.x = pose.orientation.x
    out.orientation.y = pose.orientation.y
    out.orientation.z = pose.orientation.z
    out.orientation.w = pose.orientation.w
    return out


def wait_for_model_pose(model_name, timeout_sec):
    import rospy
    from gazebo_msgs.msg import ModelStates
    deadline = time.time() + float(timeout_sec)
    last_err = None
    while time.time() < deadline:
        try:
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
            if model_name in msg.name:
                idx = list(msg.name).index(model_name)
                return msg.pose[idx]
            last_err = 'model not in /gazebo/model_states: %s' % model_name
        except Exception as e:
            last_err = str(e)
    raise RuntimeError(last_err or 'timed out waiting for /gazebo/model_states')


def set_model_pose_z(model_name, base_pose, z_value, reference_frame):
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
    proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = model_name
    state.reference_frame = reference_frame
    state.pose = clone_pose(base_pose)
    state.pose.position.z = float(z_value)
    state.twist.linear.x = 0.0
    state.twist.linear.y = 0.0
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0
    return proxy(state)


def summarize_link_sample(sample):
    links = sample.get('links', [])
    appendix = [l for l in links if str(l.get('name', '')).endswith('appendix_link')]
    second_like = [l for l in links if str(l.get('name', '')).endswith('thigh_link')]
    all_links = links

    def min_item(items):
        if not items:
            return None
        return min(items, key=lambda x: float(x['position'][2]))

    foot = min_item(appendix)
    second = min_item(second_like)
    any_link = min_item(all_links)
    return {
        'gazebo_min_appendix_z_m': None if foot is None else float(foot['position'][2]),
        'gazebo_min_appendix_link': None if foot is None else foot['name'],
        'gazebo_second_like_min_z_m': None if second is None else float(second['position'][2]),
        'gazebo_second_like_min_link': None if second is None else second['name'],
        'gazebo_min_link_z_m': None if any_link is None else float(any_link['position'][2]),
        'gazebo_min_link_name': None if any_link is None else any_link['name'],
        'matched_link_count': len(links),
    }


def main():
    ap = argparse.ArgumentParser(description='Gazebo touchdown hold pose z-offset checker.')
    ap.add_argument('--hold-command-log', default='testdata/entry_touchdown_roll_sequence/touchdown_hold_commands.jsonl')
    ap.add_argument('--output-dir', default='testdata/gazebo_touchdown_pose_check')
    ap.add_argument('--offsets', default='0.013,0.015,0.020')
    ap.add_argument('--model-name', default='lily_octpus')
    ap.add_argument('--relative-model-z', action='store_true', default=True,
                    help='Set model z to initial Gazebo model z + offset. Default: enabled.')
    ap.add_argument('--absolute-model-z', action='store_false', dest='relative_model_z',
                    help='Set model z to record base_pose.z + offset instead of initial Gazebo model z + offset.')
    ap.add_argument('--reference-frame', default='world')
    ap.add_argument('--rate', type=float, default=15.0)
    ap.add_argument('--hold-sec', type=float, default=3.0)
    ap.add_argument('--settle-sec', type=float, default=0.5)
    ap.add_argument('--node-name', default='lily_motion_v3_0_gazebo_touchdown_pose_check')
    ap.add_argument('--dry-run', action='store_true', help='Do not require ROS/Gazebo; write FK-only reports.')
    ap.add_argument('--gazebo-foot-link-regex', default='base_clause_link$|thigh_link$|tibia_link$|appendix_link$')
    args = ap.parse_args()

    ensure_dir(args.output_dir)
    offsets = parse_float_list(args.offsets)
    rec = load_first_record(args.hold_command_log)
    base_pose_z = float((rec.get('base_pose') or {}).get('z', 0.4))
    command = [float(v) for v in rec['joint_command_rad']]

    sample_path = os.path.join(args.output_dir, 'link_state_samples.jsonl')
    rows = []
    sample_count = 0
    gazebo_available = False
    initial_model_pose = None
    initial_model_z = None
    initial_model_pose_error = None
    set_model_state_results = []

    if args.dry_run:
        logger = None
        publisher = None
        rate = None
    else:
        import rospy
        rospy.init_node(args.node_name, anonymous=True)
        publisher = GazeboCommandPublisher()
        rate = rospy.Rate(args.rate)
        logger = GazeboLinkStateLogger(
            sample_path,
            name_regex=args.gazebo_foot_link_regex,
            log_all_links=False,
        )
        logger.start()
        try:
            initial_model_pose = wait_for_model_pose(args.model_name, timeout_sec=5.0)
            initial_model_z = float(initial_model_pose.position.z)
            gazebo_available = True
        except Exception as e:
            initial_model_pose_error = str(e)
            gazebo_available = False

    for offset in offsets:
        fk = fk_touchdown_report(rec, offset)
        target_model_z = None
        set_ok = False
        set_status = ''
        sample_summary = {}
        if not args.dry_run and gazebo_available:
            if args.relative_model_z:
                target_model_z = float(initial_model_z) + float(offset)
            else:
                target_model_z = base_pose_z + float(offset)
            try:
                resp = set_model_pose_z(args.model_name, initial_model_pose, target_model_z, args.reference_frame)
                set_ok = bool(getattr(resp, 'success', False))
                set_status = str(getattr(resp, 'status_message', ''))
            except Exception as e:
                set_ok = False
                set_status = str(e)
            set_model_state_results.append({
                'offset_m': offset,
                'target_model_z_m': target_model_z,
                'success': set_ok,
                'status_message': set_status,
            })
            try:
                rospy.sleep(max(0.0, float(args.settle_sec)))
            except Exception:
                time.sleep(max(0.0, float(args.settle_sec)))
            ticks = max(1, int(round(max(0.0, float(args.hold_sec)) * float(args.rate))))
            for tick in range(ticks):
                publisher.publish(command)
                rate.sleep()
            logger.snapshot({
                'offset_m': offset,
                'target_model_z_m': target_model_z,
                'hold_sec': args.hold_sec,
                'rate_hz': args.rate,
                'set_model_state_success': set_ok,
                'set_model_state_status': set_status,
                'base_pose_z_m': base_pose_z,
                'relative_model_z': args.relative_model_z,
            })
            sample_count = logger.record_count
            try:
                # Re-read the last written sample to summarize exactly what was logged.
                last = None
                with open(sample_path) as f:
                    for line in f:
                        if line.strip():
                            last = json.loads(line)
                if last is not None:
                    sample_summary = summarize_link_sample(last)
            except Exception as e:
                sample_summary = {'error': str(e)}
        row = {
            'offset_m': offset,
            'base_pose_z_plus_offset_m': base_pose_z + float(offset),
            'target_model_z_m': target_model_z,
            'set_model_state_success': set_ok,
            'fk_foot_min_clearance_m': fk['foot_min_clearance_m'],
            'fk_penetration_count': fk['penetration_count'],
            'fk_second_joint_min_clearance_m': fk['second_joint_min_clearance_m'],
            'fk_inter_leg_collision_count': fk['inter_leg_collision_count'],
            'fk_housing_collision_count': fk['housing_collision_count'],
            'gazebo_min_appendix_z_m': sample_summary.get('gazebo_min_appendix_z_m'),
            'gazebo_min_appendix_link': sample_summary.get('gazebo_min_appendix_link'),
            'gazebo_second_like_min_z_m': sample_summary.get('gazebo_second_like_min_z_m'),
            'gazebo_second_like_min_link': sample_summary.get('gazebo_second_like_min_link'),
            'gui_hold_sec': args.hold_sec if (not args.dry_run and set_ok) else 0.0,
            'gui_hold_attempted': bool(not args.dry_run and gazebo_available),
        }
        rows.append(row)

    if not args.dry_run and logger is not None:
        logger.close()

    csv_path = os.path.join(args.output_dir, 'touchdown_pose_offsets.csv')
    fields = [
        'offset_m',
        'base_pose_z_plus_offset_m',
        'target_model_z_m',
        'set_model_state_success',
        'fk_foot_min_clearance_m',
        'fk_penetration_count',
        'fk_second_joint_min_clearance_m',
        'fk_inter_leg_collision_count',
        'fk_housing_collision_count',
        'gazebo_min_appendix_z_m',
        'gazebo_min_appendix_link',
        'gazebo_second_like_min_z_m',
        'gazebo_second_like_min_link',
        'gui_hold_sec',
        'gui_hold_attempted',
    ]
    with open(csv_path, 'w') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)

    issues = []
    if args.dry_run:
        issues.append('Dry-run only: Gazebo model pose was not set and GUI hold was not attempted.')
    elif not gazebo_available:
        issues.append('Could not read model pose for %s from /gazebo/model_states: %s' % (args.model_name, initial_model_pose_error))
    elif any(not r['set_model_state_success'] for r in rows):
        issues.append('At least one /gazebo/set_model_state call failed; inspect summary.json.')
    else:
        issues.append('No Gazebo pose-check blocker found. GUI visual judgment still requires looking at Gazebo during the hold windows.')

    with open(os.path.join(args.output_dir, 'remaining_issues.md'), 'w') as f:
        f.write('# Remaining issues\n\n')
        for issue in issues:
            f.write('- ' + issue + '\n')
        f.write('- This helper publishes only Gazebo joint controller commands; it never opens can0 and never sends Jetson/CAN commands.\n')
        f.write('- Default model z mode is relative to the current Gazebo model pose. Use `--absolute-model-z` only if the Gazebo model origin is known to match command-log base_pose.z.\n')

    summary = {
        'dry_run': args.dry_run,
        'opened_can0': False,
        'sent_to_hardware_can': False,
        'jetson_publish_enabled': False,
        'hold_command_log': args.hold_command_log,
        'output_dir': args.output_dir,
        'offsets_m': offsets,
        'base_pose_z_m': base_pose_z,
        'model_name': args.model_name,
        'relative_model_z': args.relative_model_z,
        'gazebo_available': gazebo_available,
        'initial_model_pose': None if initial_model_pose is None else model_pose_to_dict(initial_model_pose),
        'initial_model_pose_error': initial_model_pose_error,
        'set_model_state_results': set_model_state_results,
        'rows': rows,
        'link_state_sample_count': sample_count,
        'gui_hold_sec_per_offset': args.hold_sec if not args.dry_run else 0.0,
        'outputs': {
            'summary': os.path.join(args.output_dir, 'summary.json'),
            'touchdown_pose_offsets_csv': csv_path,
            'link_state_samples': sample_path,
            'remaining_issues': os.path.join(args.output_dir, 'remaining_issues.md'),
        },
    }
    write_json(os.path.join(args.output_dir, 'summary.json'), summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
