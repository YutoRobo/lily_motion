#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS graph readiness checks used by Lily Gazebo launchers.

This helper is read-only.  It does not start/stop ROS nodes and does not publish
commands.  Python 2.7 / ROS Melodic compatible.
"""
from __future__ import division, print_function

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.interface_config import GAZEBO_JOINT_TOPICS_IN_JOINT_STATE_ORDER

GAZEBO_MODEL_STATES_TOPIC = '/gazebo/model_states'
GAZEBO_WORLD_SERVICE = '/gazebo/get_world_properties'


def canonical_topic(name):
    text = str(name)
    return text if text.startswith('/') else '/' + text


def expected_controller_topics():
    return [canonical_topic(x) for x in GAZEBO_JOINT_TOPICS_IN_JOINT_STATE_ORDER]


def controller_status(subscriber_topics):
    subscribers = set(canonical_topic(x) for x in subscriber_topics)
    expected = expected_controller_topics()
    missing = [topic for topic in expected if topic not in subscribers]
    ready = len(expected) - len(missing)
    return ready, len(expected), missing


def gazebo_ready(publisher_topics, services):
    publishers = set(canonical_topic(x) for x in publisher_topics)
    service_names = set(canonical_topic(x) for x in services)
    return (GAZEBO_MODEL_STATES_TOPIC in publishers and
            GAZEBO_WORLD_SERVICE in service_names)


def get_master_state():
    import rosgraph
    master = rosgraph.Master('/lily_gazebo_runtime_readiness')
    publishers, subscribers, services = master.getSystemState()
    pub_topics = [topic for topic, unused_nodes in publishers]
    sub_topics = [topic for topic, unused_nodes in subscribers]
    service_topics = [topic for topic, unused_nodes in services]
    return pub_topics, sub_topics, service_topics


def wait_for_gazebo(timeout_sec, poll_sec=0.2):
    deadline = time.time() + float(timeout_sec)
    last_error = None
    while True:
        try:
            publishers, unused_subscribers, services = get_master_state()
            if gazebo_ready(publishers, services):
                print('GAZEBO_READY')
                return 0
            last_error = None
        except Exception as exc:
            last_error = str(exc)
        if time.time() >= deadline:
            if last_error:
                print('GAZEBO_NOT_READY master_error=%s' % last_error)
            else:
                print('GAZEBO_NOT_READY missing=%s,%s' % (
                    GAZEBO_MODEL_STATES_TOPIC, GAZEBO_WORLD_SERVICE))
            return 1
        time.sleep(float(poll_sec))


def read_controller_status():
    try:
        unused_publishers, subscribers, unused_services = get_master_state()
    except Exception as exc:
        print('CONTROLLERS_ERROR %s' % exc)
        return 4
    ready, total, missing = controller_status(subscribers)
    if ready == total:
        print('CONTROLLERS_READY %d/%d' % (ready, total))
        return 0
    if ready == 0:
        print('CONTROLLERS_ABSENT 0/%d' % total)
        return 2
    print('CONTROLLERS_PARTIAL %d/%d missing=%s' % (
        ready, total, ','.join(missing)))
    return 3


def wait_for_controllers(timeout_sec, poll_sec=0.2):
    deadline = time.time() + float(timeout_sec)
    last_ready = 0
    total = len(expected_controller_topics())
    last_missing = expected_controller_topics()
    last_error = None
    while True:
        try:
            unused_publishers, subscribers, unused_services = get_master_state()
            last_ready, total, last_missing = controller_status(subscribers)
            if last_ready == total:
                print('CONTROLLERS_READY %d/%d' % (last_ready, total))
                return 0
            last_error = None
        except Exception as exc:
            last_error = str(exc)
        if time.time() >= deadline:
            if last_error:
                print('CONTROLLERS_NOT_READY master_error=%s' % last_error)
            else:
                print('CONTROLLERS_NOT_READY %d/%d missing=%s' % (
                    last_ready, total, ','.join(last_missing)))
            return 1
        time.sleep(float(poll_sec))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Read-only Lily Gazebo ROS readiness checks')
    sub = parser.add_subparsers(dest='command')

    p_gazebo = sub.add_parser('gazebo')
    p_gazebo.add_argument('--timeout-sec', type=float, default=30.0)

    sub.add_parser('controller-status')

    p_controllers = sub.add_parser('controllers')
    p_controllers.add_argument('--timeout-sec', type=float, default=30.0)

    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.command == 'gazebo':
        if args.timeout_sec <= 0.0:
            raise SystemExit('--timeout-sec must be > 0')
        return wait_for_gazebo(args.timeout_sec)
    if args.command == 'controller-status':
        return read_controller_status()
    if args.command == 'controllers':
        if args.timeout_sec <= 0.0:
            raise SystemExit('--timeout-sec must be > 0')
        return wait_for_controllers(args.timeout_sec)
    raise SystemExit('command is required: gazebo | controller-status | controllers')


if __name__ == '__main__':
    raise SystemExit(main())
