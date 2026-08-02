#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish a single-axis diagnostic sequence to the PC StateMachine.

This process never opens SocketCAN and never imports the UI.
"""
from __future__ import print_function

import argparse
import time

import rospy
from std_msgs.msg import String

from motion_check import DEFAULT_MOTION_CHECK_CONFIG, build_motion_values


TOPIC = "/can/axis_command"


def offset_sequence(direction):
    sign = 1 if direction == "plus" else -1
    return build_motion_values(0.0, sign, DEFAULT_MOTION_CHECK_CONFIG)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Request selected-axis Diagnostic RUN and position offsets")
    parser.add_argument("--axis", type=int, required=True)
    parser.add_argument(
        "--direction", choices=("plus", "minus"), default="plus")
    parser.add_argument("--run-wait-sec", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()
    if not 0 <= args.axis <= 23:
        raise SystemExit("--axis must be in 0..23")
    if args.run_wait_sec < 0.0:
        raise SystemExit("--run-wait-sec must be non-negative")

    rospy.init_node("single_axis_external_test_publisher")
    publisher = rospy.Publisher(TOPIC, String, queue_size=10)
    time.sleep(0.2)

    publisher.publish("diagnostic_run:%d" % args.axis)
    time.sleep(args.run_wait_sec)

    values = offset_sequence(args.direction)
    outward_count = int(round(
        DEFAULT_MOTION_CHECK_CONFIG.amplitude_rad
        / DEFAULT_MOTION_CHECK_CONFIG.step_rad))
    for index, offset in enumerate(values):
        if index == outward_count:
            time.sleep(DEFAULT_MOTION_CHECK_CONFIG.end_hold_sec)
        publisher.publish("position_offset:%d:%.9f" % (args.axis, offset))
        time.sleep(DEFAULT_MOTION_CHECK_CONFIG.command_period_sec)
    time.sleep(DEFAULT_MOTION_CHECK_CONFIG.end_hold_sec)
    rospy.loginfo(
        "external single-axis test submitted axis=%d direction=%s returned_offset=0",
        args.axis, args.direction)


if __name__ == "__main__":
    main()
