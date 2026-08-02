#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish a configurable single-axis test through the CAN StateMachine.

This process never opens SocketCAN and never imports the UI.  It uses the
existing /can/axis_command interface, so Diagnostic RUN, joint-limit, command
jump, MCU-error, and STOP checks remain owned by StateMachine.
"""
from __future__ import print_function

import argparse
import math
import time

import rospy
from std_msgs.msg import String


TOPIC = "/can/axis_command"


def build_offset_sequence(direction, amplitude_rad, step_rad):
    """Return an out-and-back offset sequence ending exactly at zero."""
    direction_sign = 1.0 if direction == "plus" else -1.0
    amplitude_rad = float(amplitude_rad)
    step_rad = float(step_rad)
    if not all(math.isfinite(v) if hasattr(math, "isfinite") else not (
            math.isnan(v) or math.isinf(v))
            for v in (amplitude_rad, step_rad)):
        raise ValueError("amplitude and step must be finite")
    if amplitude_rad <= 0.0 or step_rad <= 0.0:
        raise ValueError("amplitude and step must be positive")
    step_count_float = amplitude_rad / step_rad
    step_count = int(round(step_count_float))
    if step_count <= 0 or abs(step_count_float - step_count) > 1e-9:
        raise ValueError("amplitude must be an exact multiple of step")

    outward = [direction_sign * step_rad * index
               for index in range(1, step_count + 1)]
    returning = [direction_sign * step_rad * index
                 for index in range(step_count - 1, -1, -1)]
    return outward + returning, step_count


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Move exactly one selected axis through the existing CAN "
            "StateMachine diagnostic interface"))
    parser.add_argument("--axis", type=int, required=True,
                        help="CAN axis number, 0..23")
    parser.add_argument("--direction", choices=("plus", "minus"),
                        default="plus")
    parser.add_argument("--amplitude-rad", type=float, default=0.020)
    parser.add_argument("--step-rad", type=float, default=0.005)
    parser.add_argument("--period-sec", type=float, default=0.100)
    parser.add_argument("--peak-hold-sec", type=float, default=0.500)
    parser.add_argument("--end-hold-sec", type=float, default=0.500)
    parser.add_argument("--run-wait-sec", type=float, default=0.500)
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument(
        "--no-diagnostic-run", action="store_true",
        help="Do not send diagnostic_run; use only when RUN was already sent")
    return parser.parse_args()


def validate_args(args):
    if not 0 <= args.axis <= 23:
        raise SystemExit("--axis must be in 0..23")
    for name in ("period_sec", "run_wait_sec", "peak_hold_sec",
                 "end_hold_sec"):
        value = float(getattr(args, name))
        if math.isnan(value) or math.isinf(value) or value < 0.0:
            raise SystemExit("--%s must be finite and non-negative" %
                             name.replace("_", "-"))
    if args.period_sec <= 0.0:
        raise SystemExit("--period-sec must be positive")
    try:
        return build_offset_sequence(
            args.direction, args.amplitude_rad, args.step_rad)
    except ValueError as exc:
        raise SystemExit(str(exc))


def main():
    args = parse_args()
    values, outward_count = validate_args(args)

    rospy.init_node("single_axis_command_test_publisher", anonymous=True)
    publisher = rospy.Publisher(args.topic, String, queue_size=10)
    time.sleep(0.2)

    if not args.no_diagnostic_run:
        publisher.publish("diagnostic_run:%d" % args.axis)
        time.sleep(args.run_wait_sec)

    for index, offset in enumerate(values):
        if index == outward_count:
            time.sleep(args.peak_hold_sec)
        publisher.publish(
            "position_offset:%d:%.9f" % (args.axis, offset))
        time.sleep(args.period_sec)

    time.sleep(args.end_hold_sec)
    rospy.loginfo(
        "single-axis command test submitted axis=%d direction=%s "
        "amplitude=%.6f step=%.6f returned_offset=0",
        args.axis, args.direction, args.amplitude_rad, args.step_rad)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
