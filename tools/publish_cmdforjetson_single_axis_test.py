#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish a one-axis out-and-back test on the production /cmdForJetson path.

The message always contains 24 positions. Only the selected axis is finite;
all other elements are NaN. The unified StateMachine validates only Use=True
axes, so an accidentally active non-target axis rejects the whole frame instead
of receiving an unintended zero-position command.

This publisher never opens SocketCAN and does not issue ALIGN, HOME, or RUN.
Complete those operations in the normal UI before starting this program.
"""
from __future__ import division, print_function

import argparse
import math
import sys


POSITION_COUNT = 24
DEFAULT_TOPIC = "/cmdForJetson"


def _is_finite(value):
    return not (math.isnan(value) or math.isinf(value))


def build_offsets(direction, amplitude_rad, step_rad):
    """Return out-and-back offsets, including initial and final zero."""
    amplitude = float(amplitude_rad)
    step = float(step_rad)
    if not _is_finite(amplitude) or not _is_finite(step):
        raise ValueError("amplitude and step must be finite")
    if amplitude <= 0.0 or step <= 0.0:
        raise ValueError("amplitude and step must be positive")
    count_float = amplitude / step
    count = int(round(count_float))
    if count <= 0 or abs(count_float - count) > 1e-9:
        raise ValueError("amplitude must be an exact multiple of step")
    sign = 1.0 if direction == "plus" else -1.0
    outward = [sign * step * index for index in range(0, count + 1)]
    returning = [sign * step * index for index in range(count - 1, -1, -1)]
    return outward + returning, count


def build_position(axis, value):
    """Build a 24-element safety-masked position vector."""
    if not 0 <= int(axis) < POSITION_COUNT:
        raise ValueError("axis must be in 0..23")
    value = float(value)
    if not _is_finite(value):
        raise ValueError("selected-axis value must be finite")
    positions = [float("nan")] * POSITION_COUNT
    positions[int(axis)] = value
    return positions


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Publish a one-axis test as 24-element JointState messages to "
            "/cmdForJetson"))
    parser.add_argument("--axis", type=int, required=True,
                        help="target axis number, 0..23")
    parser.add_argument("--direction", choices=("plus", "minus"),
                        default="plus")
    parser.add_argument("--center-rad", type=float, default=0.0,
                        help="logical q0 after SET HOME; normally 0.0 rad")
    parser.add_argument("--amplitude-rad", type=float, default=0.020)
    parser.add_argument("--step-rad", type=float, default=0.005)
    parser.add_argument("--period-sec", type=float, default=0.100)
    parser.add_argument("--peak-hold-sec", type=float, default=0.500)
    parser.add_argument("--start-hold-sec", type=float, default=0.500)
    parser.add_argument("--end-hold-sec", type=float, default=0.500)
    parser.add_argument("--subscriber-wait-sec", type=float, default=0.500)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    return parser.parse_args(argv)


def validate_args(args):
    if not 0 <= args.axis < POSITION_COUNT:
        raise SystemExit("--axis must be in 0..23")
    for name in ("center_rad", "amplitude_rad", "step_rad", "period_sec",
                 "peak_hold_sec", "start_hold_sec", "end_hold_sec",
                 "subscriber_wait_sec"):
        value = float(getattr(args, name))
        if not _is_finite(value):
            raise SystemExit("--%s must be finite" % name.replace("_", "-"))
    if args.period_sec <= 0.0:
        raise SystemExit("--period-sec must be positive")
    for name in ("peak_hold_sec", "start_hold_sec", "end_hold_sec",
                 "subscriber_wait_sec"):
        if getattr(args, name) < 0.0:
            raise SystemExit("--%s must be non-negative" %
                             name.replace("_", "-"))
    try:
        offsets, peak_index = build_offsets(
            args.direction, args.amplitude_rad, args.step_rad)
    except ValueError as exc:
        raise SystemExit(str(exc))
    return offsets, peak_index


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    offsets, peak_index = validate_args(args)

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node("cmdforjetson_single_axis_test", anonymous=True)
    publisher = rospy.Publisher(args.topic, JointState, queue_size=10)
    rospy.sleep(args.subscriber_wait_sec)

    rospy.logwarn(
        "Single-axis /cmdForJetson test: confirm only axis%d is Use=True; "
        "all non-target values are NaN safety guards", args.axis)

    for index, offset in enumerate(offsets):
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = build_position(args.axis, args.center_rad + offset)
        publisher.publish(msg)

        if index == 0:
            rospy.sleep(args.start_hold_sec)
        elif index == peak_index:
            rospy.sleep(args.peak_hold_sec)
        else:
            rospy.sleep(args.period_sec)

    rospy.sleep(args.end_hold_sec)
    rospy.loginfo(
        "single-axis /cmdForJetson test complete axis=%d direction=%s "
        "center=%.6f amplitude=%.6f returned_to_center=%s",
        args.axis, args.direction, args.center_rad, args.amplitude_rad,
        not rospy.is_shutdown())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
