#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish a safe one-leg three-axis test through /cmdForJetson.

The message always contains 24 positions. The three axes of the selected leg
are finite in every frame and every other axis is NaN. Therefore the operator
must set exactly those three axes Use=True for coordinated testing.

This publisher never opens SocketCAN and never issues ALIGN, HOME, RUN, or
STOP. Complete those operations through the normal UI before starting it.
"""
from __future__ import division, print_function

import argparse
import math
import sys


POSITION_COUNT = 24
JOINTS_PER_LEG = 3
LEG_COUNT = 8
DEFAULT_TOPIC = "/cmdForJetson"
MAX_TEST_AMPLITUDE_RAD = 0.020
JOINT_LIMITS_RAD = (
    (-6.283185307179586, 6.283185307179586),
    (-1.6580627893946132, 1.6580627893946132),
    (-2.6179938779914944, 2.6179938779914944),
)


def _is_finite(value):
    return not (math.isnan(value) or math.isinf(value))


def parse_triplet(text, option_name):
    parts = [part.strip() for part in str(text).split(",")]
    if len(parts) != JOINTS_PER_LEG:
        raise ValueError(
            "%s must contain exactly three comma-separated values" %
            option_name)
    try:
        values = [float(part) for part in parts]
    except Exception:
        raise ValueError("%s contains a non-numeric value" % option_name)
    if not all(_is_finite(value) for value in values):
        raise ValueError("%s values must be finite" % option_name)
    return values


def build_offsets(direction, amplitude_rad, step_rad):
    if direction not in ("plus", "minus"):
        raise ValueError("direction must be plus or minus")
    amplitude = float(amplitude_rad)
    step = float(step_rad)
    if not _is_finite(amplitude) or not _is_finite(step):
        raise ValueError("amplitude and step must be finite")
    if amplitude <= 0.0 or step <= 0.0:
        raise ValueError("amplitude and step must be positive")
    if amplitude > MAX_TEST_AMPLITUDE_RAD + 1e-12:
        raise ValueError(
            "amplitude exceeds the %.3f rad test limit" %
            MAX_TEST_AMPLITUDE_RAD)
    count_float = amplitude / step
    count = int(round(count_float))
    if count <= 0 or abs(count_float - count) > 1e-9:
        raise ValueError("amplitude must be an exact multiple of step")
    sign = 1.0 if direction == "plus" else -1.0
    outward = [sign * step * index for index in range(0, count + 1)]
    returning = [sign * step * index
                 for index in range(count - 1, -1, -1)]
    return outward + returning, count


def build_leg_axes(leg_index):
    leg = int(leg_index)
    if not 0 <= leg < LEG_COUNT:
        raise ValueError("leg-index must be in 0..7")
    first = leg * JOINTS_PER_LEG
    return [first, first + 1, first + 2]


def build_position(axes, values):
    if len(axes) != JOINTS_PER_LEG or len(values) != JOINTS_PER_LEG:
        raise ValueError("exactly three axes and values are required")
    positions = [float("nan")] * POSITION_COUNT
    seen = set()
    for axis, value in zip(axes, values):
        axis = int(axis)
        value = float(value)
        if not 0 <= axis < POSITION_COUNT:
            raise ValueError("axis must be in 0..23")
        if axis in seen:
            raise ValueError("axes must be unique")
        if not _is_finite(value):
            raise ValueError("active-axis values must be finite")
        positions[axis] = value
        seen.add(axis)
    return positions


def _append_motion(steps, axes, centers, moving_indexes, direction,
                   amplitude_rad, step_rad, period_sec, peak_hold_sec,
                   between_motion_hold_sec, label):
    offsets, peak_index = build_offsets(direction, amplitude_rad, step_rad)
    for offset_index, offset in enumerate(offsets[1:]):
        values = list(centers)
        for moving_index in moving_indexes:
            values[moving_index] += offset
        hold_sec = (peak_hold_sec
                    if offset_index + 1 == peak_index else period_sec)
        if offset_index + 1 == len(offsets) - 1:
            hold_sec = between_motion_hold_sec
        steps.append({
            "positions": build_position(axes, values),
            "hold_sec": float(hold_sec),
            "label": "%s_%s" % (label, direction),
        })


def build_sequence(leg_index, mode, direction, centers_rad,
                   amplitude_rad, step_rad, period_sec, start_hold_sec,
                   peak_hold_sec, between_motion_hold_sec, end_hold_sec):
    if mode not in ("individual", "coordinated", "all"):
        raise ValueError("mode must be individual, coordinated, or all")
    if direction not in ("plus", "minus", "both"):
        raise ValueError("direction must be plus, minus, or both")
    axes = build_leg_axes(leg_index)
    centers = [float(value) for value in centers_rad]
    if (len(centers) != JOINTS_PER_LEG
            or not all(_is_finite(value) for value in centers)):
        raise ValueError("centers must contain three finite values")
    directions = (("plus", "minus")
                  if direction == "both" else (direction,))
    for joint_index, center in enumerate(centers):
        lo, hi = JOINT_LIMITS_RAD[joint_index]
        if center - amplitude_rad < lo or center + amplitude_rad > hi:
            raise ValueError(
                "joint%d center +/- amplitude exceeds the software limit" %
                joint_index)

    steps = [{
        "positions": build_position(axes, centers),
        "hold_sec": float(start_hold_sec),
        "label": "start_center",
    }]
    for one_direction in directions:
        if mode in ("individual", "all"):
            for moving_index, axis in enumerate(axes):
                _append_motion(
                    steps, axes, centers, (moving_index,), one_direction,
                    amplitude_rad, step_rad, period_sec, peak_hold_sec,
                    between_motion_hold_sec,
                    "individual_axis%d" % axis)
        if mode in ("coordinated", "all"):
            _append_motion(
                steps, axes, centers, (0, 1, 2), one_direction,
                amplitude_rad, step_rad, period_sec, peak_hold_sec,
                between_motion_hold_sec, "coordinated")
    steps.append({
        "positions": build_position(axes, centers),
        "hold_sec": float(end_hold_sec),
        "label": "end_center",
    })
    return axes, steps


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish a one-leg three-axis test to /cmdForJetson")
    parser.add_argument("--leg-index", type=int, required=True,
                        help="zero-based leg index, 0..7")
    parser.add_argument("--mode",
                        choices=("individual", "coordinated", "all"),
                        default="individual")
    parser.add_argument("--direction", choices=("plus", "minus", "both"),
                        default="plus")
    parser.add_argument("--centers-rad", default="0,0,0",
                        help="base,thigh,tibia logical centers after SET HOME")
    parser.add_argument("--amplitude-rad", type=float, default=0.002)
    parser.add_argument("--step-rad", type=float, default=0.001)
    parser.add_argument("--period-sec", type=float, default=0.500)
    parser.add_argument("--start-hold-sec", type=float, default=1.000)
    parser.add_argument("--peak-hold-sec", type=float, default=1.000)
    parser.add_argument("--between-motion-hold-sec", type=float,
                        default=1.000)
    parser.add_argument("--end-hold-sec", type=float, default=1.000)
    parser.add_argument("--subscriber-wait-sec", type=float, default=0.500)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    return parser.parse_args(argv)


def validate_args(args):
    try:
        centers = parse_triplet(args.centers_rad, "--centers-rad")
        build_leg_axes(args.leg_index)
        build_offsets("plus", args.amplitude_rad, args.step_rad)
    except ValueError as exc:
        raise SystemExit(str(exc))
    for name in ("period_sec", "start_hold_sec", "peak_hold_sec",
                 "between_motion_hold_sec", "end_hold_sec",
                 "subscriber_wait_sec"):
        value = float(getattr(args, name))
        if not _is_finite(value):
            raise SystemExit(
                "--%s must be finite" % name.replace("_", "-"))
        if name == "period_sec" and value <= 0.0:
            raise SystemExit("--period-sec must be positive")
        if name != "period_sec" and value < 0.0:
            raise SystemExit(
                "--%s must be non-negative" % name.replace("_", "-"))
    return centers


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    centers = validate_args(args)
    axes, steps = build_sequence(
        args.leg_index, args.mode, args.direction, centers,
        args.amplitude_rad, args.step_rad, args.period_sec,
        args.start_hold_sec, args.peak_hold_sec,
        args.between_motion_hold_sec, args.end_hold_sec)

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node("cmdforjetson_one_leg_test", anonymous=True)
    publisher = rospy.Publisher(args.topic, JointState, queue_size=10)
    rospy.sleep(args.subscriber_wait_sec)
    rospy.logwarn(
        "One-leg /cmdForJetson test: confirm exactly axes %s are Use=True; "
        "all other positions are NaN safety guards", axes)

    for index, step in enumerate(steps):
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = step["positions"]
        publisher.publish(msg)
        rospy.loginfo("one-leg frame=%d/%d label=%s", index + 1,
                      len(steps), step["label"])
        rospy.sleep(step["hold_sec"])

    rospy.loginfo(
        "one-leg /cmdForJetson test complete leg=%d axes=%s mode=%s "
        "direction=%s returned_to_center=%s",
        args.leg_index, axes, args.mode, args.direction,
        not rospy.is_shutdown())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
