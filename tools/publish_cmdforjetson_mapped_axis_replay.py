#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Map one logical axis from a 24-axis JSONL log onto one physical axis.

The source trajectory is converted to displacement from its first sample,
scaled, optionally inverted, and bounded by a small test limit. The published
JointState always has 24 positions, with only the physical target axis finite.
Exactly that physical axis must be Use=True.

This publisher never opens SocketCAN and never issues ALIGN, HOME, RUN, or
STOP. It is a waveform and communication-path test, not an absolute-pose test.
"""
from __future__ import division, print_function

import argparse
import json
import math
import sys


POSITION_COUNT = 24
POSITION_KEYS = ("joint_command_rad", "position", "joint_positions_rad")
DEFAULT_TOPIC = "/cmdForJetson"
MAX_TEST_LIMIT_RAD = 0.020
JOINT_LIMITS_RAD = (
    (-6.283185307179586, 6.283185307179586),
    (-1.6580627893946132, 1.6580627893946132),
    (-2.6179938779914944, 2.6179938779914944),
)


def _is_finite(value):
    return not (math.isnan(value) or math.isinf(value))


def extract_position(record, record_index):
    for key in POSITION_KEYS:
        if key in record:
            positions = record[key]
            if not isinstance(positions, list):
                raise ValueError(
                    "record %d key %s is not a list" %
                    (record_index, key))
            if len(positions) != POSITION_COUNT:
                raise ValueError(
                    "record %d key %s length %d != %d" %
                    (record_index, key, len(positions), POSITION_COUNT))
            values = [float(value) for value in positions]
            return values, key
    raise ValueError(
        "record %d has none of %s" %
        (record_index, ",".join(POSITION_KEYS)))


def load_axis_samples(path, logical_axis, start_index=0, max_frames=None):
    axis = int(logical_axis)
    if not 0 <= axis < POSITION_COUNT:
        raise ValueError("logical-axis must be in 0..23")
    samples = []
    source_keys = set()
    with open(path) as source:
        for line_index, line in enumerate(source):
            line = line.strip()
            if not line:
                continue
            if line_index < start_index:
                continue
            record = json.loads(line)
            positions, source_key = extract_position(record, line_index)
            value = float(positions[axis])
            if not _is_finite(value):
                raise ValueError(
                    "record %d logical axis %d is non-finite" %
                    (line_index, axis))
            samples.append(value)
            source_keys.add(source_key)
            if max_frames is not None and len(samples) >= max_frames:
                break
    if not samples:
        raise ValueError("no source samples were loaded")
    return samples, sorted(source_keys)


def map_samples(samples, center_rad, scale, limit_rad, invert=False):
    if not samples:
        raise ValueError("samples must not be empty")
    center = float(center_rad)
    scale_value = float(scale)
    limit = float(limit_rad)
    if not all(_is_finite(value)
               for value in (center, scale_value, limit)):
        raise ValueError("center, scale, and limit must be finite")
    if scale_value <= 0.0:
        raise ValueError("scale must be positive")
    if limit <= 0.0 or limit > MAX_TEST_LIMIT_RAD + 1e-12:
        raise ValueError(
            "limit must be in (0, %.3f] rad" % MAX_TEST_LIMIT_RAD)
    baseline = float(samples[0])
    sign = -1.0 if invert else 1.0
    mapped = []
    clipped_count = 0
    raw_delta_min = None
    raw_delta_max = None
    for source_value in samples:
        source_value = float(source_value)
        if not _is_finite(source_value):
            raise ValueError("source samples must be finite")
        raw_delta = sign * scale_value * (source_value - baseline)
        raw_delta_min = (raw_delta if raw_delta_min is None
                         else min(raw_delta_min, raw_delta))
        raw_delta_max = (raw_delta if raw_delta_max is None
                         else max(raw_delta_max, raw_delta))
        bounded_delta = max(-limit, min(limit, raw_delta))
        if abs(bounded_delta - raw_delta) > 1e-15:
            clipped_count += 1
        mapped.append(center + bounded_delta)
    return {
        "baseline_source_rad": baseline,
        "mapped_values_rad": mapped,
        "clipped_count": clipped_count,
        "raw_delta_min_rad": raw_delta_min,
        "raw_delta_max_rad": raw_delta_max,
        "mapped_min_rad": min(mapped),
        "mapped_max_rad": max(mapped),
    }


def build_position(physical_axis, value):
    axis = int(physical_axis)
    value = float(value)
    if not 0 <= axis < POSITION_COUNT:
        raise ValueError("physical-axis must be in 0..23")
    if not _is_finite(value):
        raise ValueError("physical-axis value must be finite")
    positions = [float("nan")] * POSITION_COUNT
    positions[axis] = value
    return positions


def build_return_values(current_rad, center_rad, step_rad):
    current = float(current_rad)
    center = float(center_rad)
    step = float(step_rad)
    if not all(_is_finite(value)
               for value in (current, center, step)):
        raise ValueError("return values must be finite")
    if step <= 0.0:
        raise ValueError("return-step-rad must be positive")
    if abs(current - center) <= 1e-15:
        return [center]
    direction = 1.0 if center > current else -1.0
    values = []
    value = current
    while abs(center - value) > step + 1e-15:
        value += direction * step
        values.append(value)
    values.append(center)
    return values


def validate_physical_range(physical_axis, center_rad, limit_rad):
    joint_index = int(physical_axis) % 3
    lo, hi = JOINT_LIMITS_RAD[joint_index]
    if center_rad - limit_rad < lo or center_rad + limit_rad > hi:
        raise ValueError(
            "physical center +/- limit exceeds the StateMachine joint limit")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Map one logical JSONL axis onto one physical "
            "/cmdForJetson axis"))
    parser.add_argument("--command-log", required=True)
    parser.add_argument("--logical-axis", type=int, required=True)
    parser.add_argument("--physical-axis", type=int, required=True)
    parser.add_argument(
        "--confirm-physical-axis", type=int, required=True,
        help="must exactly match --physical-axis")
    parser.add_argument("--rate", type=float, required=True,
                        help="publish rate in Hz")
    parser.add_argument("--center-rad", type=float, default=0.0)
    parser.add_argument("--scale", type=float, default=0.05)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--limit-rad", type=float, default=0.010)
    parser.add_argument("--return-step-rad", type=float, default=0.001)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--start-hold-sec", type=float, default=1.0)
    parser.add_argument("--end-hold-sec", type=float, default=1.0)
    parser.add_argument("--subscriber-wait-sec", type=float, default=0.5)
    parser.add_argument("--allow-clipping", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    return parser.parse_args(argv)


def validate_args(args):
    if not 0 <= args.logical_axis < POSITION_COUNT:
        raise SystemExit("--logical-axis must be in 0..23")
    if not 0 <= args.physical_axis < POSITION_COUNT:
        raise SystemExit("--physical-axis must be in 0..23")
    if args.confirm_physical_axis != args.physical_axis:
        raise SystemExit(
            "--confirm-physical-axis must match --physical-axis")
    for name in ("rate", "center_rad", "scale", "limit_rad",
                 "return_step_rad", "start_hold_sec", "end_hold_sec",
                 "subscriber_wait_sec"):
        value = float(getattr(args, name))
        if not _is_finite(value):
            raise SystemExit(
                "--%s must be finite" % name.replace("_", "-"))
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.scale <= 0.0:
        raise SystemExit("--scale must be positive")
    if args.limit_rad <= 0.0 or args.limit_rad > MAX_TEST_LIMIT_RAD:
        raise SystemExit(
            "--limit-rad must be in (0, %.3f]" % MAX_TEST_LIMIT_RAD)
    if args.return_step_rad <= 0.0:
        raise SystemExit("--return-step-rad must be positive")
    if args.start_index < 0:
        raise SystemExit("--start-index must be >= 0")
    if args.max_frames is not None and args.max_frames <= 0:
        raise SystemExit("--max-frames must be positive")
    for name in ("start_hold_sec", "end_hold_sec",
                 "subscriber_wait_sec"):
        if getattr(args, name) < 0.0:
            raise SystemExit(
                "--%s must be non-negative" % name.replace("_", "-"))
    try:
        validate_physical_range(
            args.physical_axis, args.center_rad, args.limit_rad)
    except ValueError as exc:
        raise SystemExit(str(exc))


def print_summary(args, samples, source_keys, mapping):
    print("mapped-axis replay summary")
    print("  command_log: %s" % args.command_log)
    print("  logical_axis: %d" % args.logical_axis)
    print("  physical_axis: %d" % args.physical_axis)
    print("  samples: %d" % len(samples))
    print("  source_keys: %s" % ",".join(source_keys))
    print("  baseline_source_rad: %.9f" %
          mapping["baseline_source_rad"])
    print("  scale: %.9f" % args.scale)
    print("  invert: %s" % bool(args.invert))
    print("  raw_delta_range_rad: [%.9f, %.9f]" %
          (mapping["raw_delta_min_rad"],
           mapping["raw_delta_max_rad"]))
    print("  mapped_range_rad: [%.9f, %.9f]" %
          (mapping["mapped_min_rad"], mapping["mapped_max_rad"]))
    print("  limit_rad: %.9f" % args.limit_rad)
    print("  clipped_count: %d" % mapping["clipped_count"])


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    validate_args(args)
    try:
        samples, source_keys = load_axis_samples(
            args.command_log, args.logical_axis, args.start_index,
            args.max_frames)
        mapping = map_samples(
            samples, args.center_rad, args.scale, args.limit_rad,
            invert=args.invert)
    except (ValueError, IOError, OSError, TypeError) as exc:
        raise SystemExit(str(exc))
    print_summary(args, samples, source_keys, mapping)
    if args.dry_run:
        return 0
    if mapping["clipped_count"] and not args.allow_clipping:
        raise SystemExit(
            "mapped trajectory clips the test limit; reduce --scale or pass "
            "--allow-clipping after reviewing --dry-run")

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node("cmdforjetson_mapped_axis_replay", anonymous=True)
    publisher = rospy.Publisher(args.topic, JointState, queue_size=10)
    rospy.sleep(args.subscriber_wait_sec)
    rospy.logwarn(
        "Mapped-axis /cmdForJetson replay: confirm exactly physical axis%d "
        "is Use=True; source logical axis%d is relative-scaled and bounded",
        args.physical_axis, args.logical_axis)

    rate = rospy.Rate(args.rate)
    mapped_values = mapping["mapped_values_rad"]
    for index, value in enumerate(mapped_values):
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = build_position(args.physical_axis, value)
        publisher.publish(msg)
        if index == 0:
            rospy.sleep(args.start_hold_sec)
        else:
            rate.sleep()

    if not rospy.is_shutdown():
        for value in build_return_values(
                mapped_values[-1], args.center_rad,
                args.return_step_rad):
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.position = build_position(args.physical_axis, value)
            publisher.publish(msg)
            rate.sleep()
        rospy.sleep(args.end_hold_sec)

    rospy.loginfo(
        "mapped-axis replay complete logical=%d physical=%d frames=%d "
        "clipped=%d returned_to_center=%s",
        args.logical_axis, args.physical_axis, len(mapped_values),
        mapping["clipped_count"], not rospy.is_shutdown())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
