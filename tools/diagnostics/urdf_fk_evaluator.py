#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Evaluate command JSONL with URDF FK and the legacy fallback FK."""
from __future__ import division

import argparse
import csv
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.geometry import distance, segment_segment_distance
from lily_motion_v3.interface_config import LEG_NAMES_BY_ID
from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.urdf_kinematics import UrdfLegKinematics


SEGMENT_RADIUS_M = {
    "root_to_coxa_end": 0.05,
    "coxa_end_to_knee": 0.04,
    "knee_to_foot": 0.03,
}
LEG_ID_BY_NAME = dict((name, leg_id) for leg_id, name in LEG_NAMES_BY_ID.items())


def _read_jsonl(path):
    if not os.path.isfile(path):
        raise ValueError("command log file does not exist: %s" % path)
    rows = []
    try:
        with open(path, "r") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except ValueError as exc:
                    raise ValueError(
                        "invalid JSON in command log %s line %d: %s" %
                        (path, line_number, exc))
                if not isinstance(record, dict):
                    raise ValueError(
                        "command log %s line %d must be a JSON object" %
                        (path, line_number))
                record["_line_number"] = line_number
                rows.append(record)
    except IOError as exc:
        raise ValueError("cannot read command log %s: %s" % (path, exc))
    if not rows:
        raise ValueError("command log contains no frames: %s" % path)
    return rows


def _ensure_dir(path):
    if os.path.exists(path) and not os.path.isdir(path):
        raise ValueError("output path is not a directory: %s" % path)
    if not os.path.isdir(path):
        os.makedirs(path)


def _minimum_interleg_distance(segments):
    best = None
    for index_a in range(len(segments)):
        for index_b in range(index_a + 1, len(segments)):
            segment_a = segments[index_a]
            segment_b = segments[index_b]
            if segment_a["leg_name"] == segment_b["leg_name"]:
                continue
            raw_distance, closest_a, closest_b = segment_segment_distance(
                segment_a["a"], segment_a["b"],
                segment_b["a"], segment_b["b"])
            clearance = (
                raw_distance
                - SEGMENT_RADIUS_M[segment_a["segment_name"]]
                - SEGMENT_RADIUS_M[segment_b["segment_name"]])
            candidate = {
                "raw_distance_m": raw_distance,
                "clearance_m": clearance,
                "leg_a": segment_a["leg_name"],
                "segment_a": segment_a["segment_name"],
                "leg_b": segment_b["leg_name"],
                "segment_b": segment_b["segment_name"],
                "closest_a": closest_a,
                "closest_b": closest_b,
            }
            if best is None or clearance < best["clearance_m"]:
                best = candidate
    return best


def _fallback_differences(kinematics, fallback_model, record, urdf_positions):
    q_by_leg = kinematics.command_q_by_leg(record)
    base_pose = record.get("base_pose") or {}
    differences = {}
    for leg_name in kinematics.leg_names:
        leg_id = LEG_ID_BY_NAME[leg_name]
        fallback_foot = fallback_model.foot_position_world(
            leg_id, q_by_leg[leg_name], base_pose)
        urdf_foot = urdf_positions[leg_name]["foot"]
        differences[leg_name] = {
            "distance_m": distance(urdf_foot, fallback_foot),
            "urdf_foot": urdf_foot,
            "fallback_foot": fallback_foot,
        }
    return differences


def _frame_diagnostics(kinematics, fallback_model, record):
    positions = kinematics.link_positions_world_from_record(record)
    segments = kinematics.leg_segments_world_from_record(record)
    foot_z_by_leg = dict(
        (leg_name, positions[leg_name]["foot"][2])
        for leg_name in kinematics.leg_names)
    fallback = _fallback_differences(
        kinematics, fallback_model, record, positions)
    return {
        "positions": positions,
        "foot_z_by_leg": foot_z_by_leg,
        "minimum_foot_z_m": min(foot_z_by_leg.values()),
        "minimum_interleg": _minimum_interleg_distance(segments),
        "fallback_by_leg": fallback,
        "maximum_fallback_difference_m": max(
            item["distance_m"] for item in fallback.values()),
    }


def _write_json(path, value):
    with open(path, "w") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _write_chain_summary(kinematics, out_dir):
    chains = dict(
        (leg_name, kinematics.leg_joint_summary(leg_name))
        for leg_name in kinematics.leg_names)
    _write_json(os.path.join(out_dir, "joint_chains.json"), chains)
    return chains


def evaluate(robot_description, command_log, out_dir, label):
    kinematics = UrdfLegKinematics.from_robot_description(robot_description)
    frames = _read_jsonl(command_log)
    _ensure_dir(out_dir)
    chains = _write_chain_summary(kinematics, out_dir)
    fallback_model = RobotModel()

    csv_path = os.path.join(out_dir, "frame_diagnostics.csv")
    fields = [
        "line_number", "frame_index", "minimum_foot_z_m",
        "minimum_interleg_distance_m", "minimum_clearance_m",
        "maximum_fallback_difference_m",
    ]
    minimum_foot = None
    minimum_interleg = None
    maximum_fallback = None
    with open(csv_path, "w") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in frames:
            diagnostics = _frame_diagnostics(
                kinematics, fallback_model, record)
            interleg = diagnostics["minimum_interleg"]
            writer.writerow({
                "line_number": record["_line_number"],
                "frame_index": record.get("frame_index", ""),
                "minimum_foot_z_m": "%.15g" % diagnostics["minimum_foot_z_m"],
                "minimum_interleg_distance_m": (
                    "" if interleg is None else
                    "%.15g" % interleg["raw_distance_m"]),
                "minimum_clearance_m": (
                    "" if interleg is None else
                    "%.15g" % interleg["clearance_m"]),
                "maximum_fallback_difference_m": "%.15g" %
                diagnostics["maximum_fallback_difference_m"],
            })
            if minimum_foot is None or diagnostics["minimum_foot_z_m"] < minimum_foot:
                minimum_foot = diagnostics["minimum_foot_z_m"]
            if interleg is not None and (
                    minimum_interleg is None or
                    interleg["raw_distance_m"] < minimum_interleg["raw_distance_m"]):
                minimum_interleg = dict(interleg)
                minimum_interleg["line_number"] = record["_line_number"]
            if maximum_fallback is None or diagnostics["maximum_fallback_difference_m"] > maximum_fallback:
                maximum_fallback = diagnostics["maximum_fallback_difference_m"]

    summary = {
        "label": label,
        "robot_description": robot_description,
        "command_log": command_log,
        "frame_count": len(frames),
        "leg_names": list(kinematics.leg_names),
        "joint_chain_count": len(chains),
        "minimum_foot_z_m": minimum_foot,
        "minimum_interleg": minimum_interleg,
        "maximum_urdf_vs_fallback_foot_difference_m": maximum_fallback,
        "segment_radius_proxy_m": SEGMENT_RADIUS_M,
    }
    _write_json(os.path.join(out_dir, "summary.json"), summary)
    return summary


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Evaluate a 24-axis command JSONL using URDF FK")
    parser.add_argument("--robot-description", required=True)
    parser.add_argument("--command-log", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--label", required=True)
    return parser


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return evaluate(
            args.robot_description, args.command_log,
            args.out_dir, args.label)
    except (IOError, OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
