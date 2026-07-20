#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import datetime
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import load_command_records, full_command_diagnostics
from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
ARCHIVE_SCRIPT_DIR = os.path.join(ROOT, "archive", "v3_experiment_scripts")
if ARCHIVE_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, ARCHIVE_SCRIPT_DIR)
from run_v3_0_41_second_joint_angle_localization import _analyze_source


SCRIPT_VERSION = "v3.0.provisional_baseline_verify.1"
DEFAULT_COMMAND_LOG = "data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl"
DEFAULT_OUTPUT_DIR = "testdata/provisional_baseline_verify"

EXPECTED_MAX_ADJACENT_DELTA_DEG = 5.56001875294519
EXPECTED_MAX_SECOND_DIFF_DEG = 3.787713927320219
EXPECTED_FOOT_MIN_CLEARANCE_M = -0.03017711684493357
EXPECTED_FOOT_PENETRATION_COUNT = 1986


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _write_json(path, data):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")


def _utc_now_iso():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _git_commit():
    try:
        p = subprocess.Popen(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        out, _ = p.communicate()
        if p.returncode == 0:
            return out.decode("utf-8").strip()
    except Exception:
        pass
    return None


def _max_second_diff_deg(records):
    worst = 0.0
    worst_info = None
    if len(records) < 3:
        return worst, worst_info
    for i in range(2, len(records)):
        q0 = records[i - 2].get("joint_command_rad", [])
        q1 = records[i - 1].get("joint_command_rad", [])
        q2 = records[i].get("joint_command_rad", [])
        for j, (a, b, c) in enumerate(zip(q0, q1, q2)):
            val = abs(math.degrees(float(c) - 2.0 * float(b) + float(a)))
            if val > worst:
                legacy_leg_id, joint_index = JOINT_STATE_ORDER[j]
                worst = val
                worst_info = {
                    "record_index": i,
                    "joint_state_index": j,
                    "legacy_leg_id": legacy_leg_id,
                    "leg_name": LEG_NAMES_BY_ID[legacy_leg_id],
                    "joint_index": joint_index,
                    "joint_name": ["base_clause", "thigh", "tibia"][joint_index],
                    "from_phase_name": records[i - 2].get("phase_name"),
                    "mid_phase_name": records[i - 1].get("phase_name"),
                    "to_phase_name": records[i].get("phase_name"),
                    "from_frame_index": records[i - 2].get("frame_index"),
                    "mid_frame_index": records[i - 1].get("frame_index"),
                    "to_frame_index": records[i].get("frame_index"),
                }
    return worst, worst_info


def _diagnostics_report(command_log, records):
    diag = full_command_diagnostics(records)
    second_diff, second_diff_worst = _max_second_diff_deg(records)
    n = len(records[0]["joint_command_rad"]) if records else 0
    mins = diag.get("mins_rad", [0.0] * n)
    maxs = diag.get("maxs_rad", [0.0] * n)
    deltas = diag.get("deltas_rad", [0.0] * n)
    adj = diag.get("per_joint_max_adjacent_delta_rad", [0.0] * n)
    joints = []
    for i, pair in enumerate(JOINT_STATE_ORDER):
        if i >= n:
            break
        legacy_leg_id, joint_index = pair
        joint_name = ["base_clause", "thigh", "tibia"][joint_index]
        joints.append({
            "index": i,
            "legacy_leg_id": legacy_leg_id,
            "leg_name": LEG_NAMES_BY_ID[legacy_leg_id],
            "joint": joint_name,
            "min_rad": mins[i],
            "max_rad": maxs[i],
            "delta_rad": deltas[i],
            "min_deg": math.degrees(mins[i]),
            "max_deg": math.degrees(maxs[i]),
            "delta_deg": math.degrees(deltas[i]),
            "max_adjacent_delta_rad": adj[i],
            "max_adjacent_delta_deg": math.degrees(adj[i]),
            "worst_transition_index": diag.get("per_joint_worst_transition_index", [None] * n)[i],
        })
    return {
        "command_log": command_log,
        "frame_count": len(records),
        "nonzero_joint_count": diag.get("nonzero_joint_count"),
        "max_delta_rad": diag.get("max_delta_rad"),
        "max_delta_deg": diag.get("max_delta_deg"),
        "max_adjacent_delta_rad": diag.get("max_adjacent_delta_rad"),
        "max_adjacent_delta_deg": diag.get("max_adjacent_delta_deg"),
        "max_second_diff_deg": second_diff,
        "max_second_diff_worst": second_diff_worst,
        "worst_transition": diag.get("worst_transition"),
        "phase_summary": diag.get("phase_summary"),
        "top_joints_by_adjacent_delta": sorted(joints, key=lambda x: x["max_adjacent_delta_rad"], reverse=True)[:8],
        "joints": joints,
    }


def _constraint_report(records, args):
    json_stdout = sys.stdout
    sys.stdout = sys.stderr
    try:
        ev = LegacyConstraintEvaluator(
            second_joint_limit_deg=args.second_joint_abs_max_deg,
            ground_z=args.ground_z,
            ground_tol=args.ground_tolerance,
            inter_leg_limit_m=args.inter_leg_limit,
            default_body_z=args.legacy_body_z,
            leg_radius_m=args.inter_leg_link_radius,
            inter_leg_safety_margin_m=args.inter_leg_safety_margin,
            joint_housing_radius_m=args.inter_leg_joint_housing_radius,
            joint_housing_safety_margin_m=args.inter_leg_joint_housing_safety_margin,
        )
        report = ev.evaluate(records, top_n=args.top_n)
    finally:
        sys.stdout = json_stdout
    report.update({
        "profile": "direct_command_log_constraint_evaluation",
        "resample_factor": 1,
        "smooth_window": 1,
        "note": "The input command log is evaluated directly; no candidate generation, resampling, or smoothing is applied.",
    })
    return report


def _near(value, expected, tolerance):
    return abs(float(value) - float(expected)) <= float(tolerance)


def _gate(name, ok, actual, expected):
    return {
        "name": name,
        "ok": bool(ok),
        "actual": actual,
        "expected": expected,
    }


def _run_gazebo(args, output_dir):
    out_path = os.path.join(output_dir, "gazebo_replay_candidate.json")
    cmd = [
        sys.executable,
        os.path.join(ROOT, "tools", "gazebo", "run_v3_0_gazebo_replay.py"),
        "--command-log", args.command_log,
        "--strict-command-log-input",
        "--rate", str(args.rate),
        "--hold-start-sec", str(args.hold_start_sec),
        "--hold-end-sec", str(args.hold_end_sec),
        "--diagnose-command-log",
        "--candidate-output", out_path,
    ]
    p = subprocess.Popen(cmd, cwd=ROOT)
    rc = p.wait()
    return {
        "enabled": True,
        "command": cmd,
        "returncode": rc,
        "output": out_path,
        "ok": rc == 0,
    }


def build_summary(args, records, second_joint, command_diag, constraint, gazebo_result):
    second_filtered = second_joint["filtered"]
    foot = constraint["foot_clearance"]
    second_clearance = constraint["second_joint_clearance"]

    metrics = {
        "frame_count": len(records),
        "second_joint_max_deg": second_filtered["max_abs_angle_deg"],
        "second_joint_violation_count": second_filtered["violation_count"],
        "max_adjacent_delta_deg": command_diag["max_adjacent_delta_deg"],
        "max_second_diff_deg": command_diag["max_second_diff_deg"],
        "second_joint_penetration_count": second_clearance["penetration_count"],
        "second_joint_min_clearance_m": second_clearance["min_clearance_m"],
        "foot_min_clearance_m": foot["min_clearance_m"],
        "foot_penetration_count": foot["penetration_count"],
        "inter_leg_collision_count": constraint["inter_leg_collision_count"],
        "inter_leg_near_count": constraint["inter_leg_near_count"],
        "inter_leg_joint_housing_collision_count": constraint["inter_leg_joint_housing_collision_count"],
        "inter_leg_joint_housing_near_count": constraint["inter_leg_joint_housing_near_count"],
    }

    hard_gates = [
        _gate("second_joint_max_deg <= 95.0", metrics["second_joint_max_deg"] <= 95.0, metrics["second_joint_max_deg"], "<= 95.0"),
        _gate("second_joint_violation_count == 0", metrics["second_joint_violation_count"] == 0, metrics["second_joint_violation_count"], 0),
        _gate("second_joint_penetration_count == 0", metrics["second_joint_penetration_count"] == 0, metrics["second_joint_penetration_count"], 0),
        _gate("inter_leg_collision_count == 0", metrics["inter_leg_collision_count"] == 0, metrics["inter_leg_collision_count"], 0),
        _gate("inter_leg_joint_housing_collision_count == 0", metrics["inter_leg_joint_housing_collision_count"] == 0, metrics["inter_leg_joint_housing_collision_count"], 0),
        _gate(
            "max_adjacent_delta_deg near expected",
            _near(metrics["max_adjacent_delta_deg"], EXPECTED_MAX_ADJACENT_DELTA_DEG, args.delta_tolerance_deg),
            metrics["max_adjacent_delta_deg"],
            "%s +/- %s" % (EXPECTED_MAX_ADJACENT_DELTA_DEG, args.delta_tolerance_deg),
        ),
        _gate(
            "max_second_diff_deg near expected",
            _near(metrics["max_second_diff_deg"], EXPECTED_MAX_SECOND_DIFF_DEG, args.delta_tolerance_deg),
            metrics["max_second_diff_deg"],
            "%s +/- %s" % (EXPECTED_MAX_SECOND_DIFF_DEG, args.delta_tolerance_deg),
        ),
    ]
    if gazebo_result is not None:
        hard_gates.append(_gate("Gazebo replay returncode == 0", gazebo_result.get("ok"), gazebo_result.get("returncode"), 0))

    hard_ok = all(g["ok"] for g in hard_gates)
    monitored = {
        "foot_min_clearance_m": {
            "value": metrics["foot_min_clearance_m"],
            "expected_reference": EXPECTED_FOOT_MIN_CLEARANCE_M,
            "near_reference": _near(metrics["foot_min_clearance_m"], EXPECTED_FOOT_MIN_CLEARANCE_M, args.clearance_tolerance_m),
            "hard_gate": False,
        },
        "foot_penetration_count": {
            "value": metrics["foot_penetration_count"],
            "expected_reference": EXPECTED_FOOT_PENETRATION_COUNT,
            "matches_reference": metrics["foot_penetration_count"] == EXPECTED_FOOT_PENETRATION_COUNT,
            "hard_gate": False,
        },
        "inter_leg_near_count": {
            "value": metrics["inter_leg_near_count"],
            "hard_gate": False,
        },
        "inter_leg_joint_housing_near_count": {
            "value": metrics["inter_leg_joint_housing_near_count"],
            "hard_gate": False,
        },
    }

    return {
        "input_command_log": args.command_log,
        "generated_at": _utc_now_iso(),
        "script_version": SCRIPT_VERSION,
        "git_commit": _git_commit(),
        "pass_fail": "pass" if hard_ok else "fail",
        "hard_gate_result": {
            "ok": hard_ok,
            "gates": hard_gates,
        },
        "monitored_metrics": monitored,
        "metrics": metrics,
        "gazebo_replay": gazebo_result or {"enabled": False},
        "notes": [
            "This verifier does not generate a new candidate and does not modify the input command log.",
            "Constraint metrics evaluate the command log directly without additional resampling or smoothing.",
            "Foot penetration is monitored, not a hard gate, for candidate02_softlimit_94p8.",
        ],
    }


def main():
    ap = argparse.ArgumentParser(description="Verify the current provisional baseline candidate in one command.")
    ap.add_argument("--command-log", default=DEFAULT_COMMAND_LOG)
    ap.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    ap.add_argument("--second-joint-abs-max-deg", type=float, default=95.0)
    ap.add_argument("--boundary-window", type=int, default=3)
    ap.add_argument("--top-n", type=int, default=30)
    ap.add_argument("--legacy-body-z", type=float, default=0.35)
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--ground-tolerance", type=float, default=1e-4)
    ap.add_argument("--inter-leg-limit", type=float, default=0.04)
    ap.add_argument("--inter-leg-link-radius", type=float, default=0.015)
    ap.add_argument("--inter-leg-safety-margin", type=float, default=0.010)
    ap.add_argument("--inter-leg-joint-housing-radius", type=float, default=0.030)
    ap.add_argument("--inter-leg-joint-housing-safety-margin", type=float, default=0.005)
    ap.add_argument("--delta-tolerance-deg", type=float, default=1e-9)
    ap.add_argument("--clearance-tolerance-m", type=float, default=1e-9)
    ap.add_argument("--with-gazebo", action="store_true")
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--hold-start-sec", type=float, default=2.0)
    ap.add_argument("--hold-end-sec", type=float, default=2.0)
    args = ap.parse_args()

    _ensure_dir(args.output_dir)
    records = load_command_records(args.command_log)

    second_filtered = _analyze_source(
        records,
        "filtered",
        args.second_joint_abs_max_deg,
        args.boundary_window,
        args.top_n,
    )
    second_report = {
        "version_note": "provisional baseline verification second-joint localization",
        "inputs": {
            "filtered_source": args.command_log,
        },
        "joint_order_note": "joint_command_rad is in JOINT_STATE_ORDER. This report evaluates entries whose joint_index == 1.",
        "filtered": second_filtered,
    }
    command_diag = _diagnostics_report(args.command_log, records)
    constraint = _constraint_report(records, args)

    gazebo_result = None
    if args.with_gazebo:
        gazebo_result = _run_gazebo(args, args.output_dir)

    summary = build_summary(args, records, second_report, command_diag, constraint, gazebo_result)

    _write_json(os.path.join(args.output_dir, "second_joint_localization.json"), second_report)
    _write_json(os.path.join(args.output_dir, "command_diagnostics.json"), command_diag)
    _write_json(os.path.join(args.output_dir, "constraint_eval.json"), constraint)
    _write_json(os.path.join(args.output_dir, "summary.json"), summary)

    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["hard_gate_result"]["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
