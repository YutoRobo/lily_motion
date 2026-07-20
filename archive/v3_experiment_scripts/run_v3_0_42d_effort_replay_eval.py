#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Replay an existing command log in Gazebo while recording /joint_states effort.

v3.0.42D helper
----------------
This script is intended for candidate_02 / candidate_03 / case27 verification.
It uses the same command-log replay path as tools/gazebo/run_v3_0_gazebo_replay.py, but it
subscribes to /joint_states and summarizes effort values.

Notes:
- /joint_states.effort is the simulator/controller reported effort, not the
  command log value.
- The metric is sensitive to Gazebo contact, friction, controller gains and
  replay rate. Treat it as a practical acceptance gate, not a pure kinematic
  property.
- Python2 compatible.
"""
from __future__ import print_function

import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

GAZEBO_SCRIPT_DIR = os.path.join(ROOT, "tools", "gazebo")
if GAZEBO_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, GAZEBO_SCRIPT_DIR)

from run_v3_0_gazebo_replay import (  # noqa: E402
    load_command_log_records,
    command_records_to_frames,
    _CommandLogExporter,
    _sleep_rate,
)
from lily_motion_v3.ros_bridge import (  # noqa: E402
    CombinedCommandPublisher,
    GazeboCommandPublisher,
    JetsonJointStatePublisher,
)
from lily_motion_v3.command_resampler import (  # noqa: E402
    resample_command_records,
    moving_average_command_records,
    unwrap_continuous_command_records,
    full_command_diagnostics,
)


def _safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


def _abs(x):
    return abs(float(x))


class EffortRecorder(object):
    def __init__(self, effort_limit=40.0, top_n=20, ignore_joint_name_contains=None,
                 warmup_ignore_sec=0.0):
        self.effort_limit = float(effort_limit)
        self.top_n = int(top_n)
        self.ignore_joint_name_contains = [s for s in (ignore_joint_name_contains or []) if s]
        self.warmup_ignore_sec = float(warmup_ignore_sec)
        self.started_wall_time = time.time()
        self.current_context = {}
        self.sample_count = 0
        self.message_count = 0
        self.empty_effort_message_count = 0
        self.exceed_count = 0
        self.max_abs_effort = 0.0
        self.max_abs_sample = None
        self.top_samples = []
        self.by_joint = {}
        self.by_phase = {}
        self.by_roll_phase = {}
        self.first_stamp_sec = None
        self.last_stamp_sec = None

    def set_context(self, frame, published_index, hold_index, hold_ticks, hold_region):
        self.current_context = {
            "published_index": int(published_index),
            "hold_index": int(hold_index),
            "hold_ticks": int(hold_ticks),
            "hold_region": str(hold_region),
            "frame_index": int(getattr(frame, "frame_index", -1)),
            "phase_name": str(getattr(frame, "phase_name", "")),
            "phase_index": int(getattr(frame, "phase_index", 0) or 0),
            "phase_step_index": int(getattr(frame, "phase_step_index", 0) or 0),
            "phase_step_count": int(getattr(frame, "phase_step_count", 1) or 1),
            "roll_index": getattr(frame, "rec", {}).get("roll_index", None),
            "roll_surface_transition": getattr(frame, "rec", {}).get("roll_surface_transition", ""),
        }

    def _ignored_joint(self, name):
        if not self.ignore_joint_name_contains:
            return False
        text = str(name)
        for s in self.ignore_joint_name_contains:
            if s in text:
                return True
        return False

    def _stamp_sec(self, msg):
        try:
            return float(msg.header.stamp.to_sec())
        except Exception:
            return None

    def _record_group(self, table, key, effort_abs, exceed):
        rec = table.get(key)
        if rec is None:
            rec = {
                "key": key,
                "sample_count": 0,
                "exceed_count": 0,
                "max_abs_effort": 0.0,
            }
            table[key] = rec
        rec["sample_count"] += 1
        if exceed:
            rec["exceed_count"] += 1
        if effort_abs > rec["max_abs_effort"]:
            rec["max_abs_effort"] = effort_abs

    def _push_top_sample(self, sample):
        if self.top_n <= 0:
            return
        self.top_samples.append(sample)
        self.top_samples.sort(key=lambda s: s.get("abs_effort", 0.0), reverse=True)
        if len(self.top_samples) > self.top_n:
            self.top_samples = self.top_samples[:self.top_n]

    def callback(self, msg):
        now = time.time()
        if self.warmup_ignore_sec > 0 and (now - self.started_wall_time) < self.warmup_ignore_sec:
            return
        self.message_count += 1
        effort = list(getattr(msg, "effort", []) or [])
        names = list(getattr(msg, "name", []) or [])
        if not effort:
            self.empty_effort_message_count += 1
            return
        stamp_sec = self._stamp_sec(msg)
        if stamp_sec is not None:
            if self.first_stamp_sec is None:
                self.first_stamp_sec = stamp_sec
            self.last_stamp_sec = stamp_sec
        n = min(len(effort), len(names)) if names else len(effort)
        ctx = dict(self.current_context)
        phase_name = ctx.get("phase_name", "")
        roll_index = ctx.get("roll_index", None)
        roll_phase_key = "%s|%s" % (str(roll_index), phase_name)
        for i in range(n):
            joint_name = str(names[i]) if i < len(names) else "joint_%d" % i
            if self._ignored_joint(joint_name):
                continue
            val = _safe_float(effort[i], 0.0)
            av = abs(val)
            exceed = av > self.effort_limit
            self.sample_count += 1
            if exceed:
                self.exceed_count += 1
            if av > self.max_abs_effort:
                sample = dict(ctx)
                sample.update({
                    "joint_name": joint_name,
                    "joint_index_in_joint_states": i,
                    "effort": val,
                    "abs_effort": av,
                    "limit": self.effort_limit,
                    "excess": max(0.0, av - self.effort_limit),
                    "stamp_sec": stamp_sec,
                })
                self.max_abs_effort = av
                self.max_abs_sample = sample
            sample2 = dict(ctx)
            sample2.update({
                "joint_name": joint_name,
                "joint_index_in_joint_states": i,
                "effort": val,
                "abs_effort": av,
                "limit": self.effort_limit,
                "excess": max(0.0, av - self.effort_limit),
                "stamp_sec": stamp_sec,
            })
            self._push_top_sample(sample2)
            self._record_group(self.by_joint, joint_name, av, exceed)
            self._record_group(self.by_phase, phase_name, av, exceed)
            self._record_group(self.by_roll_phase, roll_phase_key, av, exceed)

    def _sorted_group_list(self, table, limit=50):
        rows = []
        for _k, rec in table.items():
            r = dict(rec)
            cnt = float(r.get("sample_count", 0) or 0)
            r["exceed_rate"] = float(r.get("exceed_count", 0) or 0) / cnt if cnt > 0 else 0.0
            rows.append(r)
        rows.sort(key=lambda r: (r.get("max_abs_effort", 0.0), r.get("exceed_count", 0)), reverse=True)
        return rows[:limit]

    def summary(self):
        sample_count = float(self.sample_count or 0)
        return {
            "effort_limit": self.effort_limit,
            "effort_limit_ok": self.max_abs_effort <= self.effort_limit if self.sample_count > 0 else False,
            "message_count": self.message_count,
            "empty_effort_message_count": self.empty_effort_message_count,
            "sample_count": self.sample_count,
            "exceed_count": self.exceed_count,
            "exceed_rate": float(self.exceed_count) / sample_count if sample_count > 0 else None,
            "max_abs_effort": self.max_abs_effort,
            "max_abs_sample": self.max_abs_sample,
            "top_samples": self.top_samples,
            "by_joint_top": self._sorted_group_list(self.by_joint, limit=50),
            "by_phase_top": self._sorted_group_list(self.by_phase, limit=20),
            "by_roll_phase_top": self._sorted_group_list(self.by_roll_phase, limit=30),
            "first_stamp_sec": self.first_stamp_sec,
            "last_stamp_sec": self.last_stamp_sec,
            "duration_from_joint_state_stamp_sec": (
                self.last_stamp_sec - self.first_stamp_sec
                if self.first_stamp_sec is not None and self.last_stamp_sec is not None else None
            ),
        }


def publish_frames_with_context(frames, exporter, publisher, rate, recorder,
                                repeat_last=0, rate_hz=30.0, frame_hold_sec=0.0,
                                hold_start_sec=0.0, hold_end_sec=0.0,
                                verbose=False):
    count = 0
    last_frame = None
    last_cmd = None
    rate_hz = float(rate_hz) if rate_hz and rate_hz > 0 else 30.0
    hold_ticks = max(1, int(round(max(0.0, float(frame_hold_sec)) * rate_hz)))
    start_ticks = max(0, int(round(max(0.0, float(hold_start_sec)) * rate_hz)))
    end_ticks = max(0, int(round(max(0.0, float(hold_end_sec)) * rate_hz)))

    if frames and start_ticks > 0:
        first_frame = frames[0]
        first_cmd = exporter.frame_to_joint_state_order(first_frame)
        for k in range(start_ticks):
            recorder.set_context(first_frame, count, k, start_ticks, "hold_start")
            publisher.publish(first_cmd)
            count += 1
            _sleep_rate(rate, dry_sleep=False, sleep_sec=1.0 / rate_hz)

    for frame in frames:
        cmd = exporter.frame_to_joint_state_order(frame)
        if verbose:
            print("publish frame=%s phase=%s step=%s hold_ticks=%d" % (
                getattr(frame, "frame_index", None),
                getattr(frame, "phase_name", ""),
                getattr(frame, "phase_step_index", None),
                hold_ticks))
        for k in range(hold_ticks):
            recorder.set_context(frame, count, k, hold_ticks, "replay")
            publisher.publish(cmd)
            last_frame = frame
            last_cmd = cmd
            count += 1
            _sleep_rate(rate, dry_sleep=False, sleep_sec=1.0 / rate_hz)

    total_last_ticks = max(0, int(repeat_last)) + end_ticks
    for k in range(total_last_ticks):
        if last_cmd is None:
            break
        recorder.set_context(last_frame, count, k, total_last_ticks, "hold_end")
        publisher.publish(last_cmd)
        count += 1
        _sleep_rate(rate, dry_sleep=False, sleep_sec=1.0 / rate_hz)
    return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--command-log", required=True)
    ap.add_argument("--strict-command-log-input", action="store_true")
    ap.add_argument("--output", default="")
    ap.add_argument("--effort-limit", type=float, default=40.0)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--frame-hold-sec", type=float, default=0.10)
    ap.add_argument("--hold-start-sec", type=float, default=2.0)
    ap.add_argument("--hold-end-sec", type=float, default=2.0)
    ap.add_argument("--repeat-last", type=int, default=20)
    ap.add_argument("--resample-factor", type=int, default=1)
    ap.add_argument("--smooth-window", type=int, default=1)
    ap.add_argument("--segment-key", default="")
    ap.add_argument("--unwrap-continuous-angles", action="store_true")
    ap.add_argument("--diagnose-command-log", action="store_true")
    ap.add_argument("--node-name", default="lily_motion_v3_0_42d_effort_replay_eval")
    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--jetson", action="store_true")
    ap.add_argument("--top-n", type=int, default=20)
    ap.add_argument("--warmup-ignore-sec", type=float, default=0.0,
                    help="Ignore /joint_states samples for this many seconds after node start")
    ap.add_argument("--ignore-joint-name-contains", default="",
                    help="Comma-separated substrings. Matching joint names are ignored in effort summary.")
    ap.add_argument("--verbose-publish", action="store_true")
    args = ap.parse_args()

    if args.strict_command_log_input and not os.path.exists(args.command_log):
        raise IOError("--strict-command-log-input: command log does not exist: %s" % args.command_log)
    if not os.path.exists(args.command_log):
        raise IOError("command log does not exist: %s" % args.command_log)

    records = load_command_log_records(args.command_log)
    if args.unwrap_continuous_angles:
        records = unwrap_continuous_command_records(records)
    if args.resample_factor and args.resample_factor > 1:
        records = resample_command_records(records, factor=args.resample_factor,
                                           segment_key=(args.segment_key.strip() or None))
    if args.smooth_window and args.smooth_window > 1:
        records = moving_average_command_records(records, window=args.smooth_window,
                                                segment_key=(args.segment_key.strip() or None))
    command_log_diagnostics = full_command_diagnostics(records) if args.diagnose_command_log else None
    frames = command_records_to_frames(records)
    exporter = _CommandLogExporter()

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node(args.node_name, anonymous=True)
    ignore_parts = []
    for p in str(args.ignore_joint_name_contains).split(','):
        p = p.strip()
        if p:
            ignore_parts.append(p)
    recorder = EffortRecorder(
        effort_limit=args.effort_limit,
        top_n=args.top_n,
        ignore_joint_name_contains=ignore_parts,
        warmup_ignore_sec=args.warmup_ignore_sec)
    sub = rospy.Subscriber(args.joint_states_topic, JointState, recorder.callback, queue_size=200)

    gazebo_pub = GazeboCommandPublisher()
    jetson_pub = JetsonJointStatePublisher() if args.jetson else None
    publisher = CombinedCommandPublisher(gazebo_publisher=gazebo_pub, jetson_publisher=jetson_pub)
    rate = rospy.Rate(args.rate)

    published_count = publish_frames_with_context(
        frames, exporter, publisher, rate, recorder,
        repeat_last=args.repeat_last,
        rate_hz=args.rate,
        frame_hold_sec=args.frame_hold_sec,
        hold_start_sec=args.hold_start_sec,
        hold_end_sec=args.hold_end_sec,
        verbose=args.verbose_publish)

    # Allow a few trailing joint_states samples after final command.
    time.sleep(0.25)
    try:
        sub.unregister()
    except Exception:
        pass

    summary = {
        "schema_version": "v3.0.42D-effort-replay-eval-1",
        "command_log": args.command_log,
        "preview_frame_count": len(frames),
        "published_count": published_count,
        "rate_hz": args.rate,
        "frame_hold_sec": args.frame_hold_sec,
        "hold_start_sec": args.hold_start_sec,
        "hold_end_sec": args.hold_end_sec,
        "repeat_last": args.repeat_last,
        "estimated_playback_sec": float(published_count) / float(args.rate) if args.rate else None,
        "resample_factor": args.resample_factor,
        "smooth_window": args.smooth_window,
        "segment_key": args.segment_key,
        "unwrap_continuous_angles": args.unwrap_continuous_angles,
        "joint_states_topic": args.joint_states_topic,
        "command_log_diagnostics": command_log_diagnostics,
        "effort_summary": recorder.summary(),
        "acceptance_hint": {
            "target_max_abs_effort": args.effort_limit,
            "passed_effort_limit": recorder.summary().get("effort_limit_ok", False),
            "note": "Final gait acceptance still requires visual check. If effort exceeds limit, inspect top_samples by phase/joint and compare candidate_02/candidate_03/case27 under identical Gazebo conditions."
        }
    }

    text = json.dumps(summary, indent=2, sort_keys=True)
    print(text)
    if args.output:
        out_dir = os.path.dirname(args.output)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        with open(args.output, "w") as f:
            f.write(text)
            f.write("\n")
        print("effort_report_output:", args.output)


if __name__ == "__main__":
    main()
