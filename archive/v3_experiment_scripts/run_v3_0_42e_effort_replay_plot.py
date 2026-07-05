#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Replay a command log in Gazebo, record /joint_states.effort, and save plots.

v3.0.42E helper
----------------
This is a graph-enabled extension of v3.0.42D.
It publishes an existing filtered command log, subscribes to /joint_states,
records effort time-series, and writes:

- JSON summary
- CSV time-series of max abs effort per JointState message
- CSV long-format per-joint effort samples (optional, enabled by default)
- PNG plots:
    1. max abs effort vs time
    2. top joint max effort bar chart
    3. top phase max effort bar chart
    4. effort heatmap for top joints

Notes:
- /joint_states.effort is simulator/controller reported effort, not a command-log
  value. It is sensitive to Gazebo contact, friction, PID gains, and replay rate.
- For revolute joints it should normally be interpreted as torque; for prismatic
  joints as force. The script keeps the ROS term "effort" in outputs.
- Python2 compatible.
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


def _mkdir_p(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


class EffortPlotRecorder(object):
    def __init__(self, effort_limit=40.0, top_n=20, ignore_joint_name_contains=None,
                 warmup_ignore_sec=0.0, store_joint_series=True, max_joint_series_samples=250000):
        self.effort_limit = float(effort_limit)
        self.top_n = int(top_n)
        self.ignore_joint_name_contains = [s for s in (ignore_joint_name_contains or []) if s]
        self.warmup_ignore_sec = float(warmup_ignore_sec)
        self.store_joint_series = bool(store_joint_series)
        self.max_joint_series_samples = int(max_joint_series_samples)
        self.started_wall_time = time.time()
        self.first_sample_wall_time = None
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
        self.time_series = []
        self.joint_series = []
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
                "sum_abs_effort": 0.0,
            }
            table[key] = rec
        rec["sample_count"] += 1
        rec["sum_abs_effort"] += effort_abs
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
        if self.first_sample_wall_time is None:
            self.first_sample_wall_time = now
        elapsed_wall = now - self.first_sample_wall_time
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
            elapsed_stamp = stamp_sec - self.first_stamp_sec
        else:
            elapsed_stamp = elapsed_wall
        n = min(len(effort), len(names)) if names else len(effort)
        ctx = dict(self.current_context)
        phase_name = ctx.get("phase_name", "")
        roll_index = ctx.get("roll_index", None)
        roll_phase_key = "%s|%s" % (str(roll_index), phase_name)

        message_max_abs = 0.0
        message_max_joint = ""
        message_max_effort = 0.0
        message_exceed = False

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
                message_exceed = True
            if av > message_max_abs:
                message_max_abs = av
                message_max_joint = joint_name
                message_max_effort = val
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
                    "elapsed_sec": elapsed_stamp,
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
                "elapsed_sec": elapsed_stamp,
            })
            self._push_top_sample(sample2)
            self._record_group(self.by_joint, joint_name, av, exceed)
            self._record_group(self.by_phase, phase_name, av, exceed)
            self._record_group(self.by_roll_phase, roll_phase_key, av, exceed)
            if self.store_joint_series and len(self.joint_series) < self.max_joint_series_samples:
                self.joint_series.append({
                    "elapsed_sec": elapsed_stamp,
                    "stamp_sec": stamp_sec,
                    "joint_name": joint_name,
                    "joint_index": i,
                    "effort": val,
                    "abs_effort": av,
                    "exceed": 1 if exceed else 0,
                    "phase_name": phase_name,
                    "roll_index": roll_index,
                    "frame_index": ctx.get("frame_index", -1),
                    "hold_region": ctx.get("hold_region", ""),
                })

        row = dict(ctx)
        row.update({
            "elapsed_sec": elapsed_stamp,
            "stamp_sec": stamp_sec,
            "max_abs_effort": message_max_abs,
            "max_effort": message_max_effort,
            "max_joint_name": message_max_joint,
            "exceed": 1 if message_exceed else 0,
        })
        self.time_series.append(row)

    def _sorted_group_list(self, table, limit=50):
        rows = []
        for _k, rec in table.items():
            r = dict(rec)
            cnt = float(r.get("sample_count", 0) or 0)
            r["exceed_rate"] = float(r.get("exceed_count", 0) or 0) / cnt if cnt > 0 else 0.0
            r["mean_abs_effort"] = float(r.get("sum_abs_effort", 0.0) or 0.0) / cnt if cnt > 0 else 0.0
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
            "time_series_sample_count": len(self.time_series),
            "joint_series_sample_count": len(self.joint_series),
            "joint_series_truncated": bool(self.store_joint_series and len(self.joint_series) >= self.max_joint_series_samples),
            "max_abs_effort": self.max_abs_effort,
            "max_abs_sample": self.max_abs_sample,
            "top_samples": self.top_samples,
            "by_joint_top": self._sorted_group_list(self.by_joint, limit=80),
            "by_phase_top": self._sorted_group_list(self.by_phase, limit=30),
            "by_roll_phase_top": self._sorted_group_list(self.by_roll_phase, limit=40),
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
            print("publish frame=%s phase=%s step=%s" % (
                getattr(frame, "frame_index", None),
                getattr(frame, "phase_name", ""),
                getattr(frame, "phase_step_index", None)))
        hold_ticks = max(1, int(round(max(0.0, float(frame_hold_sec)) * rate_hz)))
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


def write_time_series_csv(path, rows):
    if not path:
        return
    _mkdir_p(os.path.dirname(path))
    fields = [
        "elapsed_sec", "stamp_sec", "max_abs_effort", "max_effort", "max_joint_name", "exceed",
        "published_index", "frame_index", "roll_index", "phase_name", "phase_step_index",
        "phase_step_count", "hold_region", "hold_index", "hold_ticks"
    ]
    with open(path, "w") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            out = {}
            for k in fields:
                out[k] = r.get(k, "")
            w.writerow(out)


def write_joint_series_csv(path, rows):
    if not path:
        return
    _mkdir_p(os.path.dirname(path))
    fields = [
        "elapsed_sec", "stamp_sec", "joint_name", "joint_index", "effort", "abs_effort", "exceed",
        "frame_index", "roll_index", "phase_name", "hold_region"
    ]
    with open(path, "w") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            out = {}
            for k in fields:
                out[k] = r.get(k, "")
            w.writerow(out)


def _import_matplotlib():
    import matplotlib
    try:
        matplotlib.use("Agg")
    except Exception:
        pass
    import matplotlib.pyplot as plt
    return plt


def save_plots(plot_dir, recorder, effort_limit, top_joint_count=8, prefix="effort"):
    if not plot_dir:
        return []
    _mkdir_p(plot_dir)
    paths = []
    plt = _import_matplotlib()

    ts = list(recorder.time_series)
    if ts:
        x = [float(r.get("elapsed_sec", 0.0) or 0.0) for r in ts]
        y = [float(r.get("max_abs_effort", 0.0) or 0.0) for r in ts]
        fig = plt.figure(figsize=(12, 5))
        ax = fig.add_subplot(111)
        ax.plot(x, y, linewidth=1.0, label="max abs effort")
        ax.axhline(float(effort_limit), linestyle="--", linewidth=1.0, label="limit %.1f" % effort_limit)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Effort abs max")
        ax.set_title("Max absolute /joint_states.effort over time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        path = os.path.join(plot_dir, "%s_time_series.png" % prefix)
        fig.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(path)

    by_joint = recorder._sorted_group_list(recorder.by_joint, limit=max(1, int(top_joint_count)))
    if by_joint:
        labels = [str(r.get("key", "")) for r in by_joint]
        vals = [float(r.get("max_abs_effort", 0.0) or 0.0) for r in by_joint]
        fig = plt.figure(figsize=(12, max(4, 0.35 * len(labels) + 2)))
        ax = fig.add_subplot(111)
        ypos = list(range(len(labels)))
        ax.barh(ypos, vals)
        ax.set_yticks(ypos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        ax.axvline(float(effort_limit), linestyle="--", linewidth=1.0, label="limit %.1f" % effort_limit)
        ax.set_xlabel("Max abs effort")
        ax.set_title("Top joints by max absolute effort")
        ax.grid(True, axis="x", alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        path = os.path.join(plot_dir, "%s_top_joints.png" % prefix)
        fig.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(path)

    by_phase = recorder._sorted_group_list(recorder.by_phase, limit=12)
    if by_phase:
        labels = [str(r.get("key", "")) for r in by_phase]
        vals = [float(r.get("max_abs_effort", 0.0) or 0.0) for r in by_phase]
        fig = plt.figure(figsize=(12, max(4, 0.38 * len(labels) + 2)))
        ax = fig.add_subplot(111)
        ypos = list(range(len(labels)))
        ax.barh(ypos, vals)
        ax.set_yticks(ypos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        ax.axvline(float(effort_limit), linestyle="--", linewidth=1.0, label="limit %.1f" % effort_limit)
        ax.set_xlabel("Max abs effort")
        ax.set_title("Top phases by max absolute effort")
        ax.grid(True, axis="x", alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        path = os.path.join(plot_dir, "%s_top_phases.png" % prefix)
        fig.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(path)

    if recorder.joint_series and by_joint:
        top_names = [str(r.get("key", "")) for r in by_joint[:max(1, int(top_joint_count))]]
        # Downsample into bins for a compact heatmap.
        t0 = None
        t1 = None
        for r in recorder.joint_series:
            if str(r.get("joint_name", "")) in top_names:
                t = float(r.get("elapsed_sec", 0.0) or 0.0)
                t0 = t if t0 is None else min(t0, t)
                t1 = t if t1 is None else max(t1, t)
        if t0 is not None and t1 is not None and t1 >= t0:
            bin_count = 160
            if t1 == t0:
                t1 = t0 + 1.0
            matrix = []
            for _name in top_names:
                matrix.append([0.0 for _ in range(bin_count)])
            index = dict((name, i) for i, name in enumerate(top_names))
            for r in recorder.joint_series:
                name = str(r.get("joint_name", ""))
                if name not in index:
                    continue
                t = float(r.get("elapsed_sec", 0.0) or 0.0)
                b = int((t - t0) / (t1 - t0) * (bin_count - 1))
                if b < 0:
                    b = 0
                if b >= bin_count:
                    b = bin_count - 1
                av = float(r.get("abs_effort", 0.0) or 0.0)
                row_i = index[name]
                if av > matrix[row_i][b]:
                    matrix[row_i][b] = av
            fig = plt.figure(figsize=(12, max(4, 0.35 * len(top_names) + 2)))
            ax = fig.add_subplot(111)
            im = ax.imshow(matrix, aspect="auto", interpolation="nearest", extent=[t0, t1, len(top_names), 0])
            ax.set_yticks([i + 0.5 for i in range(len(top_names))])
            ax.set_yticklabels(top_names)
            ax.set_xlabel("Time [s]")
            ax.set_title("Top-joint effort heatmap")
            cbar = fig.colorbar(im, ax=ax)
            cbar.set_label("Abs effort")
            fig.tight_layout()
            path = os.path.join(plot_dir, "%s_heatmap_top_joints.png" % prefix)
            fig.savefig(path, dpi=150)
            plt.close(fig)
            paths.append(path)

    return paths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--command-log", required=True)
    ap.add_argument("--strict-command-log-input", action="store_true")
    ap.add_argument("--output", default="")
    ap.add_argument("--plot-dir", default="")
    ap.add_argument("--wait-for-joint-states-sec", type=float, default=5.0,
                    help="Preflight wait for one /joint_states message before replay. Set 0 to skip.")
    ap.add_argument("--pre-replay-wait-sec", type=float, default=1.0,
                    help="Sleep after subscriber setup before starting command replay.")
    ap.add_argument("--post-replay-wait-sec", type=float, default=1.0,
                    help="Sleep after replay before unregistering subscriber.")
    ap.add_argument("--csv-output", default="")
    ap.add_argument("--joint-csv-output", default="")
    ap.add_argument("--no-joint-csv", action="store_true")
    ap.add_argument("--max-joint-series-samples", type=int, default=250000)
    ap.add_argument("--plot-top-joints", type=int, default=8)
    ap.add_argument("--plot-prefix", default="effort")
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
    ap.add_argument("--node-name", default="lily_motion_v3_0_42e_effort_replay_plot")
    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--jetson", action="store_true")
    ap.add_argument("--top-n", type=int, default=20)
    ap.add_argument("--warmup-ignore-sec", type=float, default=0.0)
    ap.add_argument("--ignore-joint-name-contains", default="")
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

    # If --plot-dir is omitted but --output is present, make a deterministic
    # plot directory next to the JSON report.  This avoids the easy-to-miss
    # case where only JSON is requested and no PNG/CSV files are produced.
    if (not args.plot_dir) and args.output:
        args.plot_dir = os.path.splitext(args.output)[0] + "_plots"

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node(args.node_name, anonymous=True)
    ignore_parts = []
    for p in str(args.ignore_joint_name_contains).split(','):
        p = p.strip()
        if p:
            ignore_parts.append(p)
    recorder = EffortPlotRecorder(
        effort_limit=args.effort_limit,
        top_n=args.top_n,
        ignore_joint_name_contains=ignore_parts,
        warmup_ignore_sec=args.warmup_ignore_sec,
        store_joint_series=(not args.no_joint_csv),
        max_joint_series_samples=args.max_joint_series_samples)
    sub = rospy.Subscriber(args.joint_states_topic, JointState, recorder.callback, queue_size=300)

    preflight_joint_states = {
        "topic": args.joint_states_topic,
        "wait_sec": args.wait_for_joint_states_sec,
        "received": False,
        "name_count": None,
        "position_count": None,
        "velocity_count": None,
        "effort_count": None,
        "effort_nonempty": False,
        "error": None,
    }
    if args.wait_for_joint_states_sec and args.wait_for_joint_states_sec > 0:
        try:
            msg0 = rospy.wait_for_message(args.joint_states_topic, JointState,
                                          timeout=float(args.wait_for_joint_states_sec))
            names0 = list(getattr(msg0, "name", []) or [])
            effort0 = list(getattr(msg0, "effort", []) or [])
            preflight_joint_states.update({
                "received": True,
                "name_count": len(names0),
                "position_count": len(list(getattr(msg0, "position", []) or [])),
                "velocity_count": len(list(getattr(msg0, "velocity", []) or [])),
                "effort_count": len(effort0),
                "effort_nonempty": bool(len(effort0) > 0),
                "first_names": names0[:8],
            })
            if not effort0:
                print("WARNING: /joint_states was received, but effort[] was empty. "
                      "No effort plot can be generated unless the controller/Gazebo populates effort.")
        except Exception as e:
            preflight_joint_states["error"] = str(e)
            print("WARNING: could not receive /joint_states before replay: %s" % str(e))

    gazebo_pub = GazeboCommandPublisher()
    jetson_pub = JetsonJointStatePublisher() if args.jetson else None
    publisher = CombinedCommandPublisher(gazebo_publisher=gazebo_pub, jetson_publisher=jetson_pub)
    rate = rospy.Rate(args.rate)

    if args.pre_replay_wait_sec and args.pre_replay_wait_sec > 0:
        rospy.sleep(float(args.pre_replay_wait_sec))

    published_count = publish_frames_with_context(
        frames, exporter, publisher, rate, recorder,
        repeat_last=args.repeat_last,
        rate_hz=args.rate,
        frame_hold_sec=args.frame_hold_sec,
        hold_start_sec=args.hold_start_sec,
        hold_end_sec=args.hold_end_sec,
        verbose=args.verbose_publish)

    if args.post_replay_wait_sec and args.post_replay_wait_sec > 0:
        rospy.sleep(float(args.post_replay_wait_sec))
    else:
        rospy.sleep(0.25)

    no_effort_data_warning = None
    if recorder.sample_count == 0:
        if recorder.message_count == 0:
            no_effort_data_warning = (
                "No /joint_states messages were captured during replay. "
                "Check that Gazebo is running, the topic name is correct, and the script is running in the same ROS_MASTER_URI."
            )
        else:
            no_effort_data_warning = (
                "JointState messages were captured, but effort[] was empty or all samples were ignored. "
                "Check whether /joint_states.effort is populated by your Gazebo controller, or remove ignore filters."
            )
        print("WARNING: %s" % no_effort_data_warning)

    try:
        sub.unregister()
    except Exception:
        pass

    plot_paths = []
    plot_error = None
    if args.plot_dir:
        try:
            plot_paths = save_plots(args.plot_dir, recorder, args.effort_limit,
                                    top_joint_count=args.plot_top_joints,
                                    prefix=args.plot_prefix)
        except Exception as e:
            plot_error = str(e)

    csv_output = args.csv_output
    joint_csv_output = args.joint_csv_output
    if args.plot_dir:
        if not csv_output:
            csv_output = os.path.join(args.plot_dir, "%s_time_series.csv" % args.plot_prefix)
        if not joint_csv_output and not args.no_joint_csv:
            joint_csv_output = os.path.join(args.plot_dir, "%s_joint_series.csv" % args.plot_prefix)
    if csv_output:
        write_time_series_csv(csv_output, recorder.time_series)
    if joint_csv_output and not args.no_joint_csv:
        write_joint_series_csv(joint_csv_output, recorder.joint_series)

    summary = {
        "schema_version": "v3.0.42E-effort-replay-plot-1",
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
        "preflight_joint_states": preflight_joint_states,
        "data_collection_warning": no_effort_data_warning,
        "command_log_diagnostics": command_log_diagnostics,
        "effort_summary": recorder.summary(),
        "outputs": {
            "json_output": args.output,
            "plot_dir": args.plot_dir,
            "plot_paths": plot_paths,
            "plot_error": plot_error,
            "csv_output": csv_output,
            "joint_csv_output": joint_csv_output,
        },
        "acceptance_hint": {
            "target_max_abs_effort": args.effort_limit,
            "passed_effort_limit": recorder.summary().get("effort_limit_ok", False),
            "note": "Peak effort can be inflated by simulation/contact. Inspect plots, exceed_rate, phase distribution, and visual behavior under identical Gazebo conditions."
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
        print("effort_plot_report_output:", args.output)
    if plot_paths:
        print("effort_plot_outputs:")
        for p in plot_paths:
            print("  ", p)
    if csv_output:
        print("effort_time_series_csv:", csv_output)
    if joint_csv_output and not args.no_joint_csv:
        print("effort_joint_series_csv:", joint_csv_output)


if __name__ == "__main__":
    main()
