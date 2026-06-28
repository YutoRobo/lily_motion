#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Replay a v3 project-contained candidate in Gazebo.

The first intended use is previewing the candidate up to the first invalid frame.
This is not yet a success-gait runner; it is a Gazebo visualization bridge for
v3 generated commands.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.ros_bridge import CombinedCommandPublisher, GazeboCommandPublisher, JetsonJointStatePublisher, MockPublisher, GazeboLinkStateLogger, split_csv
from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.gazebo_export import V3GazeboCommandExporter, frames_until_invalid, frame_is_invalid
from lily_motion_v3.command_resampler import resample_command_records, moving_average_command_records, full_command_diagnostics, unwrap_continuous_command_records


def _parse_float_list(text):
    out = []
    for part in str(text).split(','):
        part = part.strip()
        if part:
            out.append(float(part))
    return out


def build_candidate(args):
    cfg = V3RollGenerationConfig(
        steps_per_phase=args.steps_per_phase,
        lift_height=args.lift_height,
        clearance_height=args.clearance_height,
        candidate_support_shift_x=args.candidate_support_shift_x,
        candidate_support_drop_z=args.candidate_support_drop_z,
        body_roll_pitch_rad=math.radians(args.body_roll_pitch_deg),
        body_roll_x_shift=args.body_roll_x_shift,
        body_roll_z_shift=args.body_roll_z_shift,
        enable_body_roll_pose_search=not args.disable_body_roll_pose_search,
        body_roll_search_x_offsets=_parse_float_list(args.body_roll_search_x_offsets),
        body_roll_search_z_offsets=_parse_float_list(args.body_roll_search_z_offsets),
        ground_z=args.ground_z,
        auto_align_initial_ground=not args.no_auto_align_initial_ground,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        min_target_point_clearance_m=args.min_target_point_clearance,
        enable_contact_lock_generation=not args.no_contact_lock_generation,
        contact_plan_variant=args.contact_plan_variant,
    )
    gen = V3RollCandidateGenerator(config=cfg)
    return gen, gen.generate_forward_one_roll(surface_id=args.surface_id)


def _sleep_rate(rate, dry_sleep=False, sleep_sec=0.0):
    """Sleep helper that works with rospy.Rate and dry-run dummy rates."""
    if dry_sleep and sleep_sec > 0:
        try:
            import time
            time.sleep(sleep_sec)
            return
        except Exception:
            pass
    rate.sleep()




class _CommandLogFrame(object):
    def __init__(self, rec, fallback_index):
        self.rec = dict(rec)
        self.frame_index = int(rec.get("frame_index", rec.get("command_index", rec.get("published_index", fallback_index))))
        self.phase_name = str(rec.get("phase_name", "command_log"))
        self.phase_index = int(rec.get("phase_index", 0) or 0)
        self.phase_step_index = int(rec.get("phase_step_index", 0) or 0)
        self.phase_step_count = int(rec.get("phase_step_count", 1) or 1)
        self.base_pose = dict(rec.get("base_pose", {}) or {})
        self.leg_roles = dict(rec.get("leg_roles", {}) or {})
        self.diagnostics = dict(rec.get("diagnostics", {}) or {})
        self.joint_command_rad = list(rec["joint_command_rad"])


class _CommandLogExporter(object):
    def frame_to_joint_state_order(self, frame):
        return list(frame.joint_command_rad)


def load_command_log_records(path):
    records = []
    with open(path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            rec = json.loads(line)
            if "joint_command_rad" not in rec:
                raise ValueError("command log record %d has no joint_command_rad: %s" % (i, path))
            records.append(rec)
    return records


def command_records_to_frames(records):
    return [_CommandLogFrame(rec, i) for i, rec in enumerate(records)]

def load_command_log_frames(path):
    return command_records_to_frames(load_command_log_records(path))

def publish_frames(frames, exporter, publisher, rate, link_state_logger=None,
                   repeat_last=0, rate_hz=30.0, frame_hold_sec=0.0,
                   hold_start_sec=0.0, hold_end_sec=0.0, verbose=False,
                   dry_sleep=False):
    """Publish frames sequentially.

    A generated v3 preview can contain only a few dozen frames.  If each frame is
    published only once at 30 Hz, Gazebo playback finishes in about a second.
    frame_hold_sec intentionally republishes each frame for multiple controller
    ticks so visual preview is slow enough to inspect.
    """
    count = 0
    last_cmd = None
    rate_hz = float(rate_hz) if rate_hz and rate_hz > 0 else 30.0
    hold_ticks = max(1, int(round(max(0.0, float(frame_hold_sec)) * rate_hz)))
    start_ticks = max(0, int(round(max(0.0, float(hold_start_sec)) * rate_hz)))
    end_ticks = max(0, int(round(max(0.0, float(hold_end_sec)) * rate_hz)))

    if frames and start_ticks > 0:
        first_cmd = exporter.frame_to_joint_state_order(frames[0])
        if verbose:
            print("holding first frame for %d ticks" % start_ticks)
        for _ in range(start_ticks):
            publisher.publish(first_cmd)
            count += 1
            _sleep_rate(rate, dry_sleep=dry_sleep, sleep_sec=1.0 / rate_hz)

    for frame in frames:
        cmd = exporter.frame_to_joint_state_order(frame)
        invalid, reasons = frame_is_invalid(frame)
        if verbose:
            print("publish frame=%s phase=%s step=%s hold_ticks=%d invalid=%s reasons=%s" % (
                getattr(frame, "frame_index", None),
                getattr(frame, "phase_name", ""),
                getattr(frame, "phase_step_index", None),
                hold_ticks,
                invalid,
                ",".join(reasons)))
        for hold_index in range(hold_ticks):
            publisher.publish(cmd)
            if link_state_logger is not None and hold_index == 0:
                link_state_logger.snapshot({
                    "frame_index": frame.frame_index,
                    "phase_name": frame.phase_name,
                    "phase_index": frame.phase_index,
                    "phase_step_index": frame.phase_step_index,
                    "phase_step_count": frame.phase_step_count,
                    "base_pose": dict(frame.base_pose),
                    "leg_roles": dict((str(k), v) for k, v in frame.leg_roles.items()),
                    "invalid_reasons": reasons,
                    "published_index": count,
                    "frame_hold_ticks": hold_ticks,
                })
            last_cmd = cmd
            count += 1
            _sleep_rate(rate, dry_sleep=dry_sleep, sleep_sec=1.0 / rate_hz)

    # Backward-compatible tick-based hold.
    total_last_ticks = max(0, int(repeat_last)) + end_ticks
    for _ in range(total_last_ticks):
        if last_cmd is None:
            break
        publisher.publish(last_cmd)
        count += 1
        _sleep_rate(rate, dry_sleep=dry_sleep, sleep_sec=1.0 / rate_hz)
    return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--surface-id", type=int, default=1)
    ap.add_argument("--contact-plan-variant", default="default")
    ap.add_argument("--steps-per-phase", type=int, default=8)
    ap.add_argument("--lift-height", type=float, default=0.08)
    ap.add_argument("--clearance-height", type=float, default=0.06)
    ap.add_argument("--candidate-support-shift-x", type=float, default=0.04)
    ap.add_argument("--candidate-support-drop-z", type=float, default=-0.02)
    ap.add_argument("--body-roll-pitch-deg", type=float, default=90.0)
    ap.add_argument("--body-roll-x-shift", type=float, default=0.0)
    ap.add_argument("--body-roll-z-shift", type=float, default=0.0)
    ap.add_argument("--disable-body-roll-pose-search", action="store_true")
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--no-auto-align-initial-ground", action="store_true")
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--min-target-point-clearance", type=float, default=0.04)
    ap.add_argument("--no-contact-lock-generation", action="store_true")
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--frame-hold-sec", type=float, default=0.10,
                    help="hold each generated frame for this many seconds by republishing it")
    ap.add_argument("--hold-start-sec", type=float, default=1.0,
                    help="publish the first command for this many seconds before replay")
    ap.add_argument("--hold-end-sec", type=float, default=2.0,
                    help="publish the final command for this many seconds after replay")
    ap.add_argument("--verbose-publish", action="store_true")
    ap.add_argument("--dry-run-sleep", action="store_true",
                    help="make --dry-run actually sleep so timing can be felt")
    ap.add_argument("--node-name", default="lily_motion_v3_0_gazebo_replay")
    ap.add_argument("--dry-run", action="store_true", help="do not require ROS; validate/export only")
    ap.add_argument("--jetson", action="store_true")
    ap.add_argument("--allow-invalid-frames", action="store_true",
                    help="publish all frames, including known invalid frames")
    ap.add_argument("--include-invalid-frame", action="store_true",
                    help="when stopping on invalid, include the first invalid frame")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--repeat-last", type=int, default=20)
    ap.add_argument("--command-log", default="", help="If this existing JSONL contains joint_command_rad, replay it as input. If it does not exist, generated preview commands are written there unless --strict-command-log-input is set.")
    ap.add_argument("--strict-command-log-input", action="store_true", help="Treat --command-log as an input file and fail if it does not exist. Prevents accidentally generating/replaying a native candidate when a processed command log was not created.")
    ap.add_argument("--resample-factor", type=int, default=1, help="When replaying an existing command log, linearly interpolate commands by this factor for smoother preview")
    ap.add_argument("--smooth-window", type=int, default=1, help="When replaying an existing command log, apply centered moving average to joint commands; use odd values such as 3 or 5")
    ap.add_argument("--segment-key", default="", help="Optional record key such as roll_index. If set, replay-time resampling/smoothing are reset at segment boundaries.")
    ap.add_argument("--unwrap-continuous-angles", action="store_true", help="Before replay-time resampling/smoothing, choose 2*pi-equivalent joint angles closest to previous frame. This preserves posture but removes representation jumps.")
    ap.add_argument("--diagnose-command-log", action="store_true", help="print command range/adjacent-delta diagnostics for the replay command sequence")
    ap.add_argument("--candidate-output", default="")
    ap.add_argument("--gazebo-link-state-log", default=None)
    ap.add_argument("--gazebo-foot-link-contains", default="")
    ap.add_argument("--gazebo-foot-link-regex", default="base_clause_link$|thigh_link$|tibia_link$|appendix_link$")
    ap.add_argument("--gazebo-log-all-links", action="store_true")
    args = ap.parse_args()

    if args.strict_command_log_input and args.command_log and not os.path.exists(args.command_log):
        raise IOError("--strict-command-log-input: command log does not exist: %s" % args.command_log)
    replaying_existing_command_log = bool(args.command_log and os.path.exists(args.command_log))
    first_invalid = None
    cand = None
    gen = None
    command_log_diagnostics = None
    if replaying_existing_command_log:
        records = load_command_log_records(args.command_log)
        if args.unwrap_continuous_angles:
            records = unwrap_continuous_command_records(records)
        if args.resample_factor and args.resample_factor > 1:
            records = resample_command_records(records, factor=args.resample_factor, segment_key=(args.segment_key.strip() or None))
        if args.smooth_window and args.smooth_window > 1:
            records = moving_average_command_records(records, window=args.smooth_window, segment_key=(args.segment_key.strip() or None))
        if args.diagnose_command_log:
            command_log_diagnostics = full_command_diagnostics(records)
        frames = command_records_to_frames(records)
        exporter = _CommandLogExporter()
        if args.max_frames and args.max_frames > 0:
            frames = frames[:args.max_frames]
    else:
        gen, cand = build_candidate(args)
        if args.candidate_output:
            with open(args.candidate_output, "w") as f:
                f.write(json.dumps(cand.to_dict(), indent=2, sort_keys=True))
                f.write("\n")

        frames = list(cand.frames)
        if not args.allow_invalid_frames:
            frames, first_invalid = frames_until_invalid(frames, include_invalid=args.include_invalid_frame)
        if args.max_frames and args.max_frames > 0:
            frames = frames[:args.max_frames]

        exporter = V3GazeboCommandExporter(gen.robot_model)

        if args.command_log:
            dirname = os.path.dirname(args.command_log)
            if dirname and not os.path.isdir(dirname):
                os.makedirs(dirname)
            with open(args.command_log, "w") as f:
                for i, frame in enumerate(frames):
                    rec = {
                        "published_index": i,
                        "frame_index": frame.frame_index,
                        "phase_name": frame.phase_name,
                        "phase_step_index": frame.phase_step_index,
                        "base_pose": dict(frame.base_pose),
                        "joint_command_rad": exporter.frame_to_joint_state_order(frame),
                    }
                    f.write(json.dumps(rec, sort_keys=True))
                    f.write("\n")

    link_state_logger = None
    published_count = 0
    if args.dry_run:
        publisher = MockPublisher()
        class DummyRate(object):
            def sleep(self):
                pass
        rate = DummyRate()
        published_count = publish_frames(
            frames, exporter, publisher, rate, None,
            repeat_last=0,
            rate_hz=args.rate,
            frame_hold_sec=args.frame_hold_sec,
            hold_start_sec=args.hold_start_sec,
            hold_end_sec=args.hold_end_sec,
            verbose=args.verbose_publish,
            dry_sleep=args.dry_run_sleep)
    else:
        try:
            import rospy
        except Exception as e:
            raise RuntimeError("rospy is required unless --dry-run is used: %s" % e)
        rospy.init_node(args.node_name, anonymous=True)
        gazebo_pub = GazeboCommandPublisher()
        jetson_pub = JetsonJointStatePublisher() if args.jetson else None
        publisher = CombinedCommandPublisher(gazebo_publisher=gazebo_pub, jetson_publisher=jetson_pub)
        rate = rospy.Rate(args.rate)
        if args.gazebo_link_state_log:
            link_state_logger = GazeboLinkStateLogger(
                args.gazebo_link_state_log,
                name_contains=split_csv(args.gazebo_foot_link_contains),
                name_regex=args.gazebo_foot_link_regex,
                log_all_links=args.gazebo_log_all_links)
            link_state_logger.start()
        published_count = publish_frames(
            frames, exporter, publisher, rate, link_state_logger,
            repeat_last=args.repeat_last,
            rate_hz=args.rate,
            frame_hold_sec=args.frame_hold_sec,
            hold_start_sec=args.hold_start_sec,
            hold_end_sec=args.hold_end_sec,
            verbose=args.verbose_publish,
            dry_sleep=False)
        if link_state_logger is not None:
            print("gazebo_link_state_records:", link_state_logger.record_count)
            print("gazebo_link_state_matched_names:", sorted(list(link_state_logger.matched_names)))
            link_state_logger.close()

    summary = {
        "candidate_completed": cand.report.task_success.get("completed") if cand is not None else None,
        "candidate_frame_count": len(cand.frames) if cand is not None else None,
        "replaying_existing_command_log": replaying_existing_command_log,
        "preview_frame_count": len(frames),
        "published_count": published_count,
        "rate_hz": args.rate,
        "frame_hold_sec": args.frame_hold_sec,
        "hold_start_sec": args.hold_start_sec,
        "hold_end_sec": args.hold_end_sec,
        "estimated_playback_sec": (float(published_count) / float(args.rate)) if args.rate else None,
        "first_invalid_frame": first_invalid,
        "dry_run": args.dry_run,
        "command_log": args.command_log,
        "resample_factor": args.resample_factor,
        "smooth_window": args.smooth_window,
        "segment_key": args.segment_key,
        "unwrap_continuous_angles": args.unwrap_continuous_angles,
        "command_log_diagnostics": command_log_diagnostics,
        "candidate_output": args.candidate_output,
        "report_task_success": cand.report.task_success if cand is not None else {},
    }
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
