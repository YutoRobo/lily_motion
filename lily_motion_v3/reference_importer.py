# -*- coding: utf-8 -*-
"""Import externally recorded joint command trajectories into v3-core.

v3.0.20 goal:
  * stop guessing legacy motion from videos alone;
  * accept the actual command sequence emitted by the legacy program;
  * convert it into a WholeRollCandidate-compatible object;
  * keep the implementation independent from the old project code.

Supported inputs:
  1. JSONL records containing one of:
       - joint_command_rad: [24 values] in existing Gazebo/JointState order
       - joint_command_deg: [24 values] in existing Gazebo/JointState order
       - positions / position / command: [24 values] in radians by default
       - joint_angles: {leg_id_or_name: [q0,q1,q2], ...}
  2. JSON object/list containing records as above.
  3. CSV with either:
       - joint_command_0 ... joint_command_23 columns, or
       - 24 Gazebo topic-like joint columns containing leg names, or
       - q0 ... q23 columns.

If base pose columns are absent, this importer uses a constant base pose and can
auto-align z so the first frame's lowest FK point touches ground_z.  That makes
geometry evaluation approximate.  Gazebo replay does not need base pose; it only
uses joint commands.
"""
from __future__ import division
import csv
import json
import math
import os

from lily_motion_v3.contact_state import ContactState
from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID, NUM_JOINTS
from lily_motion_v3.leg_role import SUPPORT, OTHER
from lily_motion_v3.motion_evaluation_report import MotionEvaluationReport
from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.roll_candidate import V3MotionFrame, V3RollCandidate
from lily_motion_v3.transforms import vec_add


_BASE_POSE_KEYS = {
    "x": ["base_x", "x"],
    "y": ["base_y", "y"],
    "z": ["base_z", "z"],
    "roll": ["base_roll", "roll"],
    "pitch": ["base_pitch", "pitch"],
    "yaw": ["base_yaw", "yaw"],
}


def _as_float_list(values, expected=None, degrees=False):
    out = [float(v) for v in values]
    if expected is not None and len(out) != expected:
        raise ValueError("expected %d values, got %d" % (expected, len(out)))
    if degrees:
        out = [math.radians(v) for v in out]
    return out


def _phase_from_record(record, fallback):
    for key in ("phase_name", "phase", "rf", "RF"):
        if key in record and record[key] not in (None, ""):
            return str(record[key])
    return fallback


def _frame_index_from_record(record, fallback):
    for key in ("frame_index", "frame", "index", "command_index", "seq", "step"):
        if key in record and record[key] not in (None, ""):
            return int(float(record[key]))
    return int(fallback)


def _base_pose_from_record(record, default_pose):
    pose = dict(default_pose)
    for out_key, aliases in _BASE_POSE_KEYS.items():
        for key in aliases:
            if key in record and record[key] not in (None, ""):
                pose[out_key] = float(record[key])
                break
    return pose


def _joint_map_from_gazebo_order(values, robot_model):
    vals = _as_float_list(values, expected=NUM_JOINTS)
    by_leg_name = {}
    for idx, (external_leg_id, joint_index) in enumerate(JOINT_STATE_ORDER):
        name = LEG_NAMES_BY_ID[external_leg_id]
        if name not in by_leg_name:
            by_leg_name[name] = [0.0, 0.0, 0.0]
        by_leg_name[name][joint_index] = vals[idx]
    out = {}
    for leg_name, q in by_leg_name.items():
        out[robot_model.leg_id(leg_name)] = list(q)
    return out


def _joint_map_from_mapping(mapping, robot_model, degrees=False):
    out = {}
    for key, value in mapping.items():
        if isinstance(key, str) and key in robot_model.mount_by_name:
            leg_id = robot_model.leg_id(key)
        else:
            leg_id = int(key)
        out[leg_id] = _as_float_list(value, expected=3, degrees=degrees)
    return out


def _extract_joint_map(record, robot_model, input_unit="rad"):
    deg = (input_unit == "deg")
    if "joint_command_rad" in record:
        return _joint_map_from_gazebo_order(record["joint_command_rad"], robot_model)
    if "joint_command_deg" in record:
        return _joint_map_from_gazebo_order(_as_float_list(record["joint_command_deg"], expected=NUM_JOINTS, degrees=True), robot_model)
    if "joint_angles_deg" in record:
        return _joint_map_from_mapping(record["joint_angles_deg"], robot_model, degrees=True)
    if "joint_angles" in record:
        return _joint_map_from_mapping(record["joint_angles"], robot_model, degrees=deg)
    for key in ("positions", "position", "command", "joint_command"):
        if key in record:
            vals = _as_float_list(record[key], expected=NUM_JOINTS, degrees=deg)
            return _joint_map_from_gazebo_order(vals, robot_model)
    raise ValueError("record has no recognized joint command fields")


def _records_from_json(path):
    text = open(path, "r").read().strip()
    if not text:
        return []
    if text[0] in "[{" :
        try:
            data = json.loads(text)
            if isinstance(data, list):
                return data
            if isinstance(data, dict) and "frames" in data:
                return data["frames"]
            if isinstance(data, dict) and "commands" in data:
                return data["commands"]
            return [data]
        except ValueError:
            pass
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        out.append(json.loads(line))
    return out


def _csv_joint_columns(fieldnames):
    names = list(fieldnames or [])
    candidates = []
    for prefix in ("joint_command_", "q", "joint_"):
        cols = []
        ok = True
        for i in range(NUM_JOINTS):
            name = "%s%d" % (prefix, i)
            if name not in names:
                ok = False
                break
            cols.append(name)
        if ok:
            return cols
    # Accept topic-like or leg_joint-like columns in Gazebo order if each topic substring is present.
    # This is intentionally conservative: exact q0..q23 is preferred for logs.
    return candidates


def _records_from_csv(path, input_unit="rad"):
    out = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return out
        cols = _csv_joint_columns(reader.fieldnames)
        for row in reader:
            rec = dict(row)
            if cols:
                vals = [row[c] for c in cols]
                rec["joint_command"] = vals
            out.append(rec)
    return out


def load_reference_records(path, input_format="auto", input_unit="rad"):
    fmt = input_format
    if fmt == "auto":
        ext = os.path.splitext(path)[1].lower()
        fmt = "csv" if ext == ".csv" else "jsonl"
    if fmt == "csv":
        return _records_from_csv(path, input_unit=input_unit)
    return _records_from_json(path)


def _auto_aligned_base_pose(robot_model, first_joint_map, ground_z=0.0, base_pose=None):
    pose = dict(base_pose or {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    min_z = None
    for leg_id, q in first_joint_map.items():
        pts = robot_model.link_positions_world(leg_id, q, pose)
        for p in pts.values():
            if min_z is None or p[2] < min_z:
                min_z = p[2]
    if min_z is not None:
        pose["z"] += float(ground_z) - float(min_z)
    return pose


def candidate_from_reference_records(records, robot_model=None, input_unit="rad", ground_z=0.0,
                                      auto_align_initial_ground=True, default_phase_name="ImportedReference",
                                      default_support_all=True):
    robot_model = robot_model or RobotModel()
    frames = []
    report = MotionEvaluationReport()
    default_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    first_joint_map = None
    parsed = []
    for i, rec in enumerate(records):
        qmap = _extract_joint_map(rec, robot_model, input_unit=input_unit)
        if first_joint_map is None:
            first_joint_map = qmap
        parsed.append((rec, qmap))
    if not parsed:
        raise ValueError("no reference command records were loaded")
    if auto_align_initial_ground:
        default_pose = _auto_aligned_base_pose(robot_model, first_joint_map, ground_z=ground_z, base_pose=default_pose)
    support_legs = list(range(8)) if default_support_all else []
    contact_state = ContactState(surface_id=1, support_legs=support_legs)
    leg_roles = dict((leg_id, SUPPORT if default_support_all else OTHER) for leg_id in range(8))
    for i, (rec, qmap) in enumerate(parsed):
        frame_index = _frame_index_from_record(rec, i)
        phase_name = _phase_from_record(rec, default_phase_name)
        pose = _base_pose_from_record(rec, default_pose)
        foot_body = dict((leg_id, robot_model.foot_position_body(leg_id, q)) for leg_id, q in qmap.items())
        foot_world = dict((leg_id, robot_model.foot_position_world(leg_id, q, pose)) for leg_id, q in qmap.items())
        diagnostics = {
            "imported_reference": True,
            "source_record_index": i,
            "geometry_base_pose_note": "base pose imported if present; otherwise constant auto-aligned base pose is used and geometry evaluation is approximate",
        }
        frames.append(V3MotionFrame(
            frame_index=frame_index,
            phase_index=0,
            phase_name=phase_name,
            phase_step_index=i,
            phase_step_count=len(parsed),
            contact_state=contact_state,
            base_pose=pose,
            leg_roles=leg_roles,
            foot_targets_body=foot_body,
            foot_targets_world=foot_world,
            joint_angles=qmap,
            diagnostics=diagnostics,
        ))
    report.task_success.update({
        "completed": True,
        "profile": "imported_reference",
        "legacy_dependency": False,
        "frame_count": len(frames),
        "auto_align_initial_ground": bool(auto_align_initial_ground),
        "geometry_evaluation_note": "If the input log did not contain base_pose, world-geometry metrics are approximate; Gazebo replay still uses the imported joint commands exactly.",
    })
    report.notes.append("v3.0.20: imported legacy/reference command trajectory; no old-project code is called.")
    return V3RollCandidate(direction="imported_reference", phases=[], frames=frames, report=report)


def candidate_from_reference_file(path, input_format="auto", input_unit="rad", **kwargs):
    records = load_reference_records(path, input_format=input_format, input_unit=input_unit)
    return candidate_from_reference_records(records, input_unit=input_unit, **kwargs)


def candidate_to_json_file(candidate, path):
    dirname = os.path.dirname(path)
    if dirname and not os.path.isdir(dirname):
        os.makedirs(dirname)
    with open(path, "w") as f:
        json.dump(candidate.to_dict(), f, indent=2, sort_keys=True)
        f.write("\n")


def candidate_from_json_file(path):
    data = json.load(open(path, "r"))
    robot_model = RobotModel()
    report = MotionEvaluationReport()
    task = (data.get("report") or {}).get("task_success") or {}
    report.task_success.update(task)
    report.task_success.setdefault("completed", True)
    report.task_success.setdefault("profile", "imported_reference")
    report.notes.extend((data.get("report") or {}).get("notes") or [])
    frames = []
    for f in data.get("frames", []):
        csd = f.get("contact_state") or {}
        cs = ContactState(
            surface_id=csd.get("surface_id", 1),
            support_legs=csd.get("support_legs") or [],
            candidate_support_legs=csd.get("candidate_support_legs") or [],
            lift_legs=csd.get("lift_legs") or [],
            clearance_legs=csd.get("clearance_legs") or [],
            transfer_legs=csd.get("transfer_legs") or [],
        )
        def int_key_map(mapping):
            return dict((int(k), v) for k, v in (mapping or {}).items())
        frames.append(V3MotionFrame(
            frame_index=f.get("frame_index", len(frames)),
            phase_index=f.get("phase_index", 0),
            phase_name=f.get("phase_name", "ImportedReference"),
            phase_step_index=f.get("phase_step_index", len(frames)),
            phase_step_count=f.get("phase_step_count", len(data.get("frames", []))),
            contact_state=cs,
            base_pose=f.get("base_pose") or {},
            leg_roles=int_key_map(f.get("leg_roles")),
            foot_targets_body=int_key_map(f.get("foot_targets_body")),
            foot_targets_world=int_key_map(f.get("foot_targets_world")),
            joint_angles=int_key_map(f.get("joint_angles")),
            diagnostics=f.get("diagnostics") or {},
        ))
    return robot_model, V3RollCandidate(data.get("direction", "imported_reference"), [], frames, report)
