# -*- coding: utf-8 -*-
"""Failure-diagnosis utilities for v3 whole-roll candidates.

The purpose is not to change the trajectory.  It summarizes *where* and *why*
a generated candidate is failing so native, legacy-style, and future optimized
profiles can be compared through the same report schema.
"""
from __future__ import division
import math
from collections import Counter


def _key_phase(rec):
    return str(rec.get("phase_name", ""))


def _key_leg(rec):
    name = rec.get("leg_name")
    if name:
        return str(name)
    if "leg_id" in rec:
        return str(rec.get("leg_id"))
    return "unknown"


def _key_role(rec):
    return str(rec.get("role", "unknown"))


def _counter_dict(counter):
    return dict((str(k), int(v)) for k, v in sorted(counter.items(), key=lambda kv: (-kv[1], str(kv[0]))))


def _first_by_frame(records):
    if not records:
        return None
    return sorted(records, key=lambda r: (int(r.get("frame_index", 10 ** 9)), str(r.get("phase_name", "")), int(r.get("phase_step_index", 10 ** 9))))[0]


def _histogram(records):
    by_phase = Counter()
    by_leg = Counter()
    by_role = Counter()
    for rec in records:
        by_phase[_key_phase(rec)] += 1
        by_leg[_key_leg(rec)] += 1
        by_role[_key_role(rec)] += 1
    return {
        "by_phase": _counter_dict(by_phase),
        "by_leg": _counter_dict(by_leg),
        "by_role": _counter_dict(by_role),
    }


def generator_ik_failure_records(candidate):
    records = []
    for f in candidate.frames:
        for rec in (f.diagnostics or {}).get("ik_failures", []):
            records.append(dict(rec))
    return records


def summarize_failure_diagnosis(candidate, whole_roll_evaluation=None):
    """Return a compact diagnosis dictionary for a candidate/evaluation pair."""
    whole = whole_roll_evaluation or {}
    raw = whole.get("raw_command", {})
    filt = whole.get("filtered_command", {})
    raw_geom = raw.get("geometry", {})
    filt_geom = filt.get("geometry", {})
    raw_contact = whole.get("raw_contact_lock", {})
    filt_contact = whole.get("filtered_contact_lock", {})

    ik_records = generator_ik_failure_records(candidate)
    raw_pen = (raw_geom.get("ground_clearance") or {}).get("top_penetration_records", [])
    filt_pen = (filt_geom.get("ground_clearance") or {}).get("top_penetration_records", [])
    raw_near = (raw_geom.get("inter_leg_clearance") or {}).get("top_near_records", [])
    filt_near = (filt_geom.get("inter_leg_clearance") or {}).get("top_near_records", [])
    raw_second = (raw_geom.get("joint_limit") or {}).get("top_second_joint_violations", [])
    filt_second = (filt_geom.get("joint_limit") or {}).get("top_second_joint_violations", [])
    raw_drift = raw_contact.get("top_contact_drift_violations", [])
    filt_drift = filt_contact.get("top_contact_drift_violations", [])
    filt_drift_hard = filt_contact.get("top_contact_drift_hard_violations", [])

    categories = {
        "generator_ik_failure": {
            "count": len(ik_records),
            "first": _first_by_frame(ik_records),
            "histogram": _histogram(ik_records),
            "top_records": ik_records[:20],
        },
        "filtered_penetration": {
            "count": (filt_geom.get("ground_clearance") or {}).get("penetration_count", len(filt_pen)),
            "first": _first_by_frame(filt_pen),
            "histogram": _histogram(filt_pen),
            "top_records": filt_pen[:20],
        },
        "filtered_inter_leg_near": {
            "count": (filt_geom.get("inter_leg_clearance") or {}).get("near_count", len(filt_near)),
            "first": _first_by_frame(filt_near),
            "histogram": _histogram(filt_near),
            "top_records": filt_near[:20],
        },
        "filtered_second_joint_violation": {
            "count": (filt_geom.get("joint_limit") or {}).get("second_joint_violation_count", len(filt_second)),
            "first": _first_by_frame(filt_second),
            "histogram": _histogram(filt_second),
            "top_records": filt_second[:20],
        },
        "filtered_contact_drift_soft": {
            "count": filt_contact.get("contact_drift_soft_violation_count", len(filt_drift)),
            "first": _first_by_frame(filt_drift),
            "histogram": _histogram(filt_drift),
            "top_records": filt_drift[:20],
        },
        "filtered_contact_drift_hard": {
            "count": filt_contact.get("contact_drift_hard_violation_count", len(filt_drift_hard)),
            "first": _first_by_frame(filt_drift_hard),
            "histogram": _histogram(filt_drift_hard),
            "top_records": filt_drift_hard[:20],
        },
    }

    # Choose a human-readable dominant failure category.  This is a diagnosis aid,
    # not a mathematical proof of root cause.
    priority = [
        "generator_ik_failure",
        "filtered_penetration",
        "filtered_inter_leg_near",
        "filtered_second_joint_violation",
        "filtered_contact_drift_hard",
        "filtered_contact_drift_soft",
    ]
    dominant = None
    for name in priority:
        if categories[name]["count"]:
            dominant = name
            break

    return {
        "schema_version": "v3.0.15.failure_diagnosis",
        "candidate_completed": bool(candidate.report.task_success.get("completed", False)) if candidate.report else False,
        "whole_roll_success_by_filtered_geometry": bool(whole.get("whole_roll_success_by_filtered_geometry", False)),
        "frame_count": len(candidate.frames),
        "dominant_failure_category": dominant,
        "categories": categories,
        "raw_reference": {
            "penetration_count": (raw_geom.get("ground_clearance") or {}).get("penetration_count", len(raw_pen)),
            "near_count": (raw_geom.get("inter_leg_clearance") or {}).get("near_count", len(raw_near)),
            "second_joint_violation_count": (raw_geom.get("joint_limit") or {}).get("second_joint_violation_count", len(raw_second)),
            "contact_drift_soft_violation_count": raw_contact.get("contact_drift_soft_violation_count", len(raw_drift)),
        },
    }
