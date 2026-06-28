# -*- coding: utf-8 -*-
"""Joint-command filtering utilities for v3 whole-roll evaluation.

The standard filter operates on angular trajectories while preserving angle
continuity.  v3.0.10 adds an optional contact-preserving projection stage:
after moving-average smoothing, SUPPORT-leg joint angles are re-solved so that
locked foot contact points remain fixed in world coordinates.  This matches the
rolling-gait assumption that raw singularity/flip-like command jumps may be
smoothed, but smoothing must not make a planted foot slide.
"""
from __future__ import division
import math

from lily_motion_v3.transforms import angle_delta
from lily_motion_v3 import leg_role as R


def unwrap_angle_sequence(values):
    if not values:
        return []
    out = [float(values[0])]
    for v in values[1:]:
        out.append(out[-1] + angle_delta(float(v), out[-1]))
    return out


def moving_average_sequence(values, window):
    window = max(1, int(window))
    if not values:
        return []
    if window <= 1:
        return [float(v) for v in values]
    half = window // 2
    out = []
    n = len(values)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        s = 0.0
        for j in range(lo, hi):
            s += float(values[j])
        out.append(s / float(hi - lo))
    return out


def filter_joint_trajectory(frames, window=5):
    """Return moving-average filtered joint maps for every frame.

    Args:
        frames: list of V3MotionFrame-like objects containing joint_angles.
        window: moving-average window in frames.  Use 1 to bypass.

    Returns:
        list[dict[int, list[float]]]: filtered joint map per frame.
    """
    frames = list(frames or [])
    if not frames:
        return []
    leg_ids = sorted([int(k) for k in frames[0].joint_angles.keys()])
    filtered = [dict() for _ in frames]
    for leg_id in leg_ids:
        for jidx in range(3):
            raw = [float(f.joint_angles[leg_id][jidx]) for f in frames]
            unwrapped = unwrap_angle_sequence(raw)
            smoothed = moving_average_sequence(unwrapped, window)
            for i, val in enumerate(smoothed):
                if leg_id not in filtered[i]:
                    filtered[i][leg_id] = [0.0, 0.0, 0.0]
                filtered[i][leg_id][jidx] = float(val)
    return filtered


def filter_joint_trajectory_contact_reproject(frames, robot_model, window=5):
    """Moving-average filter followed by SUPPORT contact-lock reprojection.

    The moving average reduces raw singularity/flip-like command jumps.  The
    reprojection then fixes a major side effect of naive filtering: SUPPORT feet
    drift because averaged joint angles no longer satisfy the original contact
    constraints.

    A lock is created when a leg enters SUPPORT.  The lock point is taken from
    frame.foot_targets_world when available, otherwise from raw FK.  While the
    leg remains SUPPORT, the filtered joint angle is replaced by an IK solution
    that reaches the locked world point under the frame's base_pose.  The
    previous reprojected joint is used as the IK branch-selection reference.

    Returns:
        (joint_maps, diagnostics)
    """
    frames = list(frames or [])
    filtered = filter_joint_trajectory(frames, window)
    projected = []
    locks = {}
    created = []
    released = []
    failures = []
    projected_count = 0
    support_sample_count = 0

    previous_q_by_leg = None
    for idx, frame in enumerate(frames):
        qmap = dict((int(k), list(v)) for k, v in filtered[idx].items())
        if previous_q_by_leg is None:
            previous_q_by_leg = dict((int(k), list(v)) for k, v in qmap.items())
        active_support = set(int(k) for k, role in frame.leg_roles.items() if role == R.SUPPORT)
        for leg_id in list(locks.keys()):
            if leg_id not in active_support:
                released.append({
                    "frame_index": frame.frame_index,
                    "phase_name": frame.phase_name,
                    "phase_step_index": frame.phase_step_index,
                    "leg_id": leg_id,
                    "leg_name": robot_model.leg_name(leg_id),
                    "lock_point_world": list(locks[leg_id]["lock_point_world"]),
                })
                del locks[leg_id]
        for leg_id in sorted(active_support):
            support_sample_count += 1
            if leg_id not in locks:
                if leg_id in frame.foot_targets_world:
                    lock_point = list(frame.foot_targets_world[leg_id])
                else:
                    lock_point = robot_model.foot_position_world(leg_id, frame.joint_angles[leg_id], frame.base_pose)
                locks[leg_id] = {
                    "lock_point_world": list(lock_point),
                    "start_frame_index": frame.frame_index,
                    "start_phase_name": frame.phase_name,
                    "start_phase_step_index": frame.phase_step_index,
                }
                created.append({
                    "frame_index": frame.frame_index,
                    "phase_name": frame.phase_name,
                    "phase_step_index": frame.phase_step_index,
                    "leg_id": leg_id,
                    "leg_name": robot_model.leg_name(leg_id),
                    "lock_point_world": list(lock_point),
                })
            target_body = robot_model.world_point_to_body(locks[leg_id]["lock_point_world"], frame.base_pose)
            prev_q = previous_q_by_leg.get(leg_id, qmap.get(leg_id))
            selected = robot_model.select_ik_body(leg_id, target_body, previous_q=prev_q)
            if selected is None:
                failures.append({
                    "frame_index": frame.frame_index,
                    "phase_name": frame.phase_name,
                    "phase_step_index": frame.phase_step_index,
                    "leg_id": leg_id,
                    "leg_name": robot_model.leg_name(leg_id),
                    "target_body": list(target_body),
                    "lock_point_world": list(locks[leg_id]["lock_point_world"]),
                    "base_pose": dict(frame.base_pose),
                })
                continue
            qmap[leg_id] = list(selected.q)
            projected_count += 1
        projected.append(qmap)
        previous_q_by_leg = dict((int(k), list(v)) for k, v in qmap.items())

    return projected, {
        "enabled": True,
        "method": "moving_average_then_support_contact_reproject",
        "window": int(window),
        "support_sample_count": support_sample_count,
        "projected_count": projected_count,
        "projection_failure_count": len(failures),
        "top_projection_failures": failures[:20],
        "created_lock_count": len(created),
        "released_lock_count": len(released),
        "top_created_locks": created[:20],
        "top_released_locks": released[:20],
    }


def max_joint_step_deg(joint_maps):
    max_delta = 0.0
    max_record = None
    for i in range(1, len(joint_maps)):
        prev = joint_maps[i - 1]
        cur = joint_maps[i]
        for leg_id in sorted(cur.keys()):
            if leg_id not in prev:
                continue
            deltas = []
            for jidx in range(3):
                d = abs(math.degrees(angle_delta(cur[leg_id][jidx], prev[leg_id][jidx])))
                deltas.append(d)
            local = max(deltas)
            if local > max_delta:
                max_delta = local
                max_record = {"frame_index": i, "leg_id": int(leg_id), "delta_deg": deltas, "max_delta_deg": local}
    return max_delta, max_record
