# -*- coding: utf-8 -*-
"""Lightweight 3D visualization utilities for v3 roll candidates.

This module is intentionally independent from ROS/Gazebo.  It renders the
self-contained v3 RobotModel, base pose, leg segments, foot points, and contact
lock points so the whole-body motion can be inspected before Gazebo replay.
"""
from __future__ import division
import os
import json

from lily_motion_v3.command_filter import filter_joint_trajectory
from lily_motion_v3 import leg_role as R


_BODY_EDGE_PAIRS = [
    (0, 1), (1, 3), (3, 2), (2, 0),
    (4, 5), (5, 7), (7, 6), (6, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def _ensure_matplotlib():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        return plt
    except Exception as exc:
        raise RuntimeError('matplotlib 3D rendering is required for visualization: %s' % exc)


def _mkdir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _frame_joint_maps(frames, command_source, filter_window):
    if command_source == 'filtered':
        return filter_joint_trajectory(frames, filter_window)
    return [dict((int(k), list(v)) for k, v in f.joint_angles.items()) for f in frames]


def _body_corners_from_mounts(robot_model):
    xs = [m.position[0] for m in robot_model.mounts]
    ys = [m.position[1] for m in robot_model.mounts]
    zs = [m.position[2] for m in robot_model.mounts]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    return [
        [xmax, ymin, zmax], [xmin, ymin, zmax], [xmax, ymin, zmin], [xmin, ymin, zmin],
        [xmax, ymax, zmax], [xmin, ymax, zmax], [xmax, ymax, zmin], [xmin, ymax, zmin],
    ]


def _set_axes_equal(ax, points, margin=0.15):
    if not points:
        return
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    cz = 0.5 * (min(zs) + max(zs))
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.5) * (1.0 + margin)
    half = 0.5 * span
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_zlim(cz - half, cz + half)


def select_key_frame_indices(candidate, whole_eval=None, max_frames=18):
    """Return representative frame indices for visual inspection.

    The selection contains start/end, phase starts/ends, and the first dominant
    failure frame when available.  It is deliberately compact so a single HTML
    page remains readable.
    """
    frames = list(candidate.frames or [])
    if not frames:
        return []
    n = len(frames)
    indices = set([0, n - 1])
    prev_phase = None
    for i, f in enumerate(frames):
        if f.phase_name != prev_phase:
            indices.add(i)
            if i > 0:
                indices.add(i - 1)
            prev_phase = f.phase_name
    diag = (whole_eval or {}).get('failure_diagnosis', {})
    dom = diag.get('dominant_failure_category')
    if dom:
        cat = (diag.get('categories') or {}).get(dom, {})
        first = cat.get('first')
        if first and first.get('frame_index') is not None:
            fi = int(first.get('frame_index'))
            for j in [fi - 1, fi, fi + 1]:
                if 0 <= j < n:
                    indices.add(j)
    ordered = sorted(indices)
    max_frames = max(1, int(max_frames))
    if len(ordered) <= max_frames:
        return ordered
    # Downsample while preserving first and last.
    keep = []
    for k in range(max_frames):
        pos = int(round(k * (len(ordered) - 1) / float(max_frames - 1))) if max_frames > 1 else 0
        keep.append(ordered[pos])
    return sorted(set(keep))


def contact_lock_points_up_to_frame(candidate, frame_index, robot_model):
    """Reconstruct active SUPPORT contact lock points up to frame_index.

    This mirrors the intended generator/evaluator semantics: a lock is created
    when a leg becomes SUPPORT and released when it leaves SUPPORT.
    """
    locks = {}
    frames = list(candidate.frames or [])
    upto = min(int(frame_index), len(frames) - 1)
    for i in range(upto + 1):
        f = frames[i]
        active = set(int(k) for k, role in f.leg_roles.items() if role == R.SUPPORT)
        for leg_id in list(locks.keys()):
            if leg_id not in active:
                del locks[leg_id]
        for leg_id in sorted(active):
            if leg_id not in locks:
                if leg_id in f.foot_targets_world:
                    locks[leg_id] = list(f.foot_targets_world[leg_id])
                elif leg_id in f.joint_angles:
                    locks[leg_id] = robot_model.foot_position_world(leg_id, f.joint_angles[leg_id], f.base_pose)
    return locks


def draw_frame(robot_model, candidate, frame_index, qmap, output_path,
               command_source='filtered', ground_z=0.0, title_extra=''):
    plt = _ensure_matplotlib()
    frames = list(candidate.frames or [])
    frame = frames[int(frame_index)]
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    all_points = []
    # Ground reference grid.
    ax.plot([-1.0, 1.0], [0.0, 0.0], [ground_z, ground_z], linewidth=0.8)
    ax.plot([0.0, 0.0], [-0.6, 0.6], [ground_z, ground_z], linewidth=0.8)

    # Body wireframe.
    corners_body = _body_corners_from_mounts(robot_model)
    corners_world = [robot_model.body_point_to_world(p, frame.base_pose) for p in corners_body]
    all_points.extend(corners_world)
    for a, b in _BODY_EDGE_PAIRS:
        pa = corners_world[a]
        pb = corners_world[b]
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]], linewidth=1.4)

    # Leg segments and foot markers.
    for leg_id in sorted(qmap.keys()):
        segments = robot_model.leg_segments_world(leg_id, qmap[leg_id], frame.base_pose)
        for seg in segments:
            a = seg['a']
            b = seg['b']
            all_points.append(a)
            all_points.append(b)
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], linewidth=1.1)
        foot = robot_model.foot_position_world(leg_id, qmap[leg_id], frame.base_pose)
        all_points.append(foot)
        ax.scatter([foot[0]], [foot[1]], [foot[2]], s=18)
        ax.text(foot[0], foot[1], foot[2], robot_model.leg_name(leg_id), fontsize=7)

    # Active contact locks.
    locks = contact_lock_points_up_to_frame(candidate, frame_index, robot_model)
    for leg_id, p in locks.items():
        all_points.append(p)
        ax.scatter([p[0]], [p[1]], [p[2]], marker='x', s=45)

    pitch_deg = frame.base_pose.get('pitch', 0.0) * 180.0 / 3.141592653589793
    title = '%s frame=%d phase=%s step=%d pitch=%.1f deg %s' % (
        command_source, frame.frame_index, frame.phase_name, frame.phase_step_index, pitch_deg, title_extra)
    ax.set_title(title)
    ax.set_xlabel('world x [m]')
    ax.set_ylabel('world y [m]')
    ax.set_zlabel('world z [m]')
    _set_axes_equal(ax, all_points)
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def visualize_candidate(robot_model, candidate, whole_eval, output_dir,
                        command_source='filtered', filter_window=5,
                        frame_indices=None, max_frames=18, ground_z=0.0):
    _mkdir(output_dir)
    frames = list(candidate.frames or [])
    qmaps = _frame_joint_maps(frames, command_source, filter_window)
    if frame_indices is None:
        frame_indices = select_key_frame_indices(candidate, whole_eval, max_frames=max_frames)
    rendered = []
    for idx in frame_indices:
        if idx < 0 or idx >= len(frames):
            continue
        filename = 'frame_%04d_%s.png' % (idx, command_source)
        path = os.path.join(output_dir, filename)
        draw_frame(robot_model, candidate, idx, qmaps[idx], path,
                   command_source=command_source, ground_z=ground_z)
        rendered.append({
            'frame_index': int(idx),
            'phase_name': frames[idx].phase_name,
            'phase_step_index': frames[idx].phase_step_index,
            'image': filename,
        })
    manifest = {
        'schema_version': 'v3.0.16.visualization_manifest',
        'command_source': command_source,
        'filter_window': int(filter_window),
        'frame_count': len(frames),
        'rendered_frame_count': len(rendered),
        'rendered_frames': rendered,
        'dominant_failure_category': (whole_eval or {}).get('failure_diagnosis', {}).get('dominant_failure_category'),
    }
    manifest_path = os.path.join(output_dir, 'manifest.json')
    with open(manifest_path, 'w') as f:
        f.write(json.dumps(manifest, indent=2, sort_keys=True))
        f.write('\n')
    html_path = os.path.join(output_dir, 'index.html')
    with open(html_path, 'w') as f:
        f.write('<!doctype html><html><head><meta charset="utf-8"><title>v3 roll visualization</title></head><body>\n')
        f.write('<h1>v3 roll visualization</h1>\n')
        f.write('<p>command_source=%s, filter_window=%s, dominant_failure=%s</p>\n' % (
            command_source, filter_window, manifest['dominant_failure_category']))
        for rec in rendered:
            f.write('<h2>frame %d / %s step %d</h2>\n' % (rec['frame_index'], rec['phase_name'], rec['phase_step_index']))
            f.write('<img src="%s" style="max-width: 900px; width: 100%%;">\n' % rec['image'])
        f.write('</body></html>\n')
    manifest['manifest_path'] = manifest_path
    manifest['html_path'] = html_path
    return manifest
