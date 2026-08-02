# Body Root Transform Geometry Check: 0.075 m Coxa

## Purpose

This note supersedes the earlier interpretation that the first-segment/coxa tip bounding box must be `0.3 m`. That condition is only meaningful for a model that treats the coxa tip as a virtual root on an outer cube. The current project-contained IK uses all three leg segments (`coxa`, `thigh`, and `tibia`), so a coxa-tip bbox different from `0.3 m` is not an immediate rejection by itself.

This audit does not modify `data/reference_candidates`, command logs, CAN tools, or publisher tools.

## Corrected 0.3 m Interpretation

`0.3 m` has multiple meanings in the codebase:

- Active link length: `THIGH_LENGTH = 0.3` and `TIBIA_LENGTH = 0.3` in `lily_motion_v3/robot_geometry.py`.
- Conditional coxa-tip cube: `a_body + 2 * coxa_length / sqrt(3) = 0.3` only if body roots are cube vertices and the coxa extends along outward body diagonals.
- Legacy coordinate frame: generated legacy transforms contain repeated `0.15` offsets, likely half of a `0.3 m` outer coordinate span.
- Legacy body parameter: `LilyRobot.__body = 0.3` and `LegacyStateMachineEmulator.setRobotParam(body=0.3)` preserve legacy behavior.
- Motion/search parameters: several `0.3` values are gait distances or search offsets, not body/root geometry.

See `testdata/body_root_transform_geometry_check/geometry_0p3_classification.csv`.

## Python FK/IK Finding

`LegKinematics.forward_kinematics()` uses:

```text
radial = coxa + thigh * cos(q1) + tibia * cos(q1 + q2)
z      =        thigh * sin(q1) + tibia * sin(q1 + q2)
```

`LegKinematics.inverse_kinematics_candidates()` computes `q0 = atan2(y, x)`, subtracts `coxa_length` from the radial distance, then solves the remaining thigh/tibia two-link problem. Therefore IK does use all three segments. The `0.3 m` cube condition is not required for this IK formulation.

## HOME Python FK Reference

For HOME `[0, 0, 0]`, the current Python fallback root model is `default_octpus_mounts(body_half_x/y/z=0.2)` with yaw-only leg roots. The generated reference table is:

- `testdata/body_root_transform_geometry_check/home_python_fk_link_positions.csv`

Each leg contains `mount`, `coxa_end`, `knee`, and `foot` body-frame points. Under the current Python fallback, the first joint axis is body-frame `+z`.

## URDF/Gazebo Consistency Status

No `*.urdf`, `*.xacro`, or robot XML file was found in this repository, so direct URDF-vs-Python FK comparison could not be completed from local files. The URDF is reported as externally updated to `coxa_length = 0.075 m`, but that geometry is not available here for parsing.

The next geometry gate should be: compare HOME `[0,0,0]` link poses from the corrected URDF/Gazebo model against the Python FK reference table. If URDF roots use diagonal first-joint transforms, they are expected to differ from the current `default_octpus_mounts()` fallback until Python root transforms are updated to match URDF.

See `testdata/body_root_transform_geometry_check/urdf_fk_consistency_check.md`.

## Redefined Cube Condition

The `0.3 m` outer cube condition applies only to the coordinate system where the coxa tips are intentionally defined as an outer cube around an inner body cube:

```text
a_body = 0.3 - 2 * coxa_length / sqrt(3)
```

It does not automatically apply to foot targets, the current yaw-only Python mount fallback, or the 3-segment IK solver.

## Outputs

- `testdata/body_root_transform_geometry_check/geometry_0p3_classification.csv`
- `testdata/body_root_transform_geometry_check/home_python_fk_link_positions.csv`
- `testdata/body_root_transform_geometry_check/urdf_fk_consistency_check.md`
- `testdata/body_root_transform_geometry_check/corrected_geometry_summary.json`
- Historical/reference-only: `body_root_points.csv`, `first_segment_tip_points.csv`, `bounding_box_summary.json`

## Safety State

- data_reference_candidates_modified=false
- tools_can_interface_modified=false
- publish_cmdforjetson_jsonl_modified=false
- can0_opened=false
- hardware_can_sent=false
