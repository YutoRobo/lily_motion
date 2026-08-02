# URDF FK Source Of Truth

The source of truth for future root/mount geometry is the Lily URDF/xacro, not `default_octpus_mounts()`.

## Source

- xacro source: `/home/yuto/catkin_ws/src/lily_octpus/lily_octpus_desctiption/robots/robot.urdf.xacro`
- captured robot description: `testdata/body_root_transform_geometry_check/robot_description_from_ros.xml`

## Geometry

- body side: `0.21339745962155612 m`
- coxa/base_clause length: `0.075 m`
- thigh length: `0.3 m`
- tibia length: `0.3 m`

URDF leg roots are body cube vertices. The first actual revolute joint is `${prefix}_base_clause_joint`, with local axis `1 0 0`. The thigh and tibia joints use local axis `0 0 1` after their URDF joint origins and rotations.

## Difference From Python Fallback FK

`default_octpus_mounts()` uses `body_half_x/y/z=0.2` and yaw-only leg roots. That does not match the URDF root positions or axis convention. Existing command logs remain valid joint-angle logs, but future FK-based diagnostics and candidate generation should use URDF-derived FK or an explicitly URDF-derived mount config.

## Evaluator

- `lily_motion_v3/urdf_kinematics.py` parses robot_description and computes URDF joint-chain FK.
- `tools/diagnostics/urdf_fk_evaluator.py` evaluates `commands.jsonl` using the URDF convention.
- Output: `testdata/urdf_fk_evaluation/`

Older primitive geometry reports based on the fallback FK are stale for hardware geometry decisions.

## Safety state

- data_reference_candidates_modified=false
- tools_can_interface_modified=false
- publish_cmdforjetson_jsonl_modified=false
- can0_opened=false
- hardware_can_sent=false
