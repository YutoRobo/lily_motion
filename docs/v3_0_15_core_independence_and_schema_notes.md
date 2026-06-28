# v3.0.15 Core Independence and Diagnosis Notes

## Purpose

v3.0.15 stops adding new gait variants and reorganizes the current work as a
portable v3-core candidate.  The immediate goal is not to tune a successful gait.
The goal is to make sure the current project-contained v3 pipeline can be moved
to another environment without relying on the older LilyRobot / legacy IK code.

## v3-core boundary

The following are considered v3-core responsibilities:

- explicit robot geometry configuration
- FK / IK candidate generation and selection
- common `V3RollCandidate` / `V3MotionFrame` trajectory objects
- raw and filtered joint command evaluation
- ground clearance, inter-leg clearance, second-joint limit, contact drift
- failure diagnosis by phase / leg / role
- optional Gazebo command export by leg name

The following are not v3-core responsibilities:

- v1/v2 legacy reproduction
- legacy `LilyRobot` adapter
- external `legacy-src-path` or xacro loading
- a specific final rolling gait profile

## Dependency status

The independent v3 package does not include the older `lily_motion/` directory.
The v3 evaluation and sweep scripts import only `lily_motion_v3` plus standard
Python modules.

Optional Gazebo replay still depends on ROS/Gazebo at runtime, but it no longer
imports the older project package.  The Gazebo joint command order has been moved
into `lily_motion_v3/interface_config.py`.

## Current trajectory schema

`V3RollCandidate`

- `direction`
- `phases`
- `frames`
- `report`

`V3MotionFrame`

- `frame_index`
- `phase_index`
- `phase_name`
- `phase_step_index`
- `phase_step_count`
- `contact_state`
- `base_pose`
- `leg_roles`
- `foot_targets_body`
- `foot_targets_world`
- `joint_angles`
- `diagnostics`

This schema is intended to be shared by:

- v3-native generators
- future legacy-style generator adapter
- future QP / optimization generator
- filters
- whole-roll evaluator
- visualization and Gazebo export

## Failure diagnosis

`lily_motion_v3/failure_diagnosis.py` summarizes failures by category:

- generator IK failure
- filtered ground penetration
- filtered inter-leg near-contact
- filtered second-joint violation
- filtered contact drift soft violation
- filtered contact drift hard violation

For each category it reports:

- count
- first record
- by-phase histogram
- by-leg histogram
- by-role histogram
- top records

Use:

```bash
python run_v3_0_diagnose_failures.py
```

or inspect `whole_roll_evaluation.failure_diagnosis` from
`run_v3_0_whole_roll_eval.py`.

## Recommended next stages

1. Keep v3-core stable.
2. Add a 3D visualizer using the common candidate schema.
3. Add a legacy-style generator that outputs the same `V3RollCandidate` schema.
4. Compare v3-native and legacy-style candidates using the same evaluator.
5. Only after that, run broader parameter search.
