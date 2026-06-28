# v3.0.15 Dependency Report

## Summary

This zip is a v3-only distribution.  It intentionally excludes the older
`lily_motion/` package and the old v1/v2/legacy scripts.

## No longer required for v3 evaluation

The following are not required by v3 evaluation/sweep scripts:

- `legacy-src-path`
- legacy `LilyRobot`
- hidden legacy inverse kinematics
- `robot.xacro`
- `rospkg`
- ROS/Gazebo

## Required for v3 evaluation and sweep

- Python standard library only
- local `lily_motion_v3` package

Entry points that do not require ROS/Gazebo:

- `run_v3_0_whole_roll_eval.py`
- `run_v3_0_parameter_sweep.py`
- `run_v3_0_goal_oriented_sweep.py`
- `run_v3_0_contact_plan_catalog.py`
- `run_v3_0_diagnose_failures.py`
- `run_v3_0_export_commands.py`

## Optional ROS/Gazebo dependency

`run_v3_0_gazebo_replay.py` requires ROS/Gazebo only when not using `--dry-run`.
It imports the self-contained `lily_motion_v3.ros_bridge` module, not the older
`lily_motion` package.

## Known external interface assumption

Gazebo command export assumes the existing Lily Gazebo controller topic order.
The order is checked into:

- `lily_motion_v3/interface_config.py`

This is an interface configuration, not a dependency on old source code.

## String-search check

The v3-only distribution should not contain imports of:

- `from lily_motion.`
- `import lily_motion.`
- `legacy-src-path`
- `LilyRobot`
- `xacro-path`

except where these strings are mentioned in documentation as things not required.
