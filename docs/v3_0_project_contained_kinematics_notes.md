# v3.0 Project-contained kinematics foundation

## Purpose

v3.0 must be portable to another environment. Therefore, the motion generator must not depend on `legacy-src-path`, legacy `LilyRobot`, or a hidden legacy inverse kinematics implementation.

This patch adds the first self-contained v3 foundation:

- `lily_motion_v3/leg_config.py`
- `lily_motion_v3/leg_kinematics.py`
- `lily_motion_v3/robot_model.py`
- `lily_motion_v3/contact_state.py`
- `lily_motion_v3/leg_role.py`
- `lily_motion_v3/phase_spec.py`
- `lily_motion_v3/constraints.py`
- `lily_motion_v3/motion_evaluation_report.py`
- `lily_motion_v3/v3_roll_concept_generator.py`

## Scope

This is not yet a complete v3 roll gait. It is Step 0.

It establishes:

1. project-contained robot/leg geometry,
2. project-contained FK,
3. project-contained IK candidate generation,
4. explicit IK candidate selection,
5. contact-state and leg-role vocabulary,
6. a first role-based roll concept phase list.

## IK convention

The v3 3-DOF leg convention is:

- `q0`: yaw around local z
- `q1`: thigh pitch
- `q2`: tibia pitch relative to thigh

The default link lengths are initialized from the observed legacy values:

```text
coxa  = 0.05 m
thigh = 0.30 m
tibia = 0.30 m
```

These are now explicit project parameters, not a runtime dependency on legacy code.

## Important design decision

v3 does **not** expose only:

```text
inverse_kinematics(target) -> q
```

Instead, it exposes:

```text
inverse_kinematics_candidates(target) -> candidates
select_candidate(candidates, constraints, previous_q) -> selected_q
```

This is necessary because the current problems involve branch selection, unnecessary flips, second-joint limits, and continuity.

## Next step

The next real implementation step is not to tune v2 RF phases. It is to connect this v3 kinematic layer to a minimal one-roll candidate generator and then evaluate it with a unified `MotionEvaluationReport`.
