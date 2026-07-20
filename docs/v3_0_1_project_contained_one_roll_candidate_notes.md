# v3.0.1 Project-contained one-roll candidate skeleton

## Purpose

v3.0.1 advances v3.0 from a kinematics-only foundation to an executable one-roll candidate skeleton.

The point is not to finalize a good rolling gait yet. The point is to prove that the project can now generate a role-based roll candidate without calling legacy `LilyRobot`, legacy IK, or `legacy-src-path`.

## Added files

```text
lily_motion_v3/roll_candidate.py
lily_motion_v3/role_utils.py
lily_motion_v3/v3_roll_candidate_generator.py
archive/v3_experiment_scripts/run_v3_0_concept_roll.py
docs/v3_0_1_project_contained_one_roll_candidate_notes.md
```

## Main structure

The v3.0.1 generator runs the following sequence:

```text
RobotModel
  -> project-contained FK/IK

PhaseSpec / ContactState
  -> role-based roll phases

V3RollCandidateGenerator
  -> foot target interpolation
  -> IK candidate generation
  -> IK candidate selection
  -> joint/discontinuity/reachability report

V3RollCandidate
  -> frames + report
```

## Role-based phases

The generated candidate uses these conceptual v3 phases:

```text
StableInitialContact
ClearancePreparation
EstablishNextSupportCandidates
LiftTransitionLegs
ConstrainedBodyRoll
SupportTransfer
PostureNormalization
```

These phases are not legacy RF-1 to RF-6 calls. They are explicit v3 role phases.

## What is project-contained now

v3.0.1 does not require legacy IK.

It contains:

```text
- leg mount definitions
- leg FK
- leg IK candidate generation
- IK candidate selection
- contact state definitions
- leg role definitions
- one-roll candidate frames
- basic integrated report
```

## What is still deliberately simplified

This is still not the final gait.

The following are intentionally approximate:

```text
- default leg mount geometry
- default initial joint posture
- foot target policies
- body roll treatment
- ground/contact consistency
- inter-leg collision evaluation inside v3 frames
```

The generator keeps `legacy_dependency = false`, but it is not yet a Gazebo-ready production roll gait.

## Run command

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py --summary-only
```

Full JSON output:

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py \
  --steps-per-phase 8 \
  --output testdata/v3_0_1_concept_roll_forward.json
```

## Example summary fields

```text
phase_count
frame_count
phase_names
report.task_success.legacy_dependency
report.ik_reachability.ik_failure_count
report.joint_limit.max_abs_second_joint_deg
report.motion_discontinuity.max_joint_delta_deg
```

## Test result

```text
Ran 42 tests
OK
```

## Next step

The next step should not be a legacy RF parameter tweak.

The next step should be to replace the placeholder foot-target policy with a constraint-aware policy:

```text
CandidateFootTargetGenerator
  -> generate multiple targets per role
  -> reject unreachable targets
  -> reject second-joint limit violations
  -> reject low-clearance/inter-leg-risk candidates
```

Then the v3 generator can choose a feasible target rather than accepting a single hard-coded target.
