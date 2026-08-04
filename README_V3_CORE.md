# Lily Motion v3-core independent package

Updated: 2026-08-04

This package contains the project-contained v3 rolling-gait core. It is not the old legacy-connected evaluator. The v3 evaluation pipeline is intended to run without `legacy-src-path`, legacy `LilyRobot`, hidden legacy IK, or xacro loading.

This document describes development and evaluation entry points. It does not define the current hardware trial order. For the current integrated status and hardware candidate, use:

- [`README.md`](README.md)
- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)

## Current Geometry

Shared geometry constants are defined in:

```text
lily_motion_v3/robot_geometry.py
```

```text
coxa  = 0.075 m
thigh = 0.300 m
tibia = 0.300 m
```

Older primitive FK reports generated with a 0.05 m coxa default are stale for exact geometry decisions. Existing joint-angle command logs are not automatically stale because of the geometry update.

## Current Frozen Pre-Hardware Candidate

The current first pre-hardware rolling candidate is:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

Recorded status:

```text
command count: 2233
maximum second-joint angle: 94.8 deg
violations over 95 deg: 0
Gazebo full roll: PASS
hardware full roll: not tested
```

The generic v3-core commands below are development and diagnostic tools. Their output is not automatically approved for hardware execution. Hardware must use reviewed and frozen files under `data/reference_candidates/`.

## Quick Evaluation

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

A useful development example:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py \
  --summary-only \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --lift-height 0.12 \
  --clearance-height 0.06 \
  --candidate-support-drop-z=-0.02 \
  --filter-window 3 \
  --body-roll-pitch-deg 60
```

This example is not the frozen v3.0.44 hardware command.

## Failure Diagnosis

```bash
python tools/diagnostics/run_v3_0_diagnose_failures.py
```

## Parameter Sweep

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py \
  --trajectory-modes phase,synchronized \
  --synchronized-steps 48,72 \
  --contact-plan-variants front_pair_roll,diagonal_front_roll \
  --steps-per-phase 6 \
  --lift-heights 0.08,0.12,0.16 \
  --clearance-heights 0.06,0.10,0.14 \
  --candidate-support-shift-xs 0.04 \
  --candidate-support-drop-zs=-0.02,0.0,0.02 \
  --filter-windows 3,5 \
  --body-roll-pitch-deg 60 \
  --output testdata/v3_0_15_sweep.json
```

Sweep results belong in `testdata/` until reviewed and frozen. Do not stream arbitrary sweep output to hardware.

## Optional Gazebo Dry Run

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_15_commands.jsonl
```

Live Gazebo replay requires ROS/Gazebo, but not the older external Lily motion package.

For the current frozen candidate:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 0.0 \
  --hold-end-sec 0.0 \
  --repeat-last 0 \
  --diagnose-command-log
```

The frozen strict dry-run result is PASS.

## Standalone Visualization

Use the visualizer before Gazebo when inspecting overall kinematic motion.

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --lift-height 0.12 \
  --clearance-height 0.06 \
  --candidate-support-drop-z=-0.02 \
  --filter-window 3 \
  --body-roll-pitch-deg 60 \
  --command-source filtered \
  --output-dir testdata/v3_0_16_visualization
```

Then open:

```text
testdata/v3_0_16_visualization/index.html
```

## Legacy-Style Adapter

A legacy-style adapter is included as:

```text
tools/diagnostics/run_v3_0_legacy_style_eval.py
lily_motion_v3/legacy_style_generator.py
```

It maps legacy-like parameters such as `step_scale`, `splited_num`, `rf2_pitch_scale`, and `rf2_x_scale` into the common v3 candidate/evaluator path without importing the old project.

This is a comparison scaffold, not an exact reproduction of every historical RF-1 through RF-6 implementation.

## Command Export

`tools/command_generation/run_v3_0_export_commands.py` supports v3-native and legacy-style development profiles.

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile native \
  --command-source filtered \
  --output testdata/native.jsonl

python tools/command_generation/run_v3_0_export_commands.py \
  --profile legacy_style \
  --command-source filtered \
  --output testdata/legacy_style.jsonl
```

Replay is a separate step:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/native.jsonl
```

Generated development logs must pass diagnostics, Gazebo review, and candidate-freeze review before they are considered for hardware.

## Imported Legacy/Reference Trajectories

Use the reference importer when an actual legacy joint command log is available.

```bash
python tools/command_generation/run_v3_0_import_legacy_reference.py \
  --input testdata/legacy_published_commands.jsonl \
  --input-format auto \
  --input-unit rad \
  --filter-window 3 \
  --candidate-output testdata/legacy_reference_candidate.json \
  --command-output testdata/legacy_reference_commands.jsonl \
  --command-source raw
```

Gazebo replay:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/legacy_reference_commands.jsonl \
  --gazebo-link-state-log testdata/legacy_reference_gazebo_link_states.jsonl \
  --verbose-publish
```

## Runtime Boundary

The v3-core does not send hardware CAN directly.

Approved runtime path:

```text
frozen 24-axis JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ tools/can_interface/statemachine/StateMachine
→ Use=True axes
→ CAN
```

CAN and hardware safety details:

- [`tools/can_interface/README.md`](tools/can_interface/README.md)
- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md)

## Archive Boundary

Scripts under `archive/v3_experiment_scripts/` are retained for historical reproduction and old experiment review. They are not current hardware entry points.

## Development Rules

- Keep generated exploratory output in `testdata/`.
- Do not edit frozen `data/reference_candidates/` command logs in place.
- Record manifest, summary, checksum, geometry, source commit, and evaluation basis when freezing a candidate.
- Treat URDF-derived evaluation as the geometry source of truth for the current 0.075 m decision.
- Do not treat a successful generic evaluator run as hardware approval.
- Do not add another production position topic beside `/cmdForJetson`.
