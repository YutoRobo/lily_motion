# v3.0.22 Legacy State-Machine Replay

## Purpose

v3.0.21 reproduced the visible five-goal structure of `roll(Direction.FORWARD)`, but it still generated commands through the v3 abstraction.  That was not faithful enough.

v3.0.22 adds a vendored legacy runtime emulator.  It uses the supplied legacy files as local code inside `lily_motion_v3/legacy_runtime/` and executes the same update pattern:

- `ServoMotor`
- `EndEfectorManager`
- `LilyRobot`
- `Leg.calcAnalyticalInverse`
- `supportMove / landingMove / swingMove`

No ROS/catkin package is imported by this generator.

## Important replay fix

Earlier guidance used `tools/gazebo/run_v3_0_gazebo_replay.py --command-log <file>` as if `--command-log` were an input. In v3.0.21 it was actually an output path when generating a native candidate. This could accidentally replay the wrong generated candidate.

In v3.0.22, if `--command-log` points to an existing JSONL containing `joint_command_rad`, replay uses it as the input command log.

## Generate legacy-state-machine commands

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_state_machine_replay.py \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --initialize-step 100 \
  --output testdata/v3_0_22_legacy_state_machine_commands.jsonl
```

To include the initialization transition in the replay:

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_state_machine_replay.py \
  --include-initialize \
  --initialize-step 100 \
  --output testdata/v3_0_22_legacy_state_machine_with_init_commands.jsonl
```

## Diagnose command amplitude

```bash
python tools/diagnostics/run_v3_0_command_diagnostics.py \
  --command-log testdata/v3_0_22_legacy_state_machine_commands.jsonl
```

## Gazebo dry-run

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_22_legacy_state_machine_commands.jsonl \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --verbose-publish
```

## Gazebo replay

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_22_legacy_state_machine_commands.jsonl \
  --gazebo-link-state-log testdata/v3_0_22_legacy_state_machine_gazebo_link_states.jsonl \
  --verbose-publish
```

## Current status

This version is a better reproduction route than `legacy_roll_spec`, because it preserves the legacy state update order and servo/IK branch behavior. It is still not guaranteed to match the original ROS program perfectly until compared in Gazebo.
