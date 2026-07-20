# v3.0.23 command smoothing and diagnostics

## Purpose

v3.0.22 moved the legacy reproduction from an abstract v3 gait generator to a vendored legacy state-machine emulator.  That made the order of the motion much closer to the original program, but Gazebo preview can still look jerky.

v3.0.23 does **not** change the legacy roll state machine.  It adds tools to inspect and optionally resample the generated command sequence:

- command range diagnostics
- adjacent-frame joint jump diagnostics
- phase-wise jump diagnostics
- command resampling for Gazebo preview
- optional moving-average smoothing for preview

The intent is to distinguish:

1. jerk caused by the legacy algorithm itself, especially short 3-step phases,
2. jerk caused by replay timing,
3. jerk caused by a few specific joints or phase boundaries.

## Added files

- `lily_motion_v3/command_resampler.py`
- `tools/command_generation/run_v3_0_resample_commands.py`
- updated `tools/diagnostics/run_v3_0_command_diagnostics.py`
- updated `tools/gazebo/run_v3_0_gazebo_replay.py`
- `tests/test_v3_0_command_resampler.py`

## Recommended workflow

Generate the legacy state-machine command log:

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_state_machine_replay.py \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --initialize-step 100 \
  --output testdata/v3_0_23_legacy_state_machine_commands.jsonl
```

Diagnose the raw command jumps:

```bash
python tools/diagnostics/run_v3_0_command_diagnostics.py \
  --command-log testdata/v3_0_23_legacy_state_machine_commands.jsonl \
  --top-joints 8
```

Create a smoother preview command log:

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_23_legacy_state_machine_commands.jsonl \
  --resample-factor 4 \
  --smooth-window 1 \
  --output testdata/v3_0_23_legacy_state_machine_resampled_x4.jsonl
```

Replay in Gazebo:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 60 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_23_legacy_state_machine_resampled_x4.jsonl \
  --diagnose-command-log \
  --verbose-publish
```

Alternatively, replay an existing command log with on-the-fly resampling:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 60 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_23_legacy_state_machine_commands.jsonl \
  --resample-factor 4 \
  --smooth-window 1 \
  --diagnose-command-log \
  --verbose-publish
```

## Interpretation

If resampling makes Gazebo much smoother, the main issue is preview discretization.  If it remains jerky, check `phase_summary` and `top_joints_by_adjacent_delta`; the legacy state machine itself is producing large per-step jumps, likely around short phases such as Goal 1, Goal 3, Goal 4, or StopAdjustment.

Smoothing is preview-only.  It can change contact consistency, so final evaluation should be performed both before and after smoothing.
