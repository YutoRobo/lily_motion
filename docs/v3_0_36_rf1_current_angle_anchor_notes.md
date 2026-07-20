# v3.0.36 RF-1 current-angle anchor

## Purpose

This version addresses the observed discontinuity at repeated roll boundaries:

- previous roll: `RF-6_StopAdjustment` final frame
- next roll: `RF-1_Goal1_UpperLegPreSwing` first frame

The diagnosis showed that the first RF-1 command could jump by roughly 170 deg
from the previous frame.  This was not fixed by replay rate, segmented smoothing,
continuous unwrap, or larger `max_step`, because the first RF-1 command itself was
already far from the terminal servo state.

## Change

A new option was added:

```bash
--rf1-current-angle-anchor
```

When enabled, the emulator emits one current-servo command frame at the beginning
of every RF-1 phase, before `setSwingLeg()` and `setSupportMove()` create the RF-1
interpolation queues.

This is intentionally not:

- boundary preblend
- moving average across roll boundaries
- constrained parameterization
- a change of leg roles or RF goal sequence

It only ensures that the RF-1 phase starts from the actual current servo state.

## Recommended command

```bash
python archive/v3_experiment_scripts/run_v3_0_pure_legacy_repeated_roll.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --rf1-current-angle-anchor \
  --skip-constraints \
  --output-command-log testdata/v3_0_36_pure_legacy_repeated_m30_anchor_commands.jsonl \
  --report-output testdata/v3_0_36_pure_legacy_repeated_m30_anchor_report.json
```

Then resample inside each roll segment:

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_36_pure_legacy_repeated_m30_anchor_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --segment-key roll_index \
  --diagnose-boundaries \
  --output testdata/v3_0_36_pure_legacy_m30_anchor_x8_sw3.jsonl
```

Replay strictly from the generated log:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --strict-command-log-input \
  --rate 80 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_36_pure_legacy_m30_anchor_x8_sw3.jsonl
```

## Expected diagnostic

Boundary diagnostics should show that the transition from RF-6 final to the RF-1
anchor frame is near zero.  The next transition, from the anchor frame to the
first RF-1 interpolation frame, should be much smaller than the original one-shot
jump because it is only the first interpolation increment.
