# v3.0.43C middle swing Y bump

## Purpose

This experiment keeps provisional baseline v2 as the reference and tries a simple reversible middle-leg swing Y bump:

```text
baseline v2 at lift-off -> outward Y apex during RF-3 -> baseline v2 at RF-4 landing
```

The baseline v2 trajectory is not overwritten.  You can regenerate it at any time with:

```bash
python run_v3_0_provisional_baseline_v2.py \
  --output-dir testdata/provisional_baseline_v2
```

Gazebo replay for the regenerated baseline v2:

```bash
python run_v3_0_gazebo_replay.py \
  --command-log testdata/provisional_baseline_v2/provisional_baseline_v2_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log
```

## Main sweep command

```bash
python run_v3_0_43c_middle_swing_y_bump_sweep.py \
  --middle-swing-y-escapes 0.00,0.05,0.10,0.15,0.20 \
  --middle-swing-y-escape-modes outward \
  --middle-swing-y-escape-phases rf3_only \
  --constraint-stride 8 \
  --save-top-n 4 \
  --output testdata/v3_0_43c_middle_swing_y_bump_sweep.json \
  --candidate-output-dir testdata/v3_0_43c_candidates
```

`rf3_only` means:

- RF-3 middle-pair lift apex receives the outward Y offset.
- RF-4 middle-pair landing target stays at the baseline-v2 Y coordinate.
- Setting `middle_swing_y_escape=0.0` and `mode=none` reproduces baseline v2.

## Pre-generated quick-check logs

The package may include `testdata/v3_0_43c_single/` with these command logs:

- `baseline_v2_x8_sw40_commands.jsonl`
- `E005_x8_sw40_commands.jsonl` : outward bump amplitude 0.05 m
- `E010_x8_sw40_commands.jsonl` : outward bump amplitude 0.10 m
- `E015_x8_sw40_commands.jsonl` : outward bump amplitude 0.15 m
- `E020_x8_sw40_commands.jsonl` : outward bump amplitude 0.20 m

Replay example:

```bash
python run_v3_0_gazebo_replay.py \
  --command-log testdata/v3_0_43c_single/E010_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log
```

## Acceptance policy

Do not promote v3.0.43C only from numeric ranking.  Check Gazebo visually:

1. middle legs visibly open during RF-3/RF-4 swing,
2. 3rd/4th rotation does not show abrupt joint jumps,
3. rear-leg singular-posture avoidance remains no worse than baseline v2,
4. RF-5 and later posture is not worse than baseline v2.

If any of those fail, return to baseline v2 and do not adopt the candidate.
