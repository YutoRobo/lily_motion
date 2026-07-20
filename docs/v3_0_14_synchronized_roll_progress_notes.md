# v3.0.14 synchronized roll-progress generator

## Purpose

v3.0.14 adds an optional synchronized trajectory mode.  The older generator was
phase-local: body pitch, support roles, and foot target changes were each driven
mostly by phase-local alpha.  That made it easy to miss the real problem: during
a roll, the body rotates while support, lift, clearance, and landing legs are
also changing at every step.

The new mode introduces one roll progress variable:

```text
s in [0, 1]
```

and drives the following from the same `s`:

- body pitch progress
- active support set
- candidate support placement
- lift / clearance profiles
- persistent contact locks
- base x/z pose search
- IK target conversion at every frame

This is still not a final gait.  It is a trajectory-level search scaffold.

## Commands

Phase-local legacy v3 behavior:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

Synchronized-progress behavior:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py \
  --summary-only \
  --trajectory-mode synchronized \
  --synchronized-steps 72 \
  --contact-plan-variant front_pair_roll \
  --body-roll-pitch-deg 60 \
  --filter-window 3
```

Sweep example:

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py \
  --trajectory-modes phase,synchronized \
  --synchronized-steps 48,72 \
  --contact-plan-variants front_pair_roll,diagonal_front_roll \
  --steps-per-phase 6 \
  --lift-heights 0.08 \
  --clearance-heights 0.06 \
  --candidate-support-shift-xs 0.04 \
  --candidate-support-drop-zs=-0.02 \
  --filter-windows 3,5 \
  --body-roll-pitch-deg 60 \
  --output testdata/v3_0_14_sync_phase_compare.json
```

## Interpretation

The synchronized mode answers a different question from previous versions:

```text
When the body is rotating and the legs are also moving at every step, which
contact plan and parameter set fails least badly over the entire roll?
```

Current smoke-test results still do not produce a successful gait.  That is
expected.  The value of v3.0.14 is that failures are now evaluated under a more
realistic synchronized time structure.

## Roadmap impact

This does not remove the need for the legacy RF compatibility layer.  It just
adds a v3-native synchronized branch for whole-roll feasibility search.

The two branches should remain in the roadmap:

1. v3-native synchronized roll-progress search.
2. legacy-style RF profile adapter evaluated by the same whole-roll evaluator.

