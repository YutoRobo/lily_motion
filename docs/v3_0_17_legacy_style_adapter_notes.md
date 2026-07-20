# v3.0.17 Legacy-style adapter notes

## Purpose

v3.0.17 adds a **legacy-style adapter** that connects the old qualitative roll idea to v3-core without importing the old project.

The adapter is not a bit-for-bit reproduction of the legacy RF implementation.  It is a v3-core scaffold that represents the same design direction:

- six-leg contact preparation,
- middle/transition-leg step-over,
- singular/flip-prone raw commands are allowed,
- moving-average filtering is evaluated separately,
- floor, inter-leg clearance, second-joint limit, and contact drift are evaluated by the common v3 evaluator.

## Added files

- `lily_motion_v3/legacy_style_generator.py`
- `tools/diagnostics/run_v3_0_legacy_style_eval.py`
- `docs/v3_0_17_legacy_style_adapter_notes.md`

## New contact plan variant

`legacy_six_middle_roll` was added to `v3_roll_concept_generator.py`.

It encodes a six-contact preparation stage and lifts the middle transition pair before body roll.  This variant is intended as a bridge between v3-core and the legacy algorithm, not as the final exact legacy reproduction.

## Example

```bash
python tools/diagnostics/run_v3_0_legacy_style_eval.py \
  --summary-only \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 1.0 \
  --rf2-x-scale 1.0 \
  --filter-window 3
```

Try RF2 scale variants:

```bash
python tools/diagnostics/run_v3_0_legacy_style_eval.py \
  --summary-only \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 0.8 \
  --rf2-x-scale 0.6 \
  --filter-window 3
```

## Dependency policy

The adapter does not use:

- `legacy-src-path`,
- old `LilyRobot`,
- old IK,
- xacro parsing,
- `lily_motion` v1/v2 modules.

The exact legacy-RF numerical profile can later be added as a generator that still returns the same v3 `V3RollCandidate` schema.
