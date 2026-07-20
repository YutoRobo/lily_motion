# v3.0.11 Soft Contact Drift / Filter Window Sweep Notes

## Purpose

v3.0.10 added a contact-preserving re-projection filter. It can force SUPPORT feet back to the locked contact point, but it may also reintroduce large 180 deg-class joint changes.

v3.0.11 changes the default direction: contact drift after moving-average filtering is no longer treated as a strict zero constraint. Instead, drift is evaluated with two limits:

- soft limit: warning / scoring threshold
- hard limit: pass-fail threshold

This matches the current design assumption: raw contact-locked commands may pass through singular/flip-like configurations, and moving-average filtering may slightly move the realized contact point. The important constraints are that the drift is not excessive, the robot does not penetrate the floor, legs do not collide, and the filtered command remains smooth enough.

## New CLI options

`tools/diagnostics/run_v3_0_whole_roll_eval.py`:

```bash
--contact-drift-soft-limit 0.05
--contact-drift-hard-limit 0.15
```

`--contact-drift-warn` remains as a deprecated alias for the soft limit.

`tools/diagnostics/run_v3_0_parameter_sweep.py`:

```bash
--filter-windows 3,5,7,9
--contact-drift-soft-limit 0.05
--contact-drift-hard-limit 0.15
```

The sweep now includes filter window as a parameter. This is important because the filter window directly trades off joint smoothness against contact drift.

## Recommended command

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py \
  --contact-plan-variants front_pair_roll,rear_pair_roll \
  --steps-per-phase 6 \
  --lift-heights 0.08 \
  --clearance-heights 0.06 \
  --candidate-support-shift-xs 0.04 \
  --candidate-support-drop-zs=-0.02 \
  --filter-windows 3,5,7,9 \
  --contact-drift-soft-limit 0.05 \
  --contact-drift-hard-limit 0.15 \
  --output testdata/v3_0_11_filter_window_sweep_quick.json
```

## Example result observed during packaging

For `front_pair_roll`, `steps_per_phase=6`:

- moving average window 5:
  - max contact drift: about 0.164 m
  - hard drift violations: 4, with hard limit 0.15 m
  - max filtered joint delta: 36 deg

The quick sweep selected window 3 as the best of `3,5,7,9` for the current parameters:

- max contact drift: about 0.106 m
- hard drift violations: 0
- max filtered joint delta: 60 deg
- filtered penetration count: 2

This is not a successful gait yet. It only shows that allowing soft contact drift gives a more realistic trade-off than forcing exact contact re-projection.

## Design implication

v3.0.11 should be read as a correction of the evaluation philosophy:

- raw trajectory should preserve contact locks as much as possible
- filtered trajectory may drift within a configured tolerance
- contact drift should be scored, not automatically forced to zero
- filter window is now a gait parameter

The remaining major issues are still:

- IK failures during the roll
- filtered ground penetration
- contact plan / support set design

