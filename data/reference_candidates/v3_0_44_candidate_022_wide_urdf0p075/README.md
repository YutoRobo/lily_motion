# v3_0_44_candidate_022_wide_urdf0p075

`candidate_022_wide` is frozen here as the URDF 0.075 geometry reference candidate for pre-hardware review. The old candidate02 reference directories are not overwritten.

## Decision

- status: reference candidate, pre-hardware first candidate
- Gazebo full_roll normal: PASS
- URDF FK primitive clearance: `0.019648551564994537 m`
- candidate02 URDF FK primitive clearance: `0.01647142183967449 m`
- improvement over candidate02: about `0.003177 m`
- hardware status: not_tested

## Notes

The URDF FK evaluator is the geometry source of truth for this candidate. Older Python fallback FK reports based on fallback roots or old axis conventions are stale. The `urdf_worst_1144_1184` Gazebo window had an initial blow-up observation that was not reproduced; adoption is based on full_roll normal PASS.

## Files

- `commands.jsonl`: frozen candidate_022_wide command log
- `manifest.json`: source, checksums, geometry, and safety state
- `summary.json`: line counts, staged-log checksums, and review status
- `pre_hardware_decision.md`: staged hardware decision memo
- `reports/`: copied or summarized evaluation reports
- `staged/`: first-run and staged roll logs
