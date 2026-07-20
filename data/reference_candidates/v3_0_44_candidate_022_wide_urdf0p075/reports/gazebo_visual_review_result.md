# Gazebo Visual Review Result: candidate_022_wide

## Result

- candidate: `candidate_022_wide`
- full_roll normal: PASS
- urdf_worst_1144_1184: initial blow-up observed, but not reproduced
- adoption decision: PASS as pre-hardware first candidate

## Interpretation

The primary adoption basis is the full_roll normal Gazebo replay passing without the visual near-contact issue or posture jump side effect. The urdf_worst_1144_1184 window is recorded as a known note because an initial blow-up was observed once, but it was not reproduced and is not treated as a candidate rejection reason.

## Safety State

- can0_opened=false
- hardware_can_sent=false
- tools_can_interface_modified=false
- publish_cmdforjetson_jsonl_modified=false
