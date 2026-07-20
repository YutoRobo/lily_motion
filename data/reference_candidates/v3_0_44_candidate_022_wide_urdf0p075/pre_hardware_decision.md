# Pre-Hardware Decision: candidate_022_wide

## Decision

Use `v3_0_44_candidate_022_wide_urdf0p075` as the first pre-hardware candidate. Do not send the full roll log as the first hardware motion.

## Reasons

- full_roll normal Gazebo review: PASS
- URDF-derived FK primitive clearance improves over candidate02: `0.019648551564994537 m` vs `0.01647142183967449 m`
- old Python fallback FK evaluation is stale for the 0.075 geometry decision
- `urdf_worst_1144_1184` initial blow-up was observed once but not reproduced; it is recorded as a note, not a rejection reason

## First Hardware Log

Start with:

`data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl`

## Stage Order

1. `air_entry_and_hold_only_commands.jsonl`
2. `combined_with_hold_commands.jsonl` only after air-entry/hold is accepted
3. `roll_0_50_commands.jsonl`
4. `roll_50_100_commands.jsonl`
5. `roll_100_300_commands.jsonl`
6. `roll_300_end_commands.jsonl`

## Do Not Run First

Do not run `commands.jsonl` or `roll_300_end_commands.jsonl` as the first hardware test. Full-roll execution remains blocked until staged hardware checks pass.

## Stop Rule

If posture jump, unexpected contact, CAN/UI abnormality, command timing issue, or operator concern appears, issue STOP immediately and do not continue to the next stage.

## Safety State While Creating This Reference

- can0_opened=false
- hardware_can_sent=false
- external_can_interface_executed=false
