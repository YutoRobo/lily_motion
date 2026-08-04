# Pre-Hardware Decision: v3.0.44 candidate_022_wide

更新日: 2026-08-04

## 1. Decision

最初の回転歩容pre-hardware候補として、次を使用する。

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

ただし、`commands.jsonl`またはfull sequenceを最初の実機動作として送信してはならない。単軸、同一脚3軸、air-entry、接地、分割rollを順番に確認する。

## 2. Adoption Basis

- candidate name: `v3_0_44_candidate_022_wide_urdf0p075`
- command count: `2233`
- coxa length: `0.075 m`
- thigh length: `0.300 m`
- tibia length: `0.300 m`
- maximum second-joint angle: `94.8 deg`
- second-joint violations over `95 deg`: `0`
- full-roll Gazebo review: `PASS`
- URDF-derived FK evaluation is the source of truth for the 0.075 m geometry decision
- hardware full roll: not tested

The initial `urdf_worst_1144_1184` blow-up was observed once and was not reproduced. It remains a recorded note, not a rejection reason.

## 3. Current Hardware Status

Confirmed as of 2026-08-04:

- `/cmdForJetson` unified position path: software and vcan PASS
- vcan axis10 single-axis out-and-back: PASS
- vcan axis10,11,12 fan-out: PASS
- real axis10 `+0.002 rad` out-and-back: visually provisional PASS

Not yet confirmed:

- real axis10 negative `0.002 rad`
- real axis10 `+/-0.005 rad`
- one complete leg, three real axes
- multiple real actuators operating together
- air-entry, touchdown, staged roll, or full roll on hardware

## 4. Mandatory Stage Order

The approved order is:

```text
real axis10 negative 0.002 rad
→ real axis10 positive/negative 0.005 rad
→ one complete leg, three axes
→ air_entry_and_hold_only
→ touchdown confirmation
→ roll_0_50
→ roll_50_100
→ roll_100_300
→ roll_300_end
→ combined_with_hold as the final full-sequence confirmation
```

Do not move to the next stage unless the current stage passes.

## 5. Staged Command Logs

### 5.1 Air-entry and hold only

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/air_entry_and_hold_only_commands.jsonl
```

- line count: `135`
- contains air-entry and hold only
- contains no roll-body frames

### 5.2 Roll 0–50

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/roll_0_50_commands.jsonl
```

- line count: `50`
- source roll indexes: `0..49`

### 5.3 Roll 50–100

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/roll_50_100_commands.jsonl
```

- line count: `50`
- source roll indexes: `50..99`

### 5.4 Roll 100–300

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/roll_100_300_commands.jsonl
```

- line count: `200`
- source roll indexes: `100..299`

### 5.5 Roll 300–end

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/roll_300_end_commands.jsonl
```

- line count: `1933`
- source roll indexes: `300..2232`

This stage is not an early-stage test. It is used only after all shorter roll segments pass.

### 5.6 Final combined sequence

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/combined_with_hold_commands.jsonl
```

- line count: `2368`
- includes air-entry, hold, and the complete `2233`-frame roll body
- final confirmation only

`combined_with_hold_commands.jsonl` must not be executed before the four split roll stages pass.

## 6. Commands That Must Not Be Run First

Do not use these as the first hardware command:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_300_end_commands.jsonl
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/combined_with_hold_commands.jsonl
```

## 7. Stop Rule

Issue STOP immediately and end the current stage when any of the following occurs:

- posture jump
- unexpected contact or interference
- unexpected axis motion
- failure to return to the commanded center
- CAN or UI abnormality
- `0x0EE` error
- CAN send error or interface error
- command timing concern
- abnormal sound, current, heat, or vibration
- any operator concern

After a STOP caused by abnormal behavior, do not continue to the next stage without reviewing the CAN log, command log, software commit, and physical state.

## 8. Safety State When Candidate Was Frozen

- `can0_opened=false`
- `hardware_can_sent=false`
- `external_can_interface_executed=false`

These values describe candidate creation and Gazebo evaluation, not the later axis10 hardware test.

## 9. Related Documents

- [`../../../README.md`](../../../README.md)
- [`../../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../../docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`../../../docs/HARDWARE_PRETEST_STATUS.md`](../../../docs/HARDWARE_PRETEST_STATUS.md)
- [`../../../docs/Lily_8leg_Robot_Command_Reference.md`](../../../docs/Lily_8leg_Robot_Command_Reference.md)
- [`summary.json`](summary.json)
- [`manifest.json`](manifest.json)
