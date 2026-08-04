# Hardware Pretest Status

更新日: 2026-08-04

## 1. Status Summary

現行`master`では、回転歩容指令からROS、StateMachine、CANフレーム展開までのソフトウェア経路を統合済みである。

```text
歩容JSONL／単軸試験
→ /cmdForJetson
→ tools/can_interface StateMachine
→ Use=True軸
→ CAN RUN／POSITION
```

現在の判定:

| 項目 | 状態 |
|---|---|
| Python単体・回帰試験 | PASS済み |
| CAN FakeBus試験 | PASS |
| vcan単軸axis10 | PASS |
| vcan複数軸axis10,11,12 | PASS |
| mock end-to-end | PASS |
| Gazebo full roll | PASS |
| 実機axis10 `+0.002 rad` | 暫定PASS |
| 実機axis10負方向 | 未確認 |
| 実機axis10 `+/-0.005 rad` | 未確認 |
| 実機1脚3軸 | 未確認 |
| 実機air-entry以降 | 未確認 |
| 実機full roll | 未確認 |

この文書でいう「暫定PASS」は、目視上問題がなかったことを示す。長時間動作、負荷、温度、再現性、複数軸同期まで保証するものではない。

## 2. Maintained Execution Target

現行の実行対象:

- `tools/can_interface/statemachine/main.py`
- `tools/can_interface/initUI/ui.py`
- `tools/publish_cmdforjetson_single_axis_test.py`
- `tools/publish_cmdforjetson_jsonl.py`

実行してはならない旧経路:

- `external/can_interface/260102_usb_can_fast_alignment/`
- 削除済みの`/can/axis_command`位置指令経路

`external/can_interface`は移設前スナップショット／参照用であり、現行実機経路ではない。

## 3. Production Position Command Path

本番位置指令入力は`/cmdForJetson`だけである。

```text
Topic: /cmdForJetson
Message: sensor_msgs/JointState
position length: exactly 24
unit: rad
```

StateMachineは次を実施する。

- RUN前の位置指令をCANへ送らない
- `Use=True`軸だけを安全判定する
- `Use=True`軸だけへ`0x400 + axis`を送る
- 関節制限、非数、NaN、Inf、セッション、エラー状態を確認する
- STOP後の位置指令をCANへ送らない

単軸安全マスクpublisherでは、対象軸だけ有限値、対象外23軸をNaNとする。意図せず別軸が`Use=True`の場合、そのNaNによりフレーム全体が送信前に拒否される。

## 4. Current Reference Candidate

最新の凍結済みpre-hardware候補:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

主な状態:

- command count: `2233`
- coxa: `0.075 m`
- thigh: `0.300 m`
- tibia: `0.300 m`
- maximum second-joint angle: `94.8 deg`
- second-joint violation count over `95 deg`: `0`
- Gazebo full roll: `PASS`
- strict command-log dry run: `PASS`
- hardware full roll: not tested

候補固有の段階順序:

- [`../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)

旧`v3_0_42c_candidate_02_softlimit_94p8`は比較・履歴上の重要候補だが、現在の最初のpre-hardware候補ではない。

## 5. Staged Logs

### Air-entry and hold only

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/air_entry_and_hold_only_commands.jsonl
```

- `135` frames
- air-entry and hold only
- no roll-body frames

### Split roll logs

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/
  roll_0_50_commands.jsonl
  roll_50_100_commands.jsonl
  roll_100_300_commands.jsonl
  roll_300_end_commands.jsonl
```

### Final full sequence

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/combined_with_hold_commands.jsonl
```

- `2368` frames
- air-entry + hold + complete roll body
- final confirmation only

## 6. Verified CAN Results

### 6.1 CAN unit tests

Verified result:

```text
focused unified-path tests: 10/10 PASS
all CAN tests: 81/81 PASS
Python 2.7 syntax: PASS
Python 3 syntax: PASS
git diff --check: PASS
```

Representative tests:

```text
tests/test_can_cmdforjetson_unified_path.py
tests/test_can_diagnostic_run.py
tests/test_can_emulator_integration.py
tests/test_can_legacy_alignment_retry.py
tests/test_can_multi_actuator_emulator.py
```

### 6.2 vcan axis10

```text
RUN 0x60A: 1 frame
POSITION 0x40A: 11 frames
unexpected RUN/POS IDs: none
command: 0.000 → 0.010 → 0.000 rad
result: PASS
```

### 6.3 vcan axis10,11,12

```text
RUN: 0x60A, 0x60B, 0x60C
POSITION: 0x40A, 0x40B, 0x40C
one 24-element JointState was fanned out to three selected axes
result: PASS
```

This proves software fan-out. It does not prove simultaneous physical response or 24-unit bus behavior.

## 7. Verified Hardware Result

Real actuator axis10:

```text
direction: plus
amplitude: 0.002 rad
path: /cmdForJetson → StateMachine → 0x40A
visual result: no obvious abnormality
status: provisional PASS
```

The following remain unverified:

- negative `0.002 rad`
- positive and negative `0.005 rad`
- repeatability
- one-leg three-axis operation
- multiple real actuator operation
- current, sound, vibration, temperature under sustained motion

## 8. CAN Protocol Summary

- connection/standby heartbeat RX: `0x0FF`
- ALIGN request TX: `0x000 + axis`
- ALIGN result RX: `0x100 + axis`
- HOME jog TX: `0x200 + axis`
- SET HOME TX: `0x300 + axis`
- position command TX: `0x400 + axis`
- RUN start TX: `0x600 + axis`
- position payload: `[0,0,0,0] + little-endian float32(rad)`

`0x0FF` is a standby discovery heartbeat. It is not required to continue after successful ALIGN.

## 9. Use=True Specification

- `Use=True` is the active-axis selection.
- ALIGN, HOME, SET HOME, RUN, and POSITION are restricted by Use selection and session gates.
- RUN is rejected when no axis is active.
- RUN is accepted only when all active axes are aligned and homed in the current session.
- `Use=False` axes receive no RUN or POSITION frame.
- disconnected inactive axes do not block RUN.

## 10. Mock End-to-End Evidence

Existing mock evidence includes:

```text
testdata/end_to_end_initial_pose_to_roll_can_check/summary.json
testdata/end_to_end_initial_pose_to_roll_can_check/command_sequence_check.json
testdata/end_to_end_initial_pose_to_roll_can_check/phase_boundary_check.json
testdata/end_to_end_initial_pose_to_roll_can_check/use_all_24_can_check.json
testdata/end_to_end_initial_pose_to_roll_can_check/use_4_axis_can_check.json
testdata/end_to_end_initial_pose_to_roll_can_check/run_stop_gate_check.json
testdata/end_to_end_initial_pose_to_roll_can_check/hardware_limit_report.json
```

These results were created for the earlier candidate02 sequence and remain evidence for the `/cmdForJetson` and StateMachine conversion path. They are not a substitute for hardware validation of the current v3.0.44 candidate.

## 11. Required Hardware Test Order

```text
axis10 negative 0.002 rad
→ axis10 positive/negative 0.005 rad
→ one complete leg, three axes
→ air-entry and hold only
→ touchdown confirmation
→ roll 0–50
→ roll 50–100
→ roll 100–300
→ roll 300–end
→ combined full sequence
```

Full sequence must remain last.

## 12. Hardware-Dependent Items Still Open

- real CAN behavior with several and 24 installed units
- ACK and reset behavior of actual MCU firmware
- HOME direction for every axis
- SET HOME posture and repeatability
- physical joint sign mapping
- multi-axis timing and bus load
- air-entry clearance
- touchdown contact and support stability
- staged roll contact, current, sound, vibration, heat, and interference
- full roll motion
- Jetson Orin CPU load, temperature, and scheduling jitter during sustained operation

## 13. Safety Decision

Software pretest has progressed beyond mock-only status, but the system is not approved for full roll. The next approved action is still a small-angle single-axis test, followed by one-leg testing.

Detailed operation:

- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md)
