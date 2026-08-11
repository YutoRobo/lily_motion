# Hardware Pretest Status

更新日: 2026-08-12  
対象: staged real-hardware validation直前

## 1. Summary

現行softwareは、回転候補から `/cmdForJetson` までを実機/Gazebo共通化し、Gazebo側だけにMCU-equivalent interpolationを置く構成へ整理済みである。

現在の判定:

```text
software ready to START staged hardware validation: YES
approved to start directly with full roll:           NO
```

## 2. Current candidate

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

```text
command count:              2233
coxa:                       0.075 m
thigh:                      0.300 m
tibia:                      0.300 m
second-joint max:           94.8 deg
violations over 95 deg:     0
full-roll Gazebo review:    PASS
hardware full roll:         NOT TESTED
```

## 3. Current software baseline

```text
commit:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

branch:
baseline/pre-hardware-gazebo-pass-20260812
```

Record:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

## 4. Canonical runtime

```text
staged JSONL
→ publish_cmdforjetson_jsonl.py
→ shared transport resampling
→ /cmdForJetson
   ├→ REAL: StateMachine → CAN → real MCU
   └→ GAZEBO: MCU interpolator node → Gazebo
```

詳細:

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

## 5. Current transport comparison profile

```text
resample-factor = 2
transport rate  = 10 Hz
```

Gazebo MCU profile:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

このprofileはGazeboで検証済みだが、実MCUとの組合せは今後の実機stageで確認する。

## 6. Software / regression status

2026-08-12 freeze前に確認:

```text
test_publish_cmdforjetson_jsonl_resampling.py    5 PASS
test_command_timing.py                           7 PASS
test_shared_command_stream.py                    3 PASS
test_gazebo_mcu_interpolator_online.py           6 PASS
```

既存CAN software evidence:

```text
CAN focused unified-path tests    PASS
CAN full test set                 81/81 PASS
vcan axis10                       PASS
vcan axes10,11,12 fan-out         PASS
mock end-to-end                   PASS
```

これらは実機mechanicsや実CAN bus loadを保証しない。

## 7. Canonical-path Gazebo validation

同じpublisher、同じ `/cmdForJetson`、同じtransport profileを使用し、Gazebo MCU nodeを起動した状態でsplit stageを連続実行した。

```text
HOME → air-entry        PASS
roll 0–50               PASS
roll 50–100             PASS
roll 100–300            PASS
roll 300–end            PASS
```

追加確認:

```text
visible boundary jump         none observed
final-pose hold               PASS
MCU node kept alive across stages
```

これにより「Gazebo用に別軌道を作ったから動いた」という不確定要因を除去した。

## 8. Hardware status

既存確認:

```text
real axis10
direction: plus
amplitude: 0.002 rad
visual result: no obvious abnormality
status: provisional PASS
```

未確認:

```text
axis10 negative 0.002 rad
axis10 ±0.005 rad
one-leg three-axis
multiple real actuators
current factor=2 / 10 Hz JSONL transport with real MCU
air-entry
touchdown
split roll
full roll
```

## 9. Staged inputs

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/
  air_entry_and_hold_only_commands.jsonl
  roll_0_50_commands.jsonl
  roll_50_100_commands.jsonl
  roll_100_300_commands.jsonl
  roll_300_end_commands.jsonl
  combined_with_hold_commands.jsonl
```

air-entry source:

```text
135 source frames
```

factor=2で:

```text
269 transport frames
```

air-entry transport SHA256:

```text
e1c00e23811f841e86ca4ff3fdc9a42c380e6537f6cf9623f97334a020f5a0fa
```

## 10. Required hardware progression

```text
single-axis positive/negative small-angle
→ one-leg individual axes
→ one-leg coordinated
→ suspended air-entry
→ controlled touchdown
→ roll 0–50
→ roll 50–100
→ roll 100–300
→ roll 300–end
→ final combined sequence
```

stageを飛ばさない。

## 11. Main remaining hardware risks

- physical joint sign mapping
- HOME direction / repeatability
- actual MCU interpolation behavior
- CAN timing and multi-axis serialization
- load tracking
- backlash / compliance
- current / sound / vibration / temperature
- floor contact and slip
- cable interference
- Jetson scheduling jitter
- full mechanism dynamics

## 12. Safety boundary

Software pretestがPASSしても、物理非常停止、fixture、suspended start、段階試験を省略しない。

詳細:

- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
