# Lily Validation Status

更新日: 2026-08-12  
対象: staged real-hardware validation直前

この文書は、**現在どこまで検証済みか**を示す正本である。

## 1. Summary

現行softwareは、回転候補から `/cmdForJetson` までを実機/Gazebo共通化し、Gazebo側だけにMCU-equivalent interpolationを置く構成へ整理済みである。

現在の判定:

```text
software ready to START staged hardware validation: YES
approved to start directly with full roll:           NO
semantic quarter files frozen:                       YES
semantic quarter Gazebo validation:                  PASS
semantic quarter hardware validation:                NOT TESTED
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

Candidate SHA256:

```text
e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

## 3. Current software baseline

current baseline正本:

- [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md)

pre-hardware software baseline:

```text
commit:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

branch:
baseline/pre-hardware-gazebo-pass-20260812
```

Immutable record:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

後続のsemantic-quarter derived data固定では、このimmutable recordを変更しない。

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

2026-08-12 pre-hardware freeze前に確認:

```text
test_publish_cmdforjetson_jsonl_resampling.py    5 PASS
test_command_timing.py                           7 PASS
test_shared_command_stream.py                    3 PASS
test_gazebo_mcu_interpolator_online.py           6 PASS
```

semantic-quarter builderについても、contiguous `roll_index` block、frame count、cumulative-prefix一致、deterministic output、4/4 source一致を確認済み。

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

### 7.1 Existing risk-split stages

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

### 7.2 Semantic quarter stages

`commands.jsonl` の連続 `roll_index` blockを境界として、次のcumulative stageを固定した。

```text
roll_to_1of4_commands.jsonl     560 frames
roll_to_2of4_commands.jsonl    1120 frames
roll_to_3of4_commands.jsonl    1680 frames
roll_to_4of4_commands.jsonl    2233 frames
```

Data-freeze commit:

```text
2e42343dccf3b56066cdcc97e011dca328388a20
```

Gazebo結果:

```text
1/4    PASS
2/4    PASS
3/4    PASS
4/4    PASS
```

`roll_to_4of4_commands.jsonl` は元の `commands.jsonl` とbyte-for-byte同一。

各quarter fileはrolling-startからの**累積prefix**であり、quarter同士を順番に継ぎ足して実行するfileではない。

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
risk-split roll
semantic quarter roll
full roll
```

## 9. Frozen staged inputs

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/

  # HOME → rolling-start
  air_entry_and_hold_only_commands.jsonl

  # initial real-hardware risk progression
  roll_0_50_commands.jsonl
  roll_50_100_commands.jsonl
  roll_100_300_commands.jsonl
  roll_300_end_commands.jsonl

  # independent cumulative semantic selections
  roll_to_1of4_commands.jsonl
  roll_to_2of4_commands.jsonl
  roll_to_3of4_commands.jsonl
  roll_to_4of4_commands.jsonl
  quarter_stage_manifest.json

  # final combined validation
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

Semantic quarter SHA256:

```text
1/4  cf2f2592b6dd688a996b4bcc872509fa9ee3b85d8db53825ce2a01671a70dc58
2/4  3e54fdef3c3285b2d45f43b086081ce1dc659e7a87098981a5702561878e0bf0
3/4  2599ea79a90ae4746a10f6771589e50e0a5acf7d3a1e2e0f8e146b602cad3998
4/4  e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

## 10. Required first hardware progression

初回実機ではsemantic quarterが存在していても、risk-split stageを飛ばさない。

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

risk-split full pathがPASSした後は、必要に応じて次を独立trialとして選べる。

```text
HOME → air-entry → touchdown → roll_to_1of4
HOME → air-entry → touchdown → roll_to_2of4
HOME → air-entry → touchdown → roll_to_3of4
HOME → air-entry → touchdown → roll_to_4of4
```

`roll_to_1of4` 実行直後に `roll_to_2of4` を続けて実行しない。

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
