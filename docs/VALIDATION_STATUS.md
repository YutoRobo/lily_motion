# Lily Validation Status

更新日: 2026-08-23  
対象: 現行 `master`

この文書は、**現在どこまで検証済みか**を示す正本である。

motion runtimeとMCU Configは別系統なので、検証状況を分けて示す。

---

## 1. Summary

```text
Motion / staged roll
  software / Gazebo validation:          PASS
  semantic quarter frozen:               YES
  semantic quarter Gazebo:               PASS
  full staged real hardware:             NOT TESTED

MCU Config / parameter editor
  Axis 11 READ/WRITE/SAVE:               PASS
  SW/HW persistence after power cycle:   PASS
  GUI PC validation:                     PASS
  Jetson final low-load regression:      NOT YET FINAL
  24-axis simultaneous Config test:      NOT TESTED
```

したがって:

```text
staged hardware validationを開始してよい: YES
full rollから開始してよい:             NO
MCU Configの通常単軸運用:               YES
```

---

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

Current baselineの正本:

- [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md)

Immutable pre-hardware evidence:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

---

## 3. Canonical runtime

```text
staged JSONL
→ publish_cmdforjetson_jsonl.py
→ shared transport resampling
→ /cmdForJetson
   ├→ REAL: StateMachine → CAN → real MCU
   └→ GAZEBO: MCU interpolator node → Gazebo
```

Current transport profile:

```text
resample-factor = 2
transport rate  = 10 Hz
```

Gazebo MCU profile:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

詳細:

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

---

## 4. Motion software / CAN regression

2026-08-12 pre-hardware freeze前のsoftware evidence:

```text
test_publish_cmdforjetson_jsonl_resampling.py    5 PASS
test_command_timing.py                           7 PASS
test_shared_command_stream.py                    3 PASS
test_gazebo_mcu_interpolator_online.py           6 PASS
```

CAN software evidence:

```text
CAN focused unified-path tests    PASS
CAN full test set                 81/81 PASS
vcan axis10                       PASS
vcan axes10,11,12 fan-out         PASS
mock end-to-end                   PASS
```

これらは実機mechanicsや24軸実CAN bus loadを保証しない。

---

## 5. Gazebo validation

Risk-split:

```text
HOME → air-entry        PASS
roll 0–50               PASS
roll 50–100             PASS
roll 100–300            PASS
roll 300–end            PASS
```

Semantic quarter:

```text
roll_to_1of4    PASS
roll_to_2of4    PASS
roll_to_3of4    PASS
roll_to_4of4    PASS
```

各quarter fileはrolling-startからの累積prefix。

---

## 6. Existing real-hardware motion evidence

既存確認:

```text
real axis10
+0.002 rad
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

実機進行順は [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を正本とする。

---

## 7. MCU Config firmware validation

Axis 11単軸で確認済み:

```text
READ                                    PASS
SoftwareConfig WRITE                    PASS
SoftwareConfig Echo                     PASS
same-parameter READ back                PASS
SoftwareConfig SAVE                     PASS
SW persistence after power cycle        PASS
HardwareConfig WRITE                    PASS
HardwareConfig Echo                     PASS
HardwareConfig SAVE                     PASS
HW persistence after power cycle        PASS
HW/SW independent persistence           PASS
```

基準復元確認済み:

```text
Kp         = 500
gear_ratio = 30.8
```

HardwareConfig SAVE後の再起動要求動作も確認済み。

---

## 8. MCU Config GUI validation

PC環境で確認済み:

```text
Python 2 GUI起動                        PASS
can0接続                                PASS
Axis 11 parameter READ                  PASS
missing axis + connected axis混在       PASS
1 parameter WRITE                       PASS
MCU Echo                                PASS
same-parameter-only READ back           PASS
SoftwareConfig SAVE                     PASS
HardwareConfig SAVE                     PASS
power cycle後の表示復元                 PASS
```

Kp変更が実際の制御挙動へ反映されることも確認済み。

ただし、極端なKpで発振することはcontrol-safe rangeの問題であり、Config通信正常性とは分けて扱う。

---

## 9. Config remaining checks

```text
Jetson actual final low-load regression     remaining
24-axis simultaneous connection             remaining
Flash corruption injection                  deferred
SAVE中power-loss                            deferred / operation controlled
linker script last 4KB reservation          remaining integration hardening
```

急ぎ実験では、通常値・正常CAN・SAVE中power-off禁止の運用で現行baselineを使用する。

---

## 10. CAN / Config documentation

CAN setup / Config操作:

- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)

Config GUI:

- [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md)

CAN StateMachine:

- [`../tools/can_interface/README.md`](../tools/can_interface/README.md)

---

## 11. Main remaining hardware risks

- physical joint sign mapping
- HOME direction / repeatability
- actual MCU interpolation under full mechanism
- multi-axis CAN timing / serialization
- load tracking
- backlash / compliance
- current / sound / vibration / temperature
- floor contact / slip
- cable interference
- Jetson scheduling jitter
- full mechanism dynamics

---

## 12. Safety boundary

Software / Config validationがPASSしても、物理非常停止、fixture、suspended start、段階試験を省略しない。

詳細:

- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
