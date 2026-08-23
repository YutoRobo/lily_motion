# Lily MCU Parameter Reference

更新日: 2026-08-23  
対象: 現行MCU Config firmware / `master` host tools

この文書は、**MCU側のHardwareConfig / SoftwareConfig parameterが何を意味し、firmware内のどこへ効くかを確認するための正本**である。

CAN操作方法は [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)、GUI操作方法は [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md) を参照する。

---

## 1. 最重要: Jetson parameterとMCU parameterを混同しない

```text
Jetson
  --resample-factor
  --rate
      ↓
  /cmdForJetson
      ↓
  CAN target
      ↓
MCU
  gear_ratio
  motor_direction
  joint limits
  PID
  jump/error limits
  interpolation_time_ms
  torque ramp
```

`--resample-factor` はJetson側parameterであり、MCUには保存されない。

MCUの `interpolation_time_ms` は別parameterである。

Jetson側引数は [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) を参照する。

---

# 2. HardwareConfig

HardwareConfigは**機体 / 軸固有の設定**として扱う。

基本動作:

```text
WRITE
→ pending HardwareConfigを変更
→ 現在動作中のactive値には即反映しない

SAVE
→ Flashへ保存
→ reboot required

power cycle
→ 保存済みHardwareConfigをload
→ active値へ反映
```

HardwareConfig SAVE後は、必ずMCUを再起動してからALIGNする。

---

## 2.1 `gear_ratio`

Config ID:

```text
HW 0x01
```

型:

```text
float32
```

意味:

```text
motor_position
= joint_position
* gear_ratio
* motor_direction
```

つまり、CANで受けるjoint側 [rad] をmotor側positionへ変換する倍率。

条件:

```text
gear_ratio > 0
```

firmware default:

```text
LEG_NUMBER % 3 == 2 : 30.8
otherwise           : 42.0
```

Axis 11は:

```text
gear_ratio = 30.8
```

注意:

- 符号は `gear_ratio` に持たせず `motor_direction` に分離している。
- HardwareConfigなのでWRITEだけではactive動作へ切り替わらない。

---

## 2.2 `motor_direction`

Config ID:

```text
HW 0x02
```

型:

```text
int32
```

許容値:

```text
+1
-1
```

意味:

joint正方向とmotor正方向の対応を決める符号。

```text
motor_position
= joint_position * gear_ratio * motor_direction
```

firmware default:

```text
LEG_NUMBER % 3 == 2 : -1
LEG_NUMBER % 3 == 0 : -1
otherwise           : +1
```

軸方向を誤ると実機motion方向が反転するため、24軸展開前に個別確認する。

---

## 2.3 `joint_min_rad`

Config ID:

```text
HW 0x03
```

型:

```text
float32 [rad]
```

GUI表示 / 入力:

```text
deg
```

意味:

MCUが受け付けるjoint側position commandの下限。

firmware内部では `gear_ratio` と `motor_direction` を使ってmotor側範囲へ変換し、command / actual positionのrange checkに使用する。

current firmware default:

```text
-2*pi rad = -360 deg
```

重要:

このdefaultはConfig移行時に旧動作を維持するための値であり、Lily実機のphysical limit正本とは一致しない軸がある。

physical limit正本:

```text
base   ±360 deg
thigh   ±95 deg
tibia  ±150 deg
```

physical safety判定は [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) を優先する。

---

## 2.4 `joint_max_rad`

Config ID:

```text
HW 0x04
```

型:

```text
float32 [rad]
```

GUI表示 / 入力:

```text
deg
```

意味:

MCUが受け付けるjoint側position commandの上限。

条件:

```text
joint_min_rad < joint_max_rad
```

current firmware default:

```text
+2*pi rad = +360 deg
```

`joint_min_rad` と同様、physical limitは `HARDWARE_LIMITS.md` を正本とする。

---

## 2.5 `can_termination_enable`

Config ID:

```text
HW 0x05
```

型:

```text
uint32
```

許容値:

```text
0 = CAN termination OFF
1 = CAN termination ON
```

意味:

MCUの `CAN_TERM_Pin` GPIOを制御し、その軸基板のCAN終端設定を切り替える。

HardwareConfigなので、保存後の起動時にactiveへ反映する運用とする。

---

# 3. SoftwareConfig

SoftwareConfigは**制御・監視・補間に関する設定**。

基本動作:

```text
WRITE
→ RAMへ即時反映

SAVEしない
→ 現在のpower sessionだけ有効
→ power cycleで保存済み値へ戻る

SAVEする
→ Flashへ保存
→ power cycle後も保持
```

WRITE / SAVEは `aliment_standby` で行う。

---

## 3.1 `Kp`

Config ID:

```text
SW 0x01
```

型:

```text
int32 on Config wire/storage
```

firmware default:

```text
500
```

意味:

position controllerの比例gain。WRITE時に `PID_SetKP()` へ即反映する。

注意:

- GUIで書けることと、機械的に安全なgainであることは別。
- 大幅に増加するとposition loopが振動する可能性がある。
- 現行MCSDK API側は16-bit gainを受けるため、極端な値を使用しない。

---

## 3.2 `Ki`

Config ID:

```text
SW 0x02
```

型:

```text
int32 on Config wire/storage
```

firmware default:

```text
50
```

意味:

position controllerの積分gain。WRITE時に `PID_SetKI()` へ即反映する。

---

## 3.3 `Kd`

Config ID:

```text
SW 0x03
```

型:

```text
int32 on Config wire/storage
```

firmware default:

```text
1
```

意味:

position controllerの微分gain。WRITE時に `PID_SetKD()` へ即反映する。

---

## 3.4 `position_jump_limit_rad`

Config ID:

```text
SW 0x04
```

型:

```text
float32 [joint-side rad]
```

GUI:

```text
deg入力 / deg表示
```

firmware default:

```text
10 deg
= 10*pi/180 rad
```

意味:

**新しく受信したtargetと直前targetの差**が大きすぎないかを判定するlimit。

firmware内部ではmotor側limitへ:

```text
motor_jump_limit
= position_jump_limit_rad * gear_ratio
```

として変換し、`SetPositionCmd()` のtarget jump checkに使用する。

これはjoint physical limitとは別物。

---

## 3.5 `position_error_limit_rad`

Config ID:

```text
SW 0x05
```

型:

```text
float32 [joint-side rad]
```

GUI:

```text
deg入力 / deg表示
```

firmware default:

```text
4 deg
= 4*pi/180 rad
```

意味:

MCU内部で生成しているcurrent interpolated commandとactual motor positionとの差が大きすぎないかを監視するlimit。

motor側では:

```text
motor_error_limit
= position_error_limit_rad * gear_ratio
```

として使用する。

追従誤差がlimitを超えるとposition errorとしてError_Handler経路へ入る。

---

## 3.6 `interpolation_time_ms`

Config ID:

```text
SW 0x06
```

型:

```text
uint32 [ms]
```

firmware default:

```text
50 ms
```

意味:

**MCUが1つ前のtargetから新targetへ線形に遷移する時間**。

現行firmwareのlow-frequency main周期:

```text
MAIN_DURATION = 2 ms
```

firmwareは:

```text
max_splited_count
= interpolation_time_ms / MAIN_DURATION
```

を整数除算で計算する。

Defaultでは:

```text
50 / 2 = 25 splits
```

各周期で概念的に:

```text
q_cmd(k)
= q_prev
+ (q_target - q_prev) * k / max_splited_count
```

を生成する。

### `resample_factor` との違い

```text
Jetson resample_factor
= CANへ送るtarget点そのものを増やす

MCU interpolation_time_ms
= 受信済みの1 targetへ内部的に滑らかに追従する
```

例:

```text
Jetson:
  resample_factor = 2
  rate = 10 Hz
  → 100 msごとにtransport target

MCU:
  interpolation_time_ms = 50 ms
  → target受信後50 msかけて遷移
  → 次targetまで残り時間はtarget保持
```

したがって、この2つは同じ補間ではない。

validation:

```text
interpolation_time_ms >= MAIN_DURATION
```

現行では最低2 ms。

---

## 3.7 `torque_ramp_target`

Config ID:

```text
SW 0x07
```

型:

```text
int32 on Config wire/storage
```

firmware default:

```text
4000
```

意味:

`MC_ProgramTorqueRampMotor1(target, duration)` のtargetとして使用する。

単位はこのLily firmware内でSI torque [N m]へ変換しておらず、**ST Motor Control SDKの内部torque command unit**として扱う。

WRITE時にtorque rampを再programする。

注意:

現行MCSDK APIのtarget引数は16-bitであり、極端な値を入力しない。

---

## 3.8 `torque_ramp_duration_ms`

Config ID:

```text
SW 0x08
```

型:

```text
uint32 [ms] on Config wire/storage
```

firmware default:

```text
100 ms
```

意味:

`MC_ProgramTorqueRampMotor1()` のramp duration。

`torque_ramp_target` または `torque_ramp_duration_ms` をWRITEすると、targetとdurationの現在値を組み合わせてtorque rampを再programする。

注意:

現行MCSDK API側のduration引数は16-bitなので、極端に大きいdurationを使用しない。

---

# 4. Default値まとめ

firmware default:

| parameter | default |
|---|---:|
| gear_ratio | axis class依存: 30.8 or 42.0 |
| motor_direction | axis class依存: +1 or -1 |
| joint_min_rad | -360 deg |
| joint_max_rad | +360 deg |
| can_termination_enable | `APPENDIX_NUMBER` axisのみ1、それ以外0 |
| Kp | 500 |
| Ki | 50 |
| Kd | 1 |
| position_jump_limit_rad | 10 deg |
| position_error_limit_rad | 4 deg |
| interpolation_time_ms | 50 ms |
| torque_ramp_target | 4000 |
| torque_ramp_duration_ms | 100 ms |

Axis 11で現在の保存baselineとして確認済み:

```text
gear_ratio = 30.8
Kp         = 500
```

保存済みFlash値がある場合、firmware defaultよりFlash値が優先される。

---

# 5. Parameterを変更したときの反映タイミング

| parameter group | WRITE直後 | SAVE | reboot |
|---|---|---|---|
| HardwareConfig | pendingのみ | Flash保存 | **必要。起動後active** |
| Kp / Ki / Kd | 即時active | 永続化 | SAVE済みなら保持 |
| jump/error limit | 即時参照値変更 | 永続化 | SAVE済みなら保持 |
| interpolation_time_ms | 即時参照値変更 | 永続化 | SAVE済みなら保持 |
| torque ramp | 即時再program | 永続化 | SAVE済みなら保持 |

---

# 6. READ / WRITE / SAVE state rule

```text
READ  : 全stateで可
WRITE : aliment_standbyのみ
SAVE  : aliment_standbyのみ
```

通常のmotion実験中にparameterを変更しない。

---

# 7. 現行実装で特に覚えておく注意点

1. `joint_min/max` のfirmware default ±360 degはphysical safety limitの代替ではない。
2. `resample_factor` はJetson側、`interpolation_time_ms` はMCU側。
3. HardwareConfigはSAVE後power cycleが必要。
4. SoftwareConfig WRITEは即時効くため、PID変更は特に慎重に行う。
5. Config wire/storageはint32/uint32でも、Kp/Ki/Kd・torque rampのMCSDK APIはより狭い型を使うため極端な値を避ける。
6. torque targetの物理SI単位換算は現行firmware / host Config仕様では定義していない。

---

# 8. 関連文書

- [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) — Jetson側CLI引数
- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) — CAN / Config操作
- [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md) — GUI操作
- [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) — physical joint limit
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) — 実機試験順序
