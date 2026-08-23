# Lily MCU Config Editor

更新日: 2026-08-23

Lily各関節MCUのConfigパラメータを、CAN経由で**確認・変更・保存**するPython 2 GUIツールである。

CAN接続、Config CAN protocol、raw CAN例の正本は [`../../docs/CAN_MCU_CONFIG_GUIDE.md`](../../docs/CAN_MCU_CONFIG_GUIDE.md) を参照する。

---

## 1. 配置

```text
lily_motion/
└─ tools/
   └─ mcu_config/
      ├─ lily_mcu_config_editor.py
      └─ README.md
```

---

## 2. 実行環境

- Jetson / Linux
- Python 2.7
- Tkinter / ttk
- can-utils (`candump`, `cansend`)
- CAN interface: `can0`
- bitrate: 500 kbit/s

PythonからSocketCANを直接操作せず、`candump` / `cansend` を利用する。

---

## 3. CAN接続

現行のCAN初期設定:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

確認:

```bash
ip -details link show can0
candump can0
```

---

## 4. 起動

Repository rootからAxis 11だけ表示:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 11
```

24軸を一覧対象:

```bash
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 0-23
```

指定例:

```text
--axes 11
--axes 10-11
--axes 0,1,2,11
--axes 0-23
```

---

## 5. GUIの通信方針

本GUIはCAN負荷を抑えるため、自動周期READを行わない。

```text
更新ボタン
→ 必要なREADを実行

1 parameter WRITE
→ MCU Echo
→ 同じparameterだけREAD back
```

全parameterをWRITEのたびに読み直さない。

---

## 6. 実験前に値を確認するだけの場合

急ぎのmotion実験では、まずこの使い方を推奨する。

```text
1. can0をUP
2. GUI起動
3. 対象axisを選択
4. 更新してREAD
5. 保存値を確認
6. WRITE / SAVEしない
7. GUIを閉じる、またはREAD用途のままにする
8. runtime実験へ進む
```

Axis 11で基準へ復元確認済み:

```text
Kp         = 500
gear_ratio = 30.8
```

---

## 7. SoftwareConfig変更手順

WRITE / SAVEは `aliment_standby` で行う。

```text
1. 対象axisを選択
2. SoftwareConfig parameterを選択
3. 新しい値を入力
4. WRITE
5. MCU Echo = OKを確認
6. 自動で行われる同一parameter READ backを確認
7. 一時変更だけならここで終了
8. 電源再投入後も残したい場合だけ SoftwareConfig SAVE
```

SoftwareConfig WRITEはRAMへ即時反映される。

```text
WRITEのみ
→ 現在の起動中だけ有効
→ power cycleで保存済み値へ戻る

WRITE + SAVE
→ Flashへ保存
→ power cycle後も保持
```

---

## 8. HardwareConfig変更手順

HardwareConfigは、SoftwareConfigと扱いが異なる。

```text
1. aliment_standbyを確認
2. 対象axisを選択
3. HardwareConfig parameterを選択
4. 新しい値を入力
5. WRITE
6. MCU Echoを確認
7. 同一parameter READ backを確認
8. HardwareConfig SAVE
9. SAVE成功を確認
10. MCU電源を再投入
11. GUIを再起動 / 再READ
12. 保存値を確認
```

HardwareConfig SAVE後は再起動を必須運用とする。

---

## 9. READ / WRITE / SAVE state rule

```text
READ  : 全stateで可
WRITE : aliment_standbyのみ
SAVE  : aliment_standbyのみ
```

RUN中にWRITE / SAVEしない。

---

## 10. Parameter一覧

### HardwareConfig

| ID | parameter | GUI input / wire |
|---:|---|---|
| `0x01` | gear_ratio | float32 |
| `0x02` | motor_direction | int32 |
| `0x03` | joint_min_rad | GUIはdeg入力 / CANはfloat32 rad |
| `0x04` | joint_max_rad | GUIはdeg入力 / CANはfloat32 rad |
| `0x05` | can_termination_enable | uint32 |

### SoftwareConfig

| ID | parameter | GUI input / wire |
|---:|---|---|
| `0x01` | Kp | int32 |
| `0x02` | Ki | int32 |
| `0x03` | Kd | int32 |
| `0x04` | position_jump_limit_rad | GUIはdeg入力 / CANはfloat32 rad |
| `0x05` | position_error_limit_rad | GUIはdeg入力 / CANはfloat32 rad |
| `0x06` | interpolation_time_ms | uint32 ms |
| `0x07` | torque_ramp_target | int32 |
| `0x08` | torque_ramp_duration_ms | uint32 ms |

physical joint limitは [`../../docs/HARDWARE_LIMITS.md`](../../docs/HARDWARE_LIMITS.md) を正本とする。GUIに保存されているjoint limit値だけで機械安全を判断しない。

---

## 11. Config CAN概要

```text
Request  = 0x080 | axis
Response = 0x180 | axis
```

Axis 11:

```text
Request  = 0x08B
Response = 0x18B
```

```text
Byte 0   Command
Byte 1   Config Type
Byte 2   Parameter ID
Byte 3   Result
Byte 4-7 Value, little endian 32 bit
```

詳細とraw `cansend` 例は [`../../docs/CAN_MCU_CONFIG_GUIDE.md`](../../docs/CAN_MCU_CONFIG_GUIDE.md) に集約する。

---

## 12. 操作上の注意

- SAVE中に電源を切らない。
- HardwareConfig SAVE後はpower cycleする。
- 極端なPID値や機械的に危険な値を入力しない。
- 一度に大量のaxis / parameterを高頻度に更新しない。
- 通常の調整はGUIを使い、raw `cansend` は診断用とする。
- `0x0EE` や予期しないmotionが出た場合は実験を継続しない。

---

## 13. 現在確認済み

Axis 11単軸で以下を確認済み:

- READ
- SoftwareConfig WRITE / Echo / same-parameter READ back
- SoftwareConfig SAVE / power cycle persistence
- HardwareConfig WRITE / Echo / same-parameter READ back
- HardwareConfig SAVE / power cycle persistence
- HW / SW独立保存
- 未接続axisとの混在表示
- Kp変更が実際の制御挙動へ反映されること

上記GUI確認はPC環境で実施済み。Jetsonでの最終低負荷回帰と24軸同時接続は別途確認対象。
