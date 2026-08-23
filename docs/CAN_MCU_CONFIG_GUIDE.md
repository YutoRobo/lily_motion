# Lily CAN / MCU Config Guide

更新日: 2026-08-23  
対象: `master`

この文書は、**実機CANの接続、CANコマンド体系、MCU Configパラメータの確認・変更・保存手順の正本**である。

役割分担:

- 実機試験の安全な順序: [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- そのままコピーするコマンド: [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md)
- UI command / CAN IDの意味: [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md)
- Config GUI固有の使い方: [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md)
- physical joint limit: [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md)

---

## 0. Path convention

このリポジトリのcurrent文書では、**個人PC固有の絶対パスを使用しない**。

`tools/...`、`docs/...`、`data/...` はすべて `lily_motion/` のrepository root基準で記載する。

実行前に各PCでclone済みの `lily_motion/` repository rootへ移動する。repository内の任意のsubdirectoryにいる場合は、次でrootへ戻せる。

```bash
cd "$(git rev-parse --show-toplevel)"
```

repository外にいる場合は、各自のclone先へ移動してから実行する。

---

## 1. CAN接続の正本

Jetson / Linuxで実機CANを使用するときは、`can0` を次の2コマンドで設定する。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

この2行を、現行のCAN初期設定コマンドとして扱う。

状態確認:

```bash
ip -details link show can0
```

CAN監視:

```bash
candump -L can0
```

Config GUIでは内部的に `candump` / `cansend` を使用するため、`can-utils` が必要である。

---

## 2. CAN操作は3層に分ける

```text
A. Linux SocketCAN設定
   sudo ip link ...

B. Lily runtime操作
   ROS UI command
   → StateMachine
   → CAN

C. MCU Config操作
   lily_mcu_config_editor.py
   または診断用raw cansend
   → Config CAN protocol
```

通常のLily motion実験ではBを使用する。  
MCUのPID、減速比、制限値などを確認・変更するときだけCを使用する。

---

## 3. Runtime CANの起動

以下はrepository rootから実行する。

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

UI:

```bash
python2 tools/can_interface/initUI/ui.py
```

実機時はGazebo MCU nodeを同時に `/cmdForJetson` へ接続しない。

---

## 4. Runtime CAN ID

| purpose | CAN ID |
|---|---:|
| standby heartbeat RX | `0x0FF` |
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN TX | `0x600 + axis` |
| error RX | `0x0EE` |

POSITION payload:

```text
Byte 0-3 : 0
Byte 4-7 : little-endian float32 [rad]
```

通常はraw `cansend` でruntime操作せず、ROS UI / StateMachineを使用する。

---

## 5. MCU Config GUIの起動

repository rootから実行する。

Axis 11だけ確認する場合:

```bash
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 11
```

24軸を一覧対象にする場合:

```bash
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 0-23
```

GUIは自動周期READを行わない。更新ボタンを押したときだけREADし、WRITE後は変更した1パラメータだけREAD backする。

---

## 6. Configの基本操作

### 6.1 READ

READはMCU仕様上、全stateで使用可能である。

実験前の確認だけなら、基本的にREADだけ使用する。

### 6.2 SoftwareConfig変更

```text
aliment_standby
→ GUIで対象axis / parameterを選択
→ 値を入力
→ WRITE
→ MCU Echo確認
→ 同一parameterのREAD back確認
→ 永続化する場合だけ SoftwareConfig SAVE
```

SoftwareConfigのWRITEはRAMへ即時反映される。SAVEしなければ、電源再投入後は保存済み値へ戻る。

### 6.3 HardwareConfig変更

```text
aliment_standby
→ GUIで対象axis / parameterを選択
→ 値を入力
→ WRITE
→ MCU Echo確認
→ 同一parameterのREAD back確認
→ HardwareConfig SAVE
→ SAVE成功を確認
→ MCU電源再投入
→ 再度READして確認
```

HardwareConfig SAVE後は、新しいHardwareConfigを使用するために再起動する。

### 6.4 WRITE / SAVEのstate制約

```text
READ        : 全stateで可
WRITE       : aliment_standbyのみ
SAVE        : aliment_standbyのみ
```

---

## 7. Config parameter ID

### HardwareConfig (`type = 0x01`)

| ID | parameter | wire type |
|---:|---|---|
| `0x01` | gear_ratio | float32 |
| `0x02` | motor_direction | int32 |
| `0x03` | joint_min_rad | float32 |
| `0x04` | joint_max_rad | float32 |
| `0x05` | can_termination_enable | uint32 |

### SoftwareConfig (`type = 0x02`)

| ID | parameter | wire type |
|---:|---|---|
| `0x01` | Kp | int32 |
| `0x02` | Ki | int32 |
| `0x03` | Kd | int32 |
| `0x04` | position_jump_limit_rad | float32 |
| `0x05` | position_error_limit_rad | float32 |
| `0x06` | interpolation_time_ms | uint32 |
| `0x07` | torque_ramp_target | int32 |
| `0x08` | torque_ramp_duration_ms | uint32 |

Axis 11で現在の基準として実機復元確認済みの値:

```text
Kp         = 500
gear_ratio = 30.8
```

機械的に安全なjoint limitの正本は `HARDWARE_LIMITS.md` であり、Config画面の保存値だけをphysical safety gateとして扱わない。

---

## 8. Config CAN protocol

各axisのConfig ID:

```text
Request  = 0x080 | axis
Response = 0x180 | axis
```

Axis 11 (`0x0B`) の場合:

```text
Request  = 0x08B
Response = 0x18B
```

8 byte payload:

```text
Byte 0   : Command
Byte 1   : Config Type
Byte 2   : Parameter ID
Byte 3   : Result / reserved in request
Byte 4-7 : Value, little endian 32 bit
```

Command:

```text
0x01 READ
0x02 WRITE
0x03 SAVE
```

Result:

```text
0x00 OK
0x01 INVALID_PARAM
0x02 INVALID_VALUE
0x03 INVALID_STATE
0x04 SAVE_ERROR
0x05 SAVE_NOT_IMPLEMENTED
0x06 STORAGE_ERROR
```

---

## 9. Axis 11 raw CAN例

通常運用ではGUIを使用する。以下は診断・プロトコル確認用。

Kp READ:

```bash
cansend can0 08B#0102010000000000
```

Kp = 500 WRITE:

```bash
cansend can0 08B#02020100F4010000
```

SoftwareConfig SAVE:

```bash
cansend can0 08B#0302000000000000
```

Gear ratio READ:

```bash
cansend can0 08B#0101010000000000
```

Gear ratio = 30.8 WRITE:

```bash
cansend can0 08B#020101006666F641
```

HardwareConfig SAVE:

```bash
cansend can0 08B#0301000000000000
```

応答はAxis 11では `0x18B` を確認する。

---

## 10. 実験前の推奨確認

急ぎの実験では、Configを書き換えず次だけ確認する。

```text
1. CANを500 kbit/sでUP
2. repository rootからConfig GUI起動
3. 対象axisをREAD
4. 意図した保存値であることを確認
5. GUIを閉じる、またはREAD用途のままにする
6. StateMachine / UIを起動
7. ALIGN → HOME → RUN → motion trial
```

特にAxis 11では、基準に戻してある `Kp=500`, `gear_ratio=30.8` を確認してから試験へ進む。

---

## 11. 運用上の注意

- SAVE中に電源を切らない。
- HardwareConfig SAVE後は再起動する。
- 極端なPID値、gear ratio、joint limitを入力しない。
- RUN中にConfig WRITE / SAVEしない。
- 通常運用では必要なparameterだけREAD / WRITEし、全軸を高頻度に更新しない。
- 異常CAN frame、`0x0EE`、予期しないmotionがあれば実験を継続しない。

---

## 12. 現在の確認状況

Axis 11単軸で以下を確認済み:

- Config READ
- SoftwareConfig WRITE / Echo / READ back
- SoftwareConfig SAVE / power cycle persistence
- HardwareConfig WRITE / Echo / READ back
- HardwareConfig SAVE / power cycle persistence
- HW / SW独立保存
- GUI上で未接続axisと接続axisが混在しても動作
- Kp変更が実際の制御挙動へ反映されること

Config GUIの上記確認はPC環境で実施済み。Jetson実機環境での最終低負荷回帰と24軸同時接続は別途確認対象とする。
