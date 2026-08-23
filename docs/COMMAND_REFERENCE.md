# Lily 8脚ロボット コマンドリファレンス

更新日: 2026-08-23  
対象: 現行 `master`

この文書は、**Lily runtime command、CAN ID、MCU Config protocolの意味を確認するためのリファレンス**である。

実際にコマンドをコピーする場合は [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md)、CAN接続とConfig操作手順は [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) を使用する。

---

## 1. CAN関連の3層

```text
Linux SocketCAN設定
  ↓
ROS UI / StateMachine runtime command
  ↓
MCU runtime CAN protocol

または

Linux SocketCAN設定
  ↓
MCU Config GUI / raw cansend
  ↓
MCU Config CAN protocol
```

CAN初期設定の正本:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

---

## 2. Runtime UI command

Topic:

```text
/ui/leg_command
```

Message:

```text
std_msgs/String
```

| command | 意味 |
|---|---|
| `use:<axis>:1` | 対象axisをruntime操作対象にする |
| `use:<axis>:0` | 対象axisをruntime操作対象から外す |
| `align` | Use=Trueの対象へALIGN |
| `align:<axis>` | 指定axisへALIGN |
| `home_step:<rad>` | HOME jog stepを設定 |
| `home_move:<axis>:1` | 指定axisを+方向へHOME jog |
| `home_move:<axis>:-1` | 指定axisを-方向へHOME jog |
| `set_home:<axis>` | 指定axisのlogical HOMEを設定 |
| `run` | RUNへ遷移 |
| `stop` | STOP |

`global home` / `global set_home` はcurrent commandとして使用しない。

---

## 3. Runtime CAN ID

| purpose | direction | CAN ID |
|---|---|---:|
| standby heartbeat | MCU → host | `0x0FF` |
| ALIGN request | host → MCU | `0x000 + axis` |
| ALIGN result | MCU → host | `0x100 + axis` |
| HOME jog | host → MCU | `0x200 + axis` |
| SET HOME | host → MCU | `0x300 + axis` |
| POSITION | host → MCU | `0x400 + axis` |
| RUN | host → MCU | `0x600 + axis` |
| error | MCU → host | `0x0EE` |

POSITION payload:

```text
Byte 0-3 : 0x00
Byte 4-7 : float32 little endian [rad]
```

Runtime操作は原則としてraw CAN frameを直接送らず、StateMachineを通す。

---

## 4. `/cmdForJetson`

Production position input:

```text
Topic   : /cmdForJetson
Message : sensor_msgs/JointState
position length : exactly 24
unit    : rad
```

Hardware:

```text
/cmdForJetson
→ tools/can_interface/statemachine/
→ CAN
→ real MCU
```

Gazebo:

```text
/cmdForJetson
→ tools/gazebo/mcu_position_interpolator_node.py
→ Gazebo
```

HardwareとGazeboのconsumerを同時に接続しない。

---

## 5. MCU Config CAN ID

Config protocolはruntime CANとは別のIDを使う。

```text
Request  = 0x080 | axis
Response = 0x180 | axis
```

Axis 11:

```text
Request  = 0x08B
Response = 0x18B
```

---

## 6. Config payload

8 byte:

```text
Byte 0   : Command
Byte 1   : Config Type
Byte 2   : Parameter ID
Byte 3   : Result / requestでは0
Byte 4-7 : Value, little endian 32 bit
```

Command:

| value | command |
|---:|---|
| `0x01` | READ |
| `0x02` | WRITE |
| `0x03` | SAVE |

Config Type:

| value | type |
|---:|---|
| `0x01` | HardwareConfig |
| `0x02` | SoftwareConfig |

Result:

| value | result |
|---:|---|
| `0x00` | OK |
| `0x01` | INVALID_PARAM |
| `0x02` | INVALID_VALUE |
| `0x03` | INVALID_STATE |
| `0x04` | SAVE_ERROR |
| `0x05` | SAVE_NOT_IMPLEMENTED |
| `0x06` | STORAGE_ERROR |

---

## 7. HardwareConfig parameter

| ID | parameter | type |
|---:|---|---|
| `0x01` | gear_ratio | float32 |
| `0x02` | motor_direction | int32 |
| `0x03` | joint_min_rad | float32 |
| `0x04` | joint_max_rad | float32 |
| `0x05` | can_termination_enable | uint32 |

HardwareConfigはWRITE後にSAVEし、SAVE成功後はMCUを再起動して使用する。

---

## 8. SoftwareConfig parameter

| ID | parameter | type |
|---:|---|---|
| `0x01` | Kp | int32 |
| `0x02` | Ki | int32 |
| `0x03` | Kd | int32 |
| `0x04` | position_jump_limit_rad | float32 |
| `0x05` | position_error_limit_rad | float32 |
| `0x06` | interpolation_time_ms | uint32 |
| `0x07` | torque_ramp_target | int32 |
| `0x08` | torque_ramp_duration_ms | uint32 |

SoftwareConfig WRITEはRAMへ即時反映される。SAVEしなければpower cycle後は保存済み値へ戻る。

---

## 9. Config state rule

```text
READ  : 全stateで可
WRITE : aliment_standbyのみ
SAVE  : aliment_standbyのみ
```

通常のmotion試験中にWRITE / SAVEしない。

---

## 10. Axis 11 raw Config例

Kp READ:

```bash
cansend can0 08B#0102010000000000
```

Kp=500 WRITE:

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

Gear ratio=30.8 WRITE:

```bash
cansend can0 08B#020101006666F641
```

HardwareConfig SAVE:

```bash
cansend can0 08B#0301000000000000
```

通常のパラメータ調整はGUIを優先する。

---

## 11. Publisher一覧

| 用途 | program |
|---|---|
| single axis | `tools/publish_cmdforjetson_single_axis_test.py` |
| one leg | `tools/publish_cmdforjetson_one_leg_test.py` |
| mapped axis diagnostic | `tools/publish_cmdforjetson_mapped_axis_replay.py` |
| frozen / staged JSONL | `tools/publish_cmdforjetson_jsonl.py` |

現在のstaged transport profile:

```text
resample-factor = 2
rate = 10 Hz
```

具体的な実行コマンドは `COPY_PASTE_COMMANDS.md` に集約する。

---

## 12. 関連文書

- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)
- [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md)
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)
- [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md)
- [`../tools/can_interface/README.md`](../tools/can_interface/README.md)
- [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md)
