# Lily CAN Interface

更新日: 2026-08-23

`tools/can_interface/` は現行Lily hardware CAN UI / StateMachineのmaintained execution targetである。

CAN接続とMCU Config操作の正本は [`../../docs/CAN_MCU_CONFIG_GUIDE.md`](../../docs/CAN_MCU_CONFIG_GUIDE.md) を参照する。

---

## 1. CAN setup

実機CANは次の2コマンドで設定する。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

確認:

```bash
ip -details link show can0
candump -L can0
```

---

## 2. Entry points

StateMachine:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

UI:

```bash
python2 tools/can_interface/initUI/ui.py
```

Position publishers:

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py --help
python2 tools/publish_cmdforjetson_one_leg_test.py --help
python2 tools/publish_cmdforjetson_mapped_axis_replay.py --help
python2 tools/publish_cmdforjetson_jsonl.py --help
```

---

## 3. Production position input

```text
Topic: /cmdForJetson
Message: sensor_msgs/JointState
position length: exactly 24
unit: rad
```

StateMachine owns:

- CAN ID
- payload encoding
- Use gate
- ALIGN/HOME/RUN session
- joint limit check
- error latch
- CAN send failure handling

```text
/cmdForJetson
→ StateMachine
→ Use=True axes only
→ 0x400 + axis
```

---

## 4. Hardware / Gazebo boundary

Hardware:

```text
/cmdForJetson → StateMachine → CAN → real MCU
```

Gazebo:

```text
/cmdForJetson → mcu_position_interpolator_node.py → Gazebo
```

実機時にGazebo MCU nodeを同時起動しない。

詳細は [`../../docs/RUNTIME_ARCHITECTURE.md`](../../docs/RUNTIME_ARCHITECTURE.md)。

---

## 5. Use semantics

- `Use=True`: ALIGN/HOME/RUN/POSITION対象
- `Use=False`: RUN/POSITION送信対象外
- RUNはactive axis 0本でreject
- RUNはactive axisがcurrent sessionでaligned/homedのときだけ成立
- Use set変更時はSTOPしてsessionを整理する

---

## 6. UI commands

```text
use:<axis>:0|1
align
align:<axis>
home_move:<axis>:-1|1
home_step:<rad>
set_home:<axis>
run
stop
```

commandの意味は [`../../docs/COMMAND_REFERENCE.md`](../../docs/COMMAND_REFERENCE.md) を参照する。

---

## 7. Runtime CAN protocol

| purpose | ID |
|---|---:|
| standby heartbeat RX | `0x0FF` |
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN TX | `0x600 + axis` |
| error RX | `0x0EE` |

POSITION:

```text
[0,0,0,0] + little-endian float32(rad)
```

MCU Configは別protocolであり、`0x080|axis` / `0x180|axis` を使用する。詳細は `CAN_MCU_CONFIG_GUIDE.md`。

---

## 8. 24-element / NaN rules

- message lengthは常に24
- `Use=True` axisはfiniteかつjoint limit内
- normal roll commandは24 axis finite
- single-axis safety publisherだけはtarget以外をNaN
- NaN axisが誤ってUse=Trueならframeをreject

physical joint limitは [`../../docs/HARDWARE_LIMITS.md`](../../docs/HARDWARE_LIMITS.md) を正本とする。

---

## 9. Current transport profile

```text
resample-factor = 2
transport rate  = 10 Hz
```

これはpublisher側のtransport policyであり、StateMachine CAN protocolとは分けて扱う。

---

## 10. Related

- [`../../docs/CAN_MCU_CONFIG_GUIDE.md`](../../docs/CAN_MCU_CONFIG_GUIDE.md)
- [`../../docs/COPY_PASTE_COMMANDS.md`](../../docs/COPY_PASTE_COMMANDS.md)
- [`../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`../../docs/COMMAND_REFERENCE.md`](../../docs/COMMAND_REFERENCE.md)
- [`../mcu_config/README.md`](../mcu_config/README.md)
