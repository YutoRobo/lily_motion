# Lily CAN Interface

更新日: 2026-08-12

`tools/can_interface/` は現行Lily hardware CAN UI / StateMachineの唯一のmaintained execution targetである。`external/can_interface/` は旧snapshotであり、現行操作には使わない。

## Entry points

```bash
python2 tools/can_interface/statemachine/main.py
python2 tools/can_interface/initUI/ui.py
```

Position publishers:

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py --help
python2 tools/publish_cmdforjetson_one_leg_test.py --help
python2 tools/publish_cmdforjetson_mapped_axis_replay.py --help
python2 tools/publish_cmdforjetson_jsonl.py --help
```

## Production position input

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

## Shared upstream architecture

`/cmdForJetson` より上流はhardware/Gazebo共通。

```text
staged/frozen JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ shared normalization/resampling
→ /cmdForJetson
```

Hardware:

```text
/cmdForJetson → StateMachine → CAN → real MCU
```

Gazebo:

```text
/cmdForJetson → mcu_position_interpolator_node.py → Gazebo
```

実機時にGazebo MCU nodeを同時起動しない。

詳細:

- [`../../docs/RUNTIME_ARCHITECTURE.md`](../../docs/RUNTIME_ARCHITECTURE.md)

## 24-element and NaN rules

- message lengthは常に24
- `Use=True` axisはfiniteかつjoint limit内
- normal roll commandは24 axis finite
- single-axis safety publisherだけはtarget以外をNaN
- NaN axisが誤ってUse=Trueならframeをreject

## CAN channel

Hardware:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

vcan:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

vcan以外のbench testで `can0` を開かない。

## Use semantics

- `Use=True`: ALIGN/HOME/RUN/POSITION対象
- `Use=False`: RUN/POSITION送信対象外
- RUNはactive axis 0本でreject
- RUNはactive axisがcurrent sessionでaligned/homedのときだけ成立
- Use set変更時はSTOPしてsessionを整理する

## UI commands

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

global `home` / global `set_home` はない。

## CAN protocol

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

## Hardware limits

```text
base   ±360 deg
thigh   ±95 deg
tibia  ±150 deg
```

source:

- [`../../docs/HARDWARE_LIMITS.md`](../../docs/HARDWARE_LIMITS.md)

## Current JSONL transport profile

2026-08-12 pre-hardware staged validation:

```text
resample-factor = 2
transport rate  = 10 Hz
```

このprofileはpublisher側のtransport policyであり、StateMachineのCAN protocolを変更しない。

## Current verified status

Software / CAN:

```text
focused unified path      PASS
all CAN tests             81/81 PASS
vcan axis10               PASS
vcan axes10,11,12         PASS
```

Hardware:

```text
real axis10 +0.002 rad    visually provisional PASS
```

Still open:

- negative axis motion
- ±0.005
- one-leg
- current factor=2/10 Hz transport with real MCU
- multi-actuator synchronization
- staged rolling

## Emulator

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

- [`emulator/README.md`](emulator/README.md)

## Related

- [`../../README.md`](../../README.md)
- [`../../docs/RUNTIME_ARCHITECTURE.md`](../../docs/RUNTIME_ARCHITECTURE.md)
- [`../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`../../docs/HARDWARE_PRETEST_STATUS.md`](../../docs/HARDWARE_PRETEST_STATUS.md)
- [`../../docs/Lily_8leg_Robot_Command_Reference.md`](../../docs/Lily_8leg_Robot_Command_Reference.md)
