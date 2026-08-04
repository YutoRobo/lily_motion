# Lily CAN Interface

更新日: 2026-08-04

This directory is the only maintained execution target for the Lily hardware CAN UI and StateMachine. Do not run the legacy snapshot under `external/`.

## Entry Points

```bash
python2 tools/can_interface/statemachine/main.py
python2 tools/can_interface/initUI/ui.py
```

Position publishers:

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py --help
python2 tools/publish_cmdforjetson_jsonl.py --help
```

## Production Position Path

Production position input is only:

```text
/cmdForJetson
sensor_msgs/JointState
position length: exactly 24
unit: rad
```

The StateMachine owns CAN IDs, payload encoding, Use/session/error gates, joint limits, and logical HOME state.

```text
single-axis / one-leg / multi-leg / roll command
                    ↓
/cmdForJetson
                    ↓
StateMachine
                    ↓
Use=True axes only
                    ↓
0x400 + axis
```

The removed `/can/axis_command` route is not a production position path.

## 24-Element And NaN Rules

`JointState.position` must always contain exactly 24 elements so axis indexes remain stable.

- Every `Use=True` axis must contain a finite radian value inside its joint limit.
- Normal multi-axis and roll commands normally contain 24 finite radian values.
- The maintained single-axis safety publisher places a finite value only at the selected axis and NaN at the other 23 indexes.
- NaN is allowed only on `Use=False` axes because inactive indexes are not sent to CAN.
- If an unintended non-target axis is accidentally `Use=True`, its NaN causes the entire frame to be rejected before any POSITION CAN frame is sent.

This behavior is a safety mask, not a general recommendation to use NaN in roll command logs.

## CAN Channel Selection

`main.py` defaults to `socketcan/can0/500000` for compatibility. Bench and regression checks must select vcan explicitly.

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

Environment-variable form:

```bash
LILY_CAN_CHANNEL=vcan0 python2 tools/can_interface/statemachine/main.py
```

Do not run against `can0` until the pre-hardware checklist and vcan checks have passed.

## Use Checkbox Semantics

The UI `Use` checkbox is the active-axis selection.

- `Use=True` means the axis participates in ALIGN, HOME, RUN, and POSITION safety gates.
- `Use=False` excludes the axis from those gates and from RUN/POSITION CAN fan-out.
- RUN is rejected when no axes are active.
- RUN is accepted only when every active axis is aligned and homed in the current session.
- disconnected inactive axes do not block RUN.
- STOP is global on the PC side and sets `is_run=False` regardless of active selection.

Avoid changing Use selection after motion-session initialization. STOP and restart the session when the active set must change.

## UI Commands

StateMachine subscribes to:

```text
/ui/leg_command
```

Implemented commands include:

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

`align` requests ALIGN for all current `Use=True` axes. `align:<axis>` requests one indexed axis.

There is no implemented global `home` or global `set_home` action. HOME jog and SET HOME are indexed.

## CAN Protocol

| Purpose | ID |
|---|---:|
| standby heartbeat RX | `0x0FF` |
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN TX | `0x600 + axis` |

Position payload:

```text
[0,0,0,0] + little-endian float32(rad)
```

`0x0FF` is a discovery heartbeat in standby. Successful ALIGN may stop the standby heartbeat without implying disconnection.

## Safety Baseline

The maintained execution copy includes:

- UI STOP command
- StateMachine STOP handling (`is_run=False`)
- RUN gate based on current-session `Use=True` axes
- exact 24-element `/cmdForJetson.position` validation
- active-axis finite-value and joint-limit checks
- inactive-axis no-send behavior
- hardware limits: base `+/-360 deg`, thigh `+/-95 deg`, tibia `+/-150 deg`
- configurable CAN interface/channel/bitrate
- standby heartbeat timeout before ALIGN
- initialization and runtime error latching
- CAN send failure handling

## Single-Axis Test

The maintained publisher:

```text
tools/publish_cmdforjetson_single_axis_test.py
```

It:

- publishes a 24-element `JointState`
- uses one finite selected-axis value and 23 NaN safety guards
- never opens SocketCAN
- never sends ALIGN, HOME, RUN, or STOP
- moves from center to a signed amplitude and returns to center

Example:

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

Complete ALIGN, SET HOME, and RUN through the UI before starting it. Issue STOP afterward.

## JSONL Publisher

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --rate <hz>
```

Accepted position keys:

```text
joint_command_rad
position
joint_positions_rad
```

Each record must contain exactly 24 values.

## SocketCAN Multi-Actuator Emulator

The emulator is vcan-only and rejects hardware CAN channel names.

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

Details:

- [`emulator/README.md`](emulator/README.md)

## Verified Status

As of 2026-08-04:

```text
focused unified-path tests: 10/10 PASS
all CAN tests: 81/81 PASS
vcan axis10 single-axis: PASS
vcan axis10,11,12 fan-out: PASS
real axis10 +0.002 rad: visually provisional PASS
```

Still open:

- real negative 0.002 rad
- real positive/negative 0.005 rad
- one-leg three-axis behavior
- multiple real actuator synchronization and bus load
- sustained Jetson Orin timing and CPU measurement

## external/ Policy

`external/can_interface/260102_usb_can_fast_alignment/` is kept only as a legacy snapshot and pre-relocation reference. New execution, testing, and operational instructions must refer to `tools/can_interface/`.

## Related Documents

- [`../../README.md`](../../README.md)
- [`../../docs/Lily_8leg_Robot_Command_Reference.md`](../../docs/Lily_8leg_Robot_Command_Reference.md)
- [`../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`../../docs/HARDWARE_PRETEST_STATUS.md`](../../docs/HARDWARE_PRETEST_STATUS.md)
