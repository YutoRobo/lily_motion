# Lily CAN Interface

This directory is the only maintained execution target for the Lily hardware CAN UI and StateMachine. Do not run the legacy snapshot under `external/`.

## Entry Points

- StateMachine: `python tools/can_interface/statemachine/main.py`
- UI: `python tools/can_interface/initUI/ui.py`

## CAN Channel Selection

`main.py` defaults to `socketcan/can0/500000` for compatibility, but bench and regression checks must use mock or vcan.

Examples:

```bash
python tools/can_interface/statemachine/main.py --can-channel vcan0
LILY_CAN_CHANNEL=vcan0 python tools/can_interface/statemachine/main.py
```

Do not run against `can0` until the pre-hardware checklist and mock/vcan checks have passed.

## Use Checkbox Semantics

The UI `Use` checkbox is the active joint selection for normal operation. There is no separate partial bring-up mode.

- `Use=True` means the joint participates in ALIGN, HOME, and RUN safety gates.
- `Use=False` is excluded from those gates and never receives an ALIGN-start request.
- RUN is rejected when no joints are active.
- Normal RUN and `/cmdForJetson.position` send CAN frames only to Use=True axes. The JointState position vector is always 24 elements.
- `/cmdForJetson.position` must contain 24 rad values so joint indexes remain stable.
- STOP remains a global stop and sets `is_run=False` regardless of the active joint set.

## Safety Patch Baseline

The maintained execution copy includes the Lily safety patches:

- UI STOP command
- StateMachine STOP handling (`is_run=False`)
- RUN gate requiring only UI `Use=True` active joints to be aligned and homed in the current session
- `/cmdForJetson.position` length check (`24` required)
- hardware_limit_v2 rejection on RUN position commands, rad input assumed
- configurable CAN interface/channel/bitrate
- standby-only heartbeat timeout; ALIGN/HOME/RUN state is not cleared merely because 0x0FF stops after ALIGN

## external/ Policy

`external/can_interface/260102_usb_can_fast_alignment/` is kept only as a legacy snapshot / pre-relocation reference. New execution, testing, and operational instructions must refer to this `tools/can_interface/` copy.

## Hardware Operation Docs

Before any hardware trial, read:

- `docs/HARDWARE_PRETEST_STATUS.md`
- `docs/HARDWARE_OPERATION_PROCEDURE.md`

These documents define the current pretest status, execution path, Use=True semantics, staged hardware procedure, and explicit prohibitions.

## Position command input

Production position input is only `/cmdForJetson` (`sensor_msgs/JointState`, 24 rad values). Use=True controls RUN and position CAN fan-out. UI Diagnostic RUN and motion check continue through `/ui/leg_command`; CAN IDs, payloads, Use/session/error gates, position limits, and logical q0 remain owned by StateMachine.

## SocketCAN multi-actuator emulator

The vcan-only actuator MCU emulator is documented in emulator/README.md. It rejects can0 before opening python-can and uses the existing StateMachine --can-interface and --can-channel options.
