# Lily CAN Interface

This directory is the maintained execution target for the Lily hardware CAN UI and StateMachine.

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

## Safety Patch Baseline

The maintained execution copy includes the Lily safety patches:

- UI STOP command
- StateMachine STOP handling (`is_run=False`)
- RUN gate requiring all 24 axes to be connected, aligned, and homed
- `/cmdForJetson.position` length check (`24` required)
- hardware_limit_v2 rejection on RUN position commands, rad input assumed
- configurable CAN interface/channel/bitrate
- connection timeout handling that also clears aligned/homed state and stops RUN

## external/ Policy

`external/can_interface/260102_usb_can_fast_alignment/` is kept as the imported-source archive / legacy reference location. New execution, testing, and operational instructions should refer to this `tools/can_interface/` copy.
