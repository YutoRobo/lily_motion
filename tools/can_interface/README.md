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

- `Use=True` means the joint is active. ALIGN, HOME jog, SET HOME, RUN start, and RUN position CAN frames are sent only for active joints.
- `Use=False` means the joint is inactive. It is excluded from the RUN gate and receives no `0x400+i` position frames or `0x600+i` RUN start frame.
- RUN is rejected when no joints are active.
- `/cmdForJetson.position` must still contain 24 rad values so joint indexes remain stable. During RUN, hardware_limit_v2 is enforced for active joints only; inactive out-of-limit values are logged as ignored warnings.
- STOP remains a global stop and sets `is_run=False` regardless of the active joint set.

## Safety Patch Baseline

The maintained execution copy includes the Lily safety patches:

- UI STOP command
- StateMachine STOP handling (`is_run=False`)
- RUN gate requiring only UI `Use=True` active joints to be connected, aligned, and homed
- `/cmdForJetson.position` length check (`24` required)
- hardware_limit_v2 rejection on RUN position commands for active joints, rad input assumed
- configurable CAN interface/channel/bitrate
- connection timeout handling that clears aligned/homed state and stops RUN only when the timed-out joint is active

## external/ Policy

`external/can_interface/260102_usb_can_fast_alignment/` is kept only as a legacy snapshot / pre-relocation reference. New execution, testing, and operational instructions must refer to this `tools/can_interface/` copy.

## Hardware Operation Docs

Before any hardware trial, read:

- `docs/HARDWARE_PRETEST_STATUS.md`
- `docs/HARDWARE_OPERATION_PROCEDURE.md`

These documents define the current pretest status, execution path, Use=True semantics, staged hardware procedure, and explicit prohibitions.
