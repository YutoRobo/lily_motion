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
- Legacy normal RUN start and `/cmdForJetson.position` remain all-24-axis fan-out. The selected-axis diagnostic path is the only single-axis RUN/position path.
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

## External selected-axis diagnostic input

The existing /cmdForJetson JointState input remains the legacy 24-axis production stream and cannot represent selected-axis Diagnostic RUN. Selected-axis offline or suspended-robot diagnostics use /can/axis_command with std_msgs/String:

- diagnostic_run:<axis>
- position:<axis>:<absolute_rad>
- position_offset:<axis>:<offset_from_diagnostic_q0_rad>
- stop

Both /ui/leg_command and /can/axis_command converge at StateMachine.submit_axis_command. CAN IDs, payloads, Use/session/error gates, position limits, and logical q0 are owned by StateMachine. The external helper publish_single_axis_external_test.py publishes only ROS commands and never opens SocketCAN.

Example:

    python tools/can_interface/publish_single_axis_external_test.py --axis 11 --direction plus

For axis 11, successful execution sends one 0x60B Diagnostic RUN frame followed by 0x40B position frames only. It does not send a new 0x00B ALIGN request.

## SocketCAN multi-actuator emulator

The vcan-only actuator MCU emulator is documented in emulator/README.md. It rejects can0 before opening python-can and uses the existing StateMachine --can-interface and --can-channel options.
