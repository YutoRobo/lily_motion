# Multi-actuator SocketCAN emulator

This emulator models the current actuator MCU CAN protocol on a real SocketCAN
vcan channel. It does not emulate ROS, motor torque, mechanics, or position
feedback. The repository contains no MCU main.c, so the protocol supplied in the
2026-08-03 task is used where firmware source could not be cross-checked.

## Safety

The default channel is vcan0. The emulator rejects can0 before python-can opens
a bus, and also rejects channels that do not begin with vcan. Never substitute a
hardware CAN channel.

## Create or reuse vcan0

Check first:

    ip -details link show vcan0

If it exists, reuse it and ensure it is up:

    sudo ip link set up vcan0

If it does not exist:

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

Observe traffic:

    candump -L vcan0

Remove it when finished:

    sudo ip link delete vcan0

## Start the PC path on vcan0

The existing StateMachine arguments are reused without emulator-only branches:

    python tools/can_interface/statemachine/main.py       --can-interface socketcan       --can-channel vcan0       --can-bitrate 500000

Start the existing UI separately if needed:

    python tools/can_interface/initUI/ui.py

The normal StateMachine default remains can0; vcan0 must be selected explicitly.

## Start emulated actuators

One axis:

    python tools/can_interface/emulator/multi_actuator_emulator.py       --interface vcan0 --axes 10

Three axes with axis11 failing its first ALIGN:

    python tools/can_interface/emulator/multi_actuator_emulator.py       --interface vcan0 --axes 10,11,12 --align-fail-once 11

All axes:

    python tools/can_interface/emulator/multi_actuator_emulator.py       --interface vcan0 --axes 0-23

Mixed axis selection is accepted, for example 0-2,10,12-15,23. Empty,
duplicate, reversed, malformed, and out-of-range selections are rejected.

## Scenario options

- --align-fail-once AXES
- --align-fail-always AXES
- --align-fail-at AXIS:ATTEMPT (repeatable)
- --initialization-error-id 8, 9, or 12
- --inject-error AXIS:ERROR_ID
- --reset-after-run AXIS:SECONDS
- --heartbeat-period, --align-delay, --reset-delay

The default initialization failure is error 8 ALIMENT_ERR. It is the primary
initialization/ALIGN error in the supplied current enum. Firmware source was not
present in this repository to establish a more specific emitted error.

## Protocol behavior

Heartbeat 0x0FF is emitted every second only in aliment_standby with payload
[axis,0,0,0,0,0,0,0]. ALIGN is accepted only at 0x000|axis while in
aliment_standby. Success sends 0x100|axis with data[7]=1 and enters get_home.
Failure sends 0x0EE, enters aliment_error, resets, and returns to standby
heartbeat. HOME position 0x200|axis and normal position 0x400|axis decode a
little-endian float32 in data[4:8]. HOME complete 0x300|axis enters run_standby.
RUN 0x600|axis enters run only from run_standby. No RUN ACK, 0x0DD liveness, or
actual-position feedback is invented.

Press Ctrl-C to print per-axis summaries and shut down the vcan bus handle.
