# Lily Operator UI v0

This branch-only UI keeps the existing CAN StateMachine logic and adds one operator-facing JSONL motion panel to the existing Leg Control UI.

## Start

The normal entry point is now the integrated launcher. It starts the CAN StateMachine and the Operator UI in one process:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
python2 tools/operator_ui/lily_operator_integrated.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

Do **not** start `tools/can_interface/statemachine/main.py` separately when using the integrated launcher. The integrated process owns the CAN StateMachine and publishes checked motion targets to `/cmdForJetson` through the same ROS node.

The older UI-only entry point remains available for diagnosis only:

```bash
python2 tools/operator_ui/lily_operator_ui.py
```

When using that UI-only entry point, the existing StateMachine must be started separately or every axis will remain `Disconnected`.

## Normal multi-file flow

The intended workflow is one RUN session with one explicitly selected JSONL at a time:

```text
ALIGN
-> HOME
-> RUN
-> select air-entry JSONL
-> LOAD / CHECK
-> SEND
-> final air-entry posture is held while RUN remains active
-> inspect / controlled touchdown
-> select roll JSONL
-> LOAD / CHECK
-> SEND
-> inspect
```

Do not press STOP between normal consecutive files. STOP remains an abnormal/emergency operation.

## LOAD / CHECK

LOAD never publishes a position command. It:

- builds the exact transport stream using the existing `prepare_transport_stream()` path;
- checks all transport frames are 24-axis, finite, and inside the same documented joint limits;
- rejects a transport frame-to-frame jump of 4 deg or larger;
- computes the transport SHA256;
- keeps the checked transport stream in memory so SEND does not re-read the file;
- checks the loaded first command against the current Operator UI continuity reference.

For the first UI-managed motion in a RUN session, the continuity reference is HOME logical zero. After a successful SEND, the actual last published UI command becomes the reference for the next JSONL. When the RUN session is ended and axes return from Running, the reference resets to HOME zero.

Changing the file path or resample factor after LOAD disables SEND until LOAD / CHECK is performed again. This prevents the selected-file display from drifting away from the checked in-memory stream.

## SEND interlocks

SEND is enabled only when:

- a JSONL has passed LOAD / CHECK;
- every `Use=True` axis is shown as `Running`;
- the boundary to the loaded first command is below 4 deg;
- the file path and resample factor still match the checked stream;
- the legacy RUN motion check is not active;
- no Operator JSONL SEND is already active;
- `/cmdForJetson` has exactly one subscriber;
- no other ROS node is publishing `/cmdForJetson`.

Publisher/subscriber topology is rechecked while SEND is active. If RUN is lost, another `/cmdForJetson` publisher appears, or the subscriber topology changes, the Operator UI stops publishing the remaining frames.

While SEND is active, the Operator UI disables Use / ALIGN / HOME / RUN and legacy diagnostic-motion controls. The global STOP control remains available for abnormal conditions.

The integrated launcher also prevents closing the application while an axis is still shown as `Running`. End the RUN session before closing so the CAN backend is not removed while the MCU remains in RUN.

## Current defaults

The UI starts with the committed pre-hardware transport defaults:

- resample factor: `2`
- rate: `10 Hz`

Changing the resample factor after LOAD requires another LOAD / CHECK. Rate is validated again at SEND time.

## Tests

The branch includes a pure JSONL/transport test that does not require Tkinter or CAN:

```bash
python2 tests/test_operator_motion_stream.py
```

It checks the HOME -> air-entry boundary, air-entry -> full-roll boundary, the 4 deg continuity rule, and invalid resample rejection.

## Scope of v0

This v0 intentionally does not change:

- the existing CAN StateMachine command IDs or state transitions;
- MCU firmware;
- the existing standalone JSONL publisher;
- staged motion files;
- the realtime position debug viewer;
- the MCU Config editor.

The viewer and MCU Config can be integrated into the same operator surface later after the JSONL SEND flow is validated on the Jetson.
