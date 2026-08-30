# Lily Operator UI v0

This feature branch keeps the maintained CAN StateMachine behavior while integrating the operator-facing controls into one window.

## Quick start

Update the feature branch and launch the integrated Operator UI with:

```bash
git checkout feature/monitor-csv-load
git pull
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
python2 tools/operator_ui/lily_operator_integrated.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

Do **not** start `tools/can_interface/statemachine/main.py` in another terminal when using the integrated launcher.

For normal desktop operation, `tools/operator_ui/install_desktop_launcher.sh` installs `Lily Operator`, `Lily Operator (vcan0)`, and `Lily Operator (Gazebo)` shortcuts.

## Preferred start: integrated Operator UI

The integrated launcher owns the existing StateMachine, CAN connection, Control UI, JSONL Motion panel, receive-only position Monitor, and MCU Config panel:

```bash
python2 tools/operator_ui/lily_operator_integrated.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

Optional initial Monitor target:

```bash
python2 tools/operator_ui/lily_operator_integrated.py --monitor-leg 4
```

`--monitor-leg` is one-based (`1..8`) and can also be changed from the Monitor tab after startup.

For a single unloaded test axis, the MCU Config tab can be limited to that axis, for example:

```bash
python2 tools/operator_ui/lily_operator_integrated.py --config-axes 11
```

Accepted examples are `--config-axes 0-23`, `--config-axes 11`, and `--config-axes 9-11`.

## Window layout

```text
Lily Operator | CAN / online-axis / RUN status                         [STOP]
----------------------------------------------------------------------------
[ Control ] [ Motion ] [ Monitor ] [ MCU Config ]
```

- **Control**: maintained Use / ALIGN / HOME / RUN controls.
- **Motion**: one JSONL at a time, `LOAD / CHECK -> SEND`.
- **Monitor**: embedded maintained MCU position-debug viewer.
- **MCU Config**: READ / WRITE / SAVE / Echo for HardwareConfig and SoftwareConfig.
- **STOP**: remains visible above the tabs at all times.

## MCU Config tab

The Config tab reuses the maintained protocol implementation from `tools/mcu_config/lily_mcu_config_editor.py` rather than defining another CAN protocol.

Protocol remains:

```text
Request ID  = 0x080 | axis
Response ID = 0x180 | axis
READ  = 0x01
WRITE = 0x02
SAVE  = 0x03
HW    = 0x01
SW    = 0x02
```

The integrated panel provides:

- selected-axis READ;
- all-config-axis READ / discovery;
- HardwareConfig and SoftwareConfig parameter display;
- WRITE to RAM;
- response Echo plus READ-back verification;
- HardwareConfig SAVE;
- SoftwareConfig SAVE;
- HardwareConfig reboot-required indication.

Operator-side safety rules:

- READ is allowed while RUN is active;
- WRITE and SAVE are disabled while RUN is active;
- WRITE and SAVE are disabled while JSONL Motion SEND is active;
- while a Config WRITE/SAVE transaction is in flight, Use / ALIGN / HOME / RUN controls are temporarily interlocked;
- the MCU remains the final authority and may return `INVALID_STATE` or another protocol error;
- HardwareConfig SAVE still requires power cycling before the normal ALIGN workflow.

The maintained standalone Config Editor remains available for diagnosis and comparison.

## Monitor tab

The Monitor tab reuses the maintained `tools/diagnostics/realtime_position_debug_viewer_ui.py` implementation rather than duplicating its parser, CSV, or plotting logic.

Controls include:

- target Leg `1..8`;
- `APPLY TARGET` to rebuild the monitor for that leg's three axes and return to live mode;
- `LOAD CSV...` to load a previously recorded Monitor CSV in offline mode;
- measurement Duration;
- `START / STOP / CLEAR`;
- command / actual plots for the selected axes;
- tracking-error plot;
- CSV logging behavior from the standalone viewer.

Changing the target leg or loading a CSV is rejected while a monitor measurement is active. Stop the monitor measurement first.

The embedded monitor uses the same telemetry definition as the standalone viewer:

```text
CAN ID   = 0x500 | axis
byte 0-3 = internal position command [rad], float32 little-endian
byte 4-7 = actual position [rad], float32 little-endian
```

The embedded Monitor retains a long sample history while limiting plot redraw work so that the shared Operator UI loop is not dominated by Matplotlib.

### Offline CSV exchange

Monitor CSV files can be copied between PCs and reopened with the same command/actual/error plots. The loader reads the axis IDs from the CSV automatically, so the receiving PC does not need to select the original Leg first.

Inside the integrated Operator UI:

```text
Monitor -> LOAD CSV... -> select position_debug_*.csv
```

Offline CSV mode is static: it does not send CAN frames, stops the Monitor's receive-only `candump` reader after the file is loaded, and performs no automatic plot rescaling. Use the Matplotlib toolbar above the plots for `Home / Pan / Zoom`.

While the CSV is displayed, `START` remains available. Pressing `START` clears the offline waveform, restarts receive-only `candump`, and begins a fresh live measurement using the **same axis IDs that were loaded from the CSV**. The Duration field remains editable, and the new live measurement is logged to CSV using the normal Monitor logging rules.

To switch to a different leg instead, choose the target Leg and press `APPLY TARGET`.

For an analysis PC that does not need the integrated StateMachine or ROS UI, the Monitor can also be started directly:

```bash
python2 tools/operator_ui/position_monitor_panel.py \
  --csv /path/to/position_debug_leg05_target002.csv
```

This direct CSV-viewer path does not initialize the Operator StateMachine or ROS. A live CAN connection is not required to inspect an already recorded CSV. `Tkinter` and `Matplotlib` are still required on the analysis PC. If that PC has the corresponding SocketCAN interface available, pressing `START` from the offline display attempts a new live capture for the CSV's axes.

The canonical CSV columns used for loading are:

```text
time_sec
axis
command_rad
actual_rad
```

The tracking-error and degree values are recomputed from these columns when the CSV is loaded. The normal Monitor CSV contains these columns plus timestamp and precomputed error/degree columns.

The standalone realtime viewer remains available for diagnosis, but it is no longer required for normal Operator UI use.

## Normal multi-file motion flow

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
- `/cmdForJetson` has an approved subscriber topology;
- no other ROS node is publishing `/cmdForJetson`.

For the normal integrated node `/lily_operator`, the approved `/cmdForJetson` subscriber topology is:

```text
hardware only:
  /lily_operator

hardware + Gazebo:
  /lily_operator
  /lily_gazebo_mcu_position_interpolator
```

`/lily_operator` is mandatory. The Gazebo MCU interpolator is the only optional second subscriber. Any unknown subscriber is rejected. Other MotionPanel hosts retain the previous exactly-one-subscriber rule.

Publisher/subscriber topology is rechecked while SEND is active. If RUN is lost, another `/cmdForJetson` publisher appears, the required StateMachine subscriber disappears, or an unknown subscriber appears, the Operator UI stops publishing the remaining frames.

While SEND is active, the Operator UI disables Use / ALIGN / HOME / RUN and legacy diagnostic-motion controls. The global STOP control remains available.

## Hardware + Gazebo shared command

The existing Gazebo splitter can subscribe to the same `/cmdForJetson` stream as the hardware StateMachine:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

One Motion SEND then drives both the real hardware path and the Gazebo joint-controller path. The abandoned CAN-to-Gazebo Sync Bridge is not used.

See `docs/GAZEBO_USAGE_GUIDE.md` and `tools/operator_ui/DESKTOP_LAUNCHER.md` for the operating sequence.

## Current motion defaults

The UI starts with:

- resample factor: `5`
- rate: `10 Hz`

This corresponds to the current operator baseline of RF5 / 10 Hz. Changing the resample factor after LOAD requires another LOAD / CHECK. Rate is validated again at SEND time.

## Tests

Pure tests that do not require CAN hardware include:

```bash
python2 tests/test_operator_motion_stream.py
python2 tests/test_motion_topology.py
```

The topology test covers hardware-only, hardware + Gazebo, missing StateMachine, unknown subscriber, and connection-count mismatch cases.

## Scope of v0

This branch still intentionally does not change:

- the existing CAN StateMachine command IDs or state transitions;
- MCU firmware;
- the existing standalone JSONL publisher;
- staged motion files;
- the maintained standalone realtime position debug viewer;
- the maintained standalone MCU Config editor.

The Operator UI integrates the maintained Monitor and MCU Config functionality without removing those standalone diagnostic tools.
