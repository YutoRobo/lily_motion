# Lily Operator desktop launcher

The desktop launchers provide three intentionally separated operating modes without changing the CAN protocol or MCU firmware.

## One-time installation

From the `lily_motion` repository on each PC / Jetson:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs three launch modes on the Desktop and in the application menu:

- `Lily Operator` — physical CAN (`can0`) for hardware operation;
- `Lily Operator (vcan0)` — virtual CAN + MCU emulator for PC-side software testing;
- `Lily Operator (Gazebo)` — Gazebo motion path with CAN StateMachine disabled.

On some Ubuntu 18.04 desktops, the first double-click may ask whether the desktop file should be trusted. Choose `Trust and Launch`.

## Physical CAN use

Double-click `Lily Operator`.

The launcher performs these checks in order:

1. Find `can0` (or `LILY_CAN_CHANNEL`).
2. If CAN is already `UP` at `500000 bit/s`, leave it unchanged.
3. Otherwise request administrator authentication and configure the physical CAN interface.
4. Source `/opt/ros/melodic/setup.bash`.
5. Source `~/catkin_ws/devel/setup.bash` when that file exists.
6. Check whether a ROS master is already reachable.
7. Start `roscore` only when no ROS master is reachable.
8. Start `tools/operator_ui/lily_operator_integrated.py` with the selected SocketCAN channel.

Physical CAN setup uses administrator permission only for the `ip link` commands. The Operator UI itself runs as the logged-in user.

When configuration is required, the physical-CAN path is equivalent to:

```bash
sudo ip link set can0 down   # tolerated if already down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

## Virtual CAN use

Double-click `Lily Operator (vcan0)`.

Virtual CAN has no physical bus bitrate, so the launcher does **not** apply the `500000 bit/s` setting to `vcan0`.

If `vcan0` already exists and is UP, it is reused unchanged. If it exists but is down, the launcher brings it UP. If it does not exist, the launcher requests administrator authentication and performs the equivalent of:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

The same integrated Operator UI opens with `--can-channel vcan0`. A vcan interface by itself does not emulate MCU behavior; ALIGN/HOME/RUN acknowledgements, telemetry, Config responses, and other MCU-originated frames require the MCU emulator.

## Gazebo use

Double-click `Lily Operator (Gazebo)`.

Gazebo mode deliberately does **not** create a CAN bus or a CAN StateMachine. It uses the existing shared command boundary:

```text
JSONL Motion UI
    ↓
/cmdForJetson
    ↓
tools/gazebo/mcu_position_interpolator_node.py
    ↓
24 Gazebo joint controller topics
    ↓
Lily Gazebo model
```

The Gazebo launcher:

1. sources ROS/catkin;
2. starts `roscore` only when needed;
3. reuses an already-running `/lily_gazebo_mcu_position_interpolator` node, or starts one automatically;
4. opens `tools/operator_ui/lily_operator_gazebo.py`, which contains the existing MotionPanel but no CAN StateMachine or hardware controls;
5. stops the Gazebo MCU interpolator on exit only when that interpolator was started by this launcher.

The Gazebo model and its joint controllers are still external to this repository and must already be running through the existing Gazebo environment.

The existing Motion safety topology check remains active. `/cmdForJetson` must have the intended single consumer for this mode. If a hardware/vcan integrated Operator is simultaneously connected to the same ROS master, SEND is rejected rather than broadcasting to both CAN and Gazebo.

Default Gazebo interpolation settings are:

```text
LILY_GAZEBO_INTERP_DURATION = 0.100 s
LILY_GAZEBO_UPDATE_PERIOD   = 0.002 s
```

These can be overridden before a terminal launch if needed.

## Mode separation

```text
Lily Operator
  /cmdForJetson -> CAN StateMachine -> can0 -> MCU

Lily Operator (vcan0)
  /cmdForJetson -> CAN StateMachine -> vcan0 -> MCU emulator

Lily Operator (Gazebo)
  /cmdForJetson -> Gazebo MCU interpolator -> Gazebo joint controllers
```

The separation is intentional: the physical/vcan StateMachine path and Gazebo consumer are not connected to `/cmdForJetson` at the same time.

## PC and Jetson

The launchers are Bash scripts, not compiled x86_64 or ARM binaries. The same files can therefore be used on an x86_64 desktop PC and an aarch64 Jetson when the required local Python/ROS environment exists.

You can confirm architecture with:

```bash
uname -m
```

## Logs

Launcher output is written under:

```text
runtime_logs/operator_ui/launcher/
```

The Gazebo launcher uses `gazebo_launcher_*.log`; the physical/vcan launcher uses `launcher_*.log`.

## Environment overrides

Physical/vcan launcher:

```text
LILY_CAN_CHANNEL     default: can0
LILY_CAN_BITRATE     default: 500000 (physical CAN only)
LILY_ROS_SETUP       default: /opt/ros/melodic/setup.bash
LILY_CATKIN_SETUP    default: ~/catkin_ws/devel/setup.bash
LILY_OPERATOR_LOG_ROOT
```

Gazebo launcher additionally accepts:

```text
LILY_GAZEBO_INTERP_DURATION   default: 0.100
LILY_GAZEBO_UPDATE_PERIOD     default: 0.002
```
