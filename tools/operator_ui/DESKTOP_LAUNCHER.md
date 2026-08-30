# Lily Operator desktop launcher

The desktop launchers provide intentionally separated operating modes without changing the CAN protocol or MCU firmware.

## One-time installation

From the `lily_motion` repository on each PC / Jetson:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs four launch entries on the Desktop and in the application menu:

- `Lily Operator` — physical CAN (`can0`) for hardware operation;
- `Lily Operator (vcan0)` — virtual CAN + MCU emulator for PC-side software testing;
- `Lily Operator (Gazebo)` — Gazebo-only motion path with CAN StateMachine disabled;
- `Lily Gazebo Sync Bridge` — external receive-only bridge used **together with the normal physical Lily Operator** to mirror commands already sent on CAN into Gazebo.

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

## Gazebo-only use

Double-click `Lily Operator (Gazebo)`.

Gazebo-only mode deliberately does **not** create a CAN bus or a CAN StateMachine. It uses the shared command boundary:

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

The existing Motion safety topology check remains active. `/cmdForJetson` must have the intended single consumer for this mode.

## Simultaneous hardware + Gazebo synchronization

For synchronization, do **not** use `Lily Operator (Gazebo)`. Use the normal physical `Lily Operator` and the external `Lily Gazebo Sync Bridge`.

```text
normal Lily Operator Motion
        ↓
/cmdForJetson
        ↓
existing StateMachine
        ↓
CAN 0x400+axis
        ├────────→ real MCU
        └→ candump receive-only
               ↓
        CAN->Gazebo sync bridge
               ↓
        Gazebo joint controllers
```

The sync bridge is intentionally outside Lily Operator. It does not modify or replace:

- `lily_operator_integrated.py`;
- MotionPanel;
- StateMachine;
- the CAN protocol;
- MCU firmware/config behavior.

It also does **not** subscribe to `/cmdForJetson`, so the existing Motion `exactly one subscriber` protection remains unchanged.

Recommended startup order:

1. Start Gazebo + Lily model + joint controllers.
2. Double-click `Lily Operator` and operate the real hardware normally through ALIGN / HOME / RUN.
3. Double-click `Lily Gazebo Sync Bridge`.
4. LOAD / CHECK and SEND from the normal Lily Operator.

The sync launcher refuses to start if the normal `/lily_operator` is not running or if the Gazebo-only `/lily_gazebo_mcu_position_interpolator` is still running.

Detailed procedure and architecture are in:

```text
tools/gazebo/CAN_GAZEBO_SYNC.md
```

## Mode summary

```text
Lily Operator
  /cmdForJetson -> CAN StateMachine -> can0 -> MCU

Lily Operator (vcan0)
  /cmdForJetson -> CAN StateMachine -> vcan0 -> MCU emulator

Lily Operator (Gazebo)
  /cmdForJetson -> Gazebo MCU interpolator -> Gazebo joint controllers

Lily Operator + Lily Gazebo Sync Bridge
  /cmdForJetson -> existing CAN StateMachine -> can0 -> MCU
                                           └-> receive-only CAN mirror -> Gazebo
```

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

Relevant log patterns:

```text
launcher_*.log                 physical/vcan Operator
gazebo_launcher_*.log          Gazebo-only Operator
gazebo_sync_bridge_*.log       hardware + Gazebo sync bridge
```

## Environment overrides

Physical/vcan launcher:

```text
LILY_CAN_CHANNEL     default: can0
LILY_CAN_BITRATE     default: 500000 (physical CAN only)
LILY_ROS_SETUP       default: /opt/ros/melodic/setup.bash
LILY_CATKIN_SETUP    default: ~/catkin_ws/devel/setup.bash
LILY_OPERATOR_LOG_ROOT
```

Gazebo output parameters used by Gazebo-only and sync paths:

```text
LILY_GAZEBO_INTERP_DURATION    default: 0.100
LILY_GAZEBO_UPDATE_PERIOD      default: 0.002
```

Sync bridge additionally accepts:

```text
LILY_GAZEBO_CAN_COALESCE_SEC   default: 0.002
```
