# Lily Operator desktop launcher

The desktop launchers provide three operating modes without changing the CAN protocol or MCU firmware.

## One-time installation

From the `lily_motion` repository:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs:

- `Lily Operator` — physical CAN (`can0`) for hardware operation;
- `Lily Operator (vcan0)` — virtual CAN + MCU emulator for PC-side software testing;
- `Lily Operator (Gazebo)` — Gazebo-only motion path with CAN StateMachine disabled.

The installer also removes obsolete desktop entries from the abandoned CAN-to-Gazebo Sync Bridge experiment.

## Physical CAN use

Double-click `Lily Operator`.

The launcher:

1. finds `can0` (or `LILY_CAN_CHANNEL`);
2. leaves it unchanged if it is already UP at `500000 bit/s`;
3. otherwise requests administrator authentication and configures the physical CAN interface;
4. sources ROS Melodic and the catkin workspace when present;
5. reuses a reachable ROS master or starts `roscore` when needed;
6. starts the existing integrated Lily Operator UI.

The hardware command path remains:

```text
Lily Operator Motion
    ↓
/cmdForJetson
    ↓
/lily_operator StateMachine
    ↓
CAN
    ↓
MCU / hardware
```

## Virtual CAN use

Double-click `Lily Operator (vcan0)`.

If `vcan0` does not exist, the launcher creates and brings it UP. Virtual CAN has no physical bitrate, so no CAN bitrate is applied to `vcan0`.

The same integrated Operator UI is used; MCU-originated responses still require the separate MCU emulator.

## Gazebo-only use

Double-click `Lily Operator (Gazebo)`.

This deliberately does not create a CAN StateMachine. It uses:

```text
Gazebo-only Motion UI
    ↓
/cmdForJetson
    ↓
/lily_gazebo_mcu_position_interpolator
    ↓
24 Gazebo joint controller topics
    ↓
Lily Gazebo model
```

The Gazebo model and joint controllers must already be running in the existing Gazebo environment.

## Hardware + Gazebo synchronized command use

For synchronized comparison, use the **normal `Lily Operator`**, not a separate Sync Bridge.

Start the existing Gazebo MCU interpolator in the same ROS graph:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

Then one Motion SEND is shared directly:

```text
                     /cmdForJetson
                          │
             ┌────────────┴────────────┐
             ↓                         ↓
/lily_operator StateMachine   /lily_gazebo_mcu_position_interpolator
             ↓                         ↓
            CAN                Gazebo joint controllers
             ↓                         ↓
         real hardware                 Gazebo
```

The normal Operator safety policy remains restrictive:

- `/lily_operator` StateMachine subscriber is mandatory;
- `/lily_gazebo_mcu_position_interpolator` is the only optional second subscriber;
- any unknown `/cmdForJetson` subscriber is rejected;
- any additional `/cmdForJetson` publisher is rejected;
- the topology is rechecked while Motion SEND is active and SEND is aborted if it changes to an unapproved topology.

Thus hardware-only remains valid with one subscriber, while hardware + Gazebo is valid with the two known subscribers.

Do not run `Lily Operator (Gazebo)` at the same time as the normal `Lily Operator` for synchronized hardware operation; use the standalone `mcu_position_interpolator_node.py` with the normal Operator instead.

## Logs

Launcher output is written under:

```text
runtime_logs/operator_ui/launcher/
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

Gazebo-only launcher additionally accepts:

```text
LILY_GAZEBO_INTERP_DURATION   default: 0.100
LILY_GAZEBO_UPDATE_PERIOD     default: 0.002
```
