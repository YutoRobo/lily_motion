# Lily Operator desktop launcher

The desktop launchers provide four operating modes without changing the CAN protocol or MCU firmware.

## One-time installation

From the `lily_motion` repository:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs:

- `Lily Operator` — physical CAN (`can0`) for hardware operation only;
- `Lily Operator (vcan0)` — virtual CAN + MCU emulator for PC-side software testing;
- `Lily Operator (Gazebo)` — Gazebo-only motion path with CAN StateMachine disabled;
- `Lily Operator (Hardware + Gazebo)` — automatically prepare Gazebo world/controllers/interpolator, then start the unchanged normal physical-CAN Operator.

The installer also removes obsolete desktop entries from the abandoned CAN-to-Gazebo Sync Bridge experiment.

## Physical CAN use

Double-click `Lily Operator`.

This path is unchanged. The launcher prepares physical CAN, ROS, and starts the existing integrated Operator UI. It does not start Gazebo, Gazebo controllers, or the Gazebo interpolator.

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

## Gazebo-only use

Double-click `Lily Operator (Gazebo)`.

This deliberately does not create a CAN StateMachine. It uses the Gazebo-only Motion UI and the existing Gazebo MCU interpolator.

The Gazebo model and joint controllers must already be running for this mode.

## Hardware + Gazebo automatic use

Double-click `Lily Operator (Hardware + Gazebo)`.

The combined launcher is an external wrapper. It does **not** modify or replace the normal `Lily Operator` launcher. Its startup sequence is:

```text
ROS environment / ROS master
    ↓
roslaunch lily_octpus_gazebo lily_octpus_world.launch
    ↓
Gazebo readiness check
    ↓
configurable settling delay (default 2.0 s)
    ↓
roslaunch lily_octpus_control lily_octpus_control.launch
    ↓
24 controller command-subscriber readiness check
    ↓
/lily_gazebo_mcu_position_interpolator
    ↓
unchanged normal Lily Operator launcher
```

Gazebo readiness requires both `/gazebo/model_states` and `/gazebo/get_world_properties` to exist in the ROS graph.

Controller readiness uses the 24 canonical Gazebo command topics from `lily_motion_v3/interface_config.py`. Before starting the control launch:

- 24/24 ready: reuse the existing controller session;
- 0/24 ready: start `lily_octpus_control.launch`;
- partial readiness such as 18/24: stop with an error instead of launching duplicate controllers.

After the control launch, the wrapper waits until all 24 controller command topics have subscribers.

The synchronized command path remains:

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
- unknown `/cmdForJetson` subscribers are rejected;
- additional `/cmdForJetson` publishers are rejected;
- topology is rechecked while Motion SEND is active.

Do not separately open `Lily Operator` before using the combined launcher. The combined launcher refuses to start when `/lily_operator` is already running.

### Existing process reuse and cleanup

The combined launcher reuses an already-ready ROS master, Gazebo world, all-24-controller session, or Gazebo interpolator when present.

Only processes started by the combined launcher are stopped when its normal Operator session ends. Cleanup is performed in reverse order: interpolator, control roslaunch, world roslaunch, then roscore if this launcher created it. Broad `pkill gazebo`-style cleanup is not used.

If a `/gazebo` node exists but Gazebo readiness is incomplete, or if only part of the 24 controller command topics are ready, startup fails instead of creating a second Gazebo/control session.

## Logs

Launcher output is written under:

```text
runtime_logs/operator_ui/launcher/
```

The combined launcher log is named:

```text
hardware_gazebo_launcher_YYYYmmdd_HHMMSS.log
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

Gazebo-related launchers:

```text
LILY_GAZEBO_INTERP_DURATION       default: 0.100
LILY_GAZEBO_UPDATE_PERIOD         default: 0.002
```

Combined Hardware + Gazebo launcher additionally accepts:

```text
LILY_GAZEBO_WORLD_PACKAGE         default: lily_octpus_gazebo
LILY_GAZEBO_WORLD_LAUNCH          default: lily_octpus_world.launch
LILY_GAZEBO_CONTROL_PACKAGE       default: lily_octpus_control
LILY_GAZEBO_CONTROL_LAUNCH        default: lily_octpus_control.launch
LILY_GAZEBO_CONTROL_DELAY_SEC     default: 2.0
LILY_GAZEBO_READY_TIMEOUT_SEC     default: 30.0
LILY_GAZEBO_CONTROL_TIMEOUT_SEC   default: 30.0
```
