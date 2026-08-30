# Lily Operator desktop launcher

The desktop launcher starts the existing integrated Operator UI without changing the StateMachine, CAN protocol, MCU firmware, or motion logic.

## One-time installation

From the `lily_motion` repository on each PC / Jetson:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs:

- a `Lily Operator` icon on the current user's Desktop;
- a `Lily Operator` entry in the application menu.

On some Ubuntu 18.04 desktops, the first double-click may ask whether the desktop file should be trusted. Choose `Trust and Launch`.

## Normal use

Double-click `Lily Operator`.

The launcher performs these checks in order:

1. Find `can0` (or `LILY_CAN_CHANNEL`).
2. If CAN is already `UP` at `500000 bit/s`, leave it unchanged.
3. Otherwise request administrator authentication and configure the CAN interface.
4. Source `/opt/ros/melodic/setup.bash`.
5. Source `~/catkin_ws/devel/setup.bash` when that file exists.
6. Check whether a ROS master is already reachable.
7. Start `roscore` only when no ROS master is reachable.
8. Start `tools/operator_ui/lily_operator_integrated.py` with the existing SocketCAN settings.

CAN setup uses administrator permission only for the `ip link` commands. The Operator UI itself runs as the logged-in user.

## CAN commands represented by the launcher

When configuration is required, the launcher is equivalent to:

```bash
sudo ip link set can0 down   # tolerated if already down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

If the correct bitrate is already configured and only the interface is down, it only brings `can0` up.

## PC and Jetson

The launcher is a Bash script, not a compiled x86_64 or ARM binary. Therefore the same launcher files can be used on both an x86_64 desktop PC and an aarch64 Jetson, provided the required local environment exists:

- Ubuntu / compatible desktop environment;
- ROS Melodic;
- Python 2;
- `can-utils` / SocketCAN tools;
- the Lily Python dependencies already used by the Operator UI.

You can confirm the CPU architecture with:

```bash
uname -m
```

The Python/ROS packages are still the packages installed locally on each machine.

## Logs

Launcher output is written under:

```text
runtime_logs/operator_ui/launcher/
```

If startup fails, the error dialog includes the relevant launcher log path when possible.

## Environment overrides

Defaults can be overridden before launching if needed:

```text
LILY_CAN_CHANNEL     default: can0
LILY_CAN_BITRATE     default: 500000
LILY_ROS_SETUP       default: /opt/ros/melodic/setup.bash
LILY_CATKIN_SETUP    default: ~/catkin_ws/devel/setup.bash
LILY_OPERATOR_LOG_ROOT
```
