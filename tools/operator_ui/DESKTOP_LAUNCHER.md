# Lily Operator desktop launcher

The desktop launcher starts the existing integrated Operator UI without changing the StateMachine, CAN protocol, MCU firmware, or motion logic.

## One-time installation

From the `lily_motion` repository on each PC / Jetson:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This installs two launch modes on the Desktop and in the application menu:

- `Lily Operator` — physical CAN (`can0`) for hardware operation;
- `Lily Operator (vcan0)` — virtual CAN for PC-side software testing.

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

If the correct bitrate is already configured and only the interface is down, it only brings `can0` up.

## Virtual CAN use

Double-click `Lily Operator (vcan0)`.

Virtual CAN has no physical bus bitrate, so the launcher does **not** apply the `500000 bit/s` setting to `vcan0`.

If `vcan0` already exists and is UP, it is reused unchanged. If it exists but is down, the launcher brings it UP. If it does not exist, the launcher requests administrator authentication and performs the equivalent of:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

The same integrated Operator UI then opens with `--can-channel vcan0`.

`vcan0` is useful for checking UI behavior, CAN transmit traffic, ROS/Motion paths, and software-side integration without a physical CAN adapter. A vcan interface by itself does not emulate MCU behavior: ALIGN/HOME/RUN acknowledgements, telemetry, Config responses, and other MCU-originated frames require a separate simulator or test process if those state transitions need to be exercised.

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

The log records the requested CAN interface and whether the launcher selected physical CAN or virtual CAN mode.

If startup fails, the error dialog includes the relevant launcher log path when possible.

## Environment overrides

Defaults can be overridden before launching if needed:

```text
LILY_CAN_CHANNEL     default: can0
LILY_CAN_BITRATE     default: 500000 (physical CAN only)
LILY_ROS_SETUP       default: /opt/ros/melodic/setup.bash
LILY_CATKIN_SETUP    default: ~/catkin_ws/devel/setup.bash
LILY_OPERATOR_LOG_ROOT
```

Any channel whose name starts with `vcan` is treated as a virtual-CAN request. For example, a terminal launch can use:

```bash
LILY_CAN_CHANNEL=vcan1 bash tools/operator_ui/launch_lily_operator.sh
```
