# Lily hardware + Gazebo synchronization through sent CAN commands

This mode mirrors the position commands that the existing Lily CAN StateMachine has already sent to the real MCU.  It is intentionally implemented outside Lily Operator so the known-good hardware command path remains unchanged.

## Architecture

```text
JSONL Motion in normal Lily Operator
        ↓
/cmdForJetson
        ↓
existing CAN StateMachine
        ↓
0x400 + axis position CAN frames
        ├──────────────→ real MCU / hardware
        │
        └→ candump (receive-only)
              ↓
          can_position_sync_bridge.py
              ↓
       MCU-equivalent Gazebo interpolation
              ↓
       24 Gazebo joint controller topics
              ↓
            Gazebo
```

The sync bridge does **not**:

- modify `lily_operator_integrated.py`;
- modify `lily_operator_ui.py` / MotionPanel;
- modify `state_machine.py`;
- modify the MCU/CAN protocol;
- subscribe to `/cmdForJetson`;
- call a CAN transmit API.

CAN input is obtained only from `candump -L`.

## Why this preserves the existing Motion safety topology

Normal hardware operation remains:

```text
/cmdForJetson publisher:  Lily Operator Motion
/cmdForJetson subscriber: existing CAN StateMachine only
```

The Gazebo sync bridge observes the **downstream CAN frames**, not `/cmdForJetson`.  Therefore MotionPanel can keep its existing exactly-one-subscriber protection.

Do not run the Gazebo-only `/lily_gazebo_mcu_position_interpolator` at the same time.  That node subscribes directly to `/cmdForJetson` and would intentionally cause the normal Motion topology check to reject SEND.

## Installation

On `feature/monitor-csv-load`:

```bash
git checkout feature/monitor-csv-load
git pull
bash tools/operator_ui/install_desktop_launcher.sh
```

This adds:

```text
Lily Gazebo Sync Bridge
```

The bridge launcher opens a terminal so its state remains visible during hardware operation.

## Recommended startup order

1. Start the existing Gazebo environment: Gazebo + Lily model + 24 joint controllers.
2. Double-click **Lily Operator** (normal physical `can0` mode).
3. Confirm the real axes, ALIGN, HOME, and RUN exactly as in the existing hardware procedure.
4. Double-click **Lily Gazebo Sync Bridge**.
5. In Motion, LOAD / CHECK the intended JSONL.
6. SEND once from the normal Lily Operator.

The same commands that are actually emitted by StateMachine on `can0` are then mirrored into Gazebo.

Do **not** open `Lily Operator (Gazebo)` during this synchronized mode.

## Launcher guards

`launch_lily_gazebo_sync_bridge.sh` refuses to start unless:

- the selected CAN interface already exists and is UP;
- ROS master is reachable;
- the normal `/lily_operator` node is running;
- `candump` is available;
- `/lily_gazebo_mcu_position_interpolator` is not running;
- another `/lily_can_gazebo_sync_bridge` is not already running.

The launcher does not configure CAN.  The normal Lily Operator remains the owner of the existing CAN setup path.

## CAN reconstruction

StateMachine sends one position command per active axis:

```text
CAN ID   = 0x400 + axis
byte 0-3 = 00 00 00 00
byte 4-7 = float32 little-endian position_rad
```

A logical 24-axis `/cmdForJetson` command therefore appears on CAN as a short burst of individual frames.  The bridge keeps the latest command for every axis and waits for a short quiet interval after the last position frame before committing one Gazebo target.

Default:

```text
CAN burst quiet/coalesce = 0.002 s
```

Axes not included in a burst retain their previous command.  The initial logical HOME vector is zero.

## Gazebo interpolation

The bridge reuses `OnlineLinearActuatorInterpolator`, the same pure interpolation model as the existing Gazebo MCU interpolator.

Defaults:

```text
LILY_GAZEBO_INTERP_DURATION = 0.100 s
LILY_GAZEBO_UPDATE_PERIOD   = 0.002 s
LILY_GAZEBO_CAN_COALESCE_SEC = 0.002 s
```

For comparison with real hardware, set `LILY_GAZEBO_INTERP_DURATION` to the intended MCU interpolation time when that value is known.  The bridge does not READ or WRITE MCU Config automatically because this synchronization path is deliberately receive-only with respect to CAN.

## Verification during a synchronized run

ROS topology should remain similar to:

```bash
rostopic info /cmdForJetson
```

Expected:

```text
Publisher:  /lily_operator
Subscriber: /lily_operator
```

The same ROS node contains both Motion publisher and StateMachine subscriber in the integrated application.  The sync bridge must **not** appear as a `/cmdForJetson` subscriber.

Useful node check:

```bash
rosnode list | grep lily
```

Expected relevant nodes:

```text
/lily_operator
/lily_can_gazebo_sync_bridge
```

and not:

```text
/lily_gazebo_mcu_position_interpolator
```

The sync-bridge terminal logs the first reconstructed CAN target and the number of distinct axes observed.  A full 24-axis run should eventually report `observed_axes=24/24`.

## Synchronization meaning

This is command synchronization, not hard real-time state synchronization.

Both branches originate from the commands actually emitted on CAN by StateMachine, but timing differences remain due to:

- CAN transmission serialization;
- the bridge coalescing interval;
- Linux/ROS scheduling;
- real MCU execution/interpolation;
- Gazebo scheduling/interpolation.

The purpose is therefore:

> replay in Gazebo the same logical position targets that were actually sent to the real MCU, while leaving the normal Lily Operator hardware path unchanged.

If later the desired comparison changes from **sent command** to **measured real position**, that should be implemented as a separate telemetry-to-Gazebo mode using `0x500|axis` actual-position telemetry rather than changing this bridge.
