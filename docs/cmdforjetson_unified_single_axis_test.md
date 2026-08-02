# /cmdForJetson unified single-axis test

## Purpose

Use the same ROS position-command interface for one-axis, one-leg, partial-leg,
and full eight-leg tests.

- Position input: `/cmdForJetson`
- Message: `sensor_msgs/JointState`
- Position length: always 24
- CAN RUN/POS_SET fan-out: only UI `Use=True` axes
- Retired external position topic: `/can/axis_command` is ignored by the
  production runtime

## Safety mask

The single-axis publisher sets only the target axis to a finite value. The
other 23 values are NaN. The unified StateMachine validates only `Use=True`
axes.

Therefore:

- exactly the intended axis is `Use=True`: the frame is accepted
- any unintended extra axis is `Use=True`: the NaN value is rejected and no
  position frame is sent

## Standard one-axis sequence

After the target axis is Connected, Use=True, Aligned, Homed, and normal RUN
has been accepted:

```bash
python tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction plus
```

Defaults:

- center: 0.000 rad
- amplitude: 0.020 rad
- step: 0.005 rad
- command period: 0.100 s
- peak hold: 0.500 s
- returns to center

Negative direction:

```bash
python tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction minus
```

Smaller initial test:

```bash
python tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction plus \
  --amplitude-rad 0.010 \
  --step-rad 0.002 \
  --period-sec 0.200
```

## Required UI preparation

1. Confirm the target axis is Connected.
2. Set only the target axis to Use=True.
3. Set all other axes to Use=False.
4. ALIGN the target axis.
5. SET HOME for the target axis.
6. Start normal RUN.
7. Run the publisher.
8. STOP after the test.

The publisher does not open SocketCAN and does not issue ALIGN, HOME, RUN, or
STOP.
