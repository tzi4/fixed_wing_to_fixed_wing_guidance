# 3.1 Pro guidance fusion: design analysis

This historical note proposes a combined guidance architecture from five
source implementations. It records an architectural and tuning plan rather
than completed flight validation. The archived implementation is
[`tzi_3_1_fused.py`](../guidance/tzi_3_1_fused.py).

## TZI architecture

`tzi.py`, `tzi2.1.py`, and `tzi_final.py` separate vision processing,
MAVLink reading, and `TestCommander` transmission into threads. Redis
pub/sub delivers bbox updates to a protected shared cache. This separation
reduces direct coupling between vision and command scheduling, although it
does not imply zero latency.

`tzi_final.py` uses `math.atan(pixel_error / fx)` to express tracking error
as an angle. Gains expressed in angular units are easier to transfer between
camera configurations when the intrinsics are correct. `tzi.py` changes
`TRIM_THROTTLE`, while `tzi2.1.py` and `tzi_final.py` use MAVLink command
178, `MAV_CMD_DO_CHANGE_SPEED`. Changing cruise throttle influences TECS
indirectly and must not be interpreted as bypassing TECS.

The reviewed TZI controllers primarily request heading and altitude targets
with fixed command-rate settings. Their target-loss behavior includes a
5 s threshold in `tzi_final.py`. Exact sender overrides and timeout behavior
must be checked in the particular archived source before interpreting a run.

## Emir control structure

`initial_camp_emir.py` and `goat_gimbal.py` use a thread-safe queue between the
Redis listener and the main loop. They calculate a heading-rate request for
`MAV_CMD_GUIDED_CHANGE_HEADING` and a climb/descent-rate request for
`MAV_CMD_GUIDED_CHANGE_ALTITUDE`. These limits provide a way to control how
rapidly the aircraft approaches a target heading or altitude.

The derivative uses an exponential moving average to reduce bbox noise.
Integral handling includes anti-windup resets. Target coverage provides a
range-related signal for speed control, while slew limits constrain speed
changes. `FlightLogger` records attitude, errors, and PID terms to CSV.
The reviewed `goat_gimbal.py` sender had heading and altitude calls disabled,
so a reported flight test of that source does not establish validation of
those two command paths.

## Proposed fusion

Use the TZI thread/cache structure and camera-based angular-error calculation.
Feed those errors into Emir's heading-rate and altitude-rate controllers.
Extend `TestCommander` to transmit both rate parameters and integrate the
CSV logger. Inspect sender overrides and target-loss logic as part of the
integration rather than assuming the source architecture is already safe.

## Offline tuning plan

Start with low proportional gain and zero integral and derivative gains to
collect interpretable small-command responses. Maintain an independently
safe flight condition while recording rise time, overshoot, and latency.
A controller's presence does not make a data-collection flight safe by itself.

Import the CSV into MATLAB, Simulink, or Python/SciPy. Compare
`cmd_heading_rate` with measured `yaw_deg`, accounting for the difference
between a rate input and angle output. Fit a suitable dynamic model and
inspect residuals and the tested operating range.

Evaluate candidate PID gains offline, with methods such as model-based
search or a justified Ziegler–Nichols procedure. Verify selected gains in
simulation, then in controlled flight. Offline optimization cannot establish
perfect tracking or guarantee the outcome of a later flight.

The principal integration recommendation is to retain the TZI software
separation while incorporating the rate controllers, derivative filtering,
and logging from the Emir implementation. Both computation and command
transmission must use the same units and rate conventions.
