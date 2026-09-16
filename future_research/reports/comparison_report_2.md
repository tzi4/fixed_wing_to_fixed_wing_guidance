# Comparing `goat_gimbal.py` and `tzi_final.py`

This historical design review examines scheduling, bbox latency, and control
structure. Its recommendations require measurement and validation against
the exact archived source. In particular, sender overrides and commented-out
commands can make calculated controller outputs differ from applied commands.

## Scheduling

`tzi_final.py` separates three tasks. `TestCommander` sends the latest
setpoints at approximately 5 Hz. `_vision_processor` computes control at
30 Hz. `_mavlink_reader` receives telemetry and keeps the pymavlink message
cache available through `master.messages.get`. A temporary vision delay
therefore need not block the command sender.

In `goat_gimbal.py`, Redis and MAVLink have background readers, while the
main loop performs control calculation and command sending sequentially.
The condition `current_time - self.last_cmd_send_time >= 0.1` limits sending
to 10 Hz. Logging or data-wait delays can reduce the effective update rate.

## Bbox latency

`tzi_final.py` overwrites the protected `self.latest_bbox` with new data.
The processor consumes the freshest available sample and skips intermediate
frames when input arrives faster than processing. This limits queueing delay.

`goat_gimbal.py` uses `queue.Queue()` and `self.data_queue.get()`.
If consumption is slower than arrival, queued observations become stale.
This is a latency risk to measure, not evidence that every run oscillates.

## Heading control

`goat_gimbal.py` computes a target heading and a heading rate using
`Kp_rate` and `Kd_rate`. Larger errors request faster turns, with lower rates
near alignment. The reviewed `tzi_final.py` uses a fixed 40 deg/s rate in
`MAV_CMD_GUIDED_CHANGE_HEADING`. These choices impose different transient
limits. Their effect on tracking smoothness requires comparison under the
same conditions.

The reviewed `goat_gimbal.py` has its heading and altitude send calls
commented out. Its rate calculations therefore cannot be treated as applied
flight commands without checking the tested source version.

## Speed control

The TZI family uses apparent target size as a range-related control signal.
A small apparent target requests approach, while a large target requests a
reduction in approach speed. `tzi.py` changes `TRIM_THROTTLE` indirectly.
The archived `tzi_final.py` uses direct airspeed requests through
`MAV_CMD_DO_CHANGE_SPEED`. These are distinct sender implementations and
must not be conflated.

The reviewed `goat_gimbal.py` sends the fixed request
`self.target_airspeed = 20.0` through `MAV_CMD_DO_CHANGE_SPEED`, rather than
adjusting speed continuously from coverage.

## Integration recommendation

Combine a freshest-sample cache and independent command scheduler with an
explicitly bounded heading-rate controller. Preserve logging of observation
timestamps, calculated outputs, transmitted commands, and measured aircraft
response. Compare latency, command timing, and tracking error before claiming
an improvement in physical flight.
