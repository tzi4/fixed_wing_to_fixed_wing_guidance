# Original goat_gimbal implementations

This directory preserves the team's first complete working fixed-wing visual
guidance implementations, developed by **Tarık Z. İnci**.

`tzi.py` was the first integrated implementation to acquire an optical lock on
another fixed-wing UAV using only its camera bounding box and the pursuing
aircraft's own telemetry. It tracks the target horizontally and vertically and
controls approach using its apparent image size. It forms the technical
foundation of the team's later visual guidance work.

## Implementations

| File | Role | Speed command |
| --- | --- | --- |
| `tzi.py` | Original implementation with a virtual gimbal, heading and altitude guidance, and size-based approach control | `TRIM_THROTTLE` parameter |
| `tzi2.py` | Variant with a direct airspeed target | `MAV_CMD_GUIDED_CHANGE_SPEED` (43000) |
| `tzi2.1.py` | Final original variant using the standard MAVLink speed command | `MAV_CMD_DO_CHANGE_SPEED` (178) |
| `goat_gimbal_reference.py` | Reference for comparison with the later `goat_gimbal` implementation | Fixed airspeed target |
| `trim_throttle_sender.py` | Standalone helper for testing `TRIM_THROTTLE` behavior | Parameter write |

## Relationship between tzi.py and goat_gimbal

Both implementations use a virtual gimbal to compensate bounding-box error and
send heading and altitude targets to the aircraft. Their main operational
difference is the speed channel. `goat_gimbal` sends a configured airspeed
target, while `tzi.py` influences speed through `TRIM_THROTTLE`.

`tzi.py` also feeds apparent target size into its control loop to regulate
approach and separation. The archived original `goat_gimbal` implementation
sends a fixed airspeed target and does not include that closed-loop approach
layer. Thus, `tzi.py` represents the team's first integrated implementation of
both tracking and approach control.

## Source snapshot

The source implementations were collected on 13 August 2026. Their control
logic is retained for traceability. Before flight, independently validate ports,
camera calibration, speed and altitude limits, and ArduPlane command support
on the intended platform.
