# Fixed-wing optical tracking and guidance

This repository contains guidance software and a two-aircraft simulation
environment for a fixed-wing UAV to detect another fixed-wing UAV with a camera,
acquire an optical lock, and autonomously track and approach it.

The core `tzi.py` implementation and its successors were developed by
**Tarık Z. İnci**. It was the team's first complete working implementation to
combine fixed-wing optical locking, horizontal and vertical tracking, and
approach control. It provides the technical foundation for the team's visual
guidance implementations, including `goat_gimbal`.

## Getting started

Start with [`goat_gimbal_raw/`](goat_gimbal_raw/) for the original implementations
and the relationships between them:

- `tzi.py`: virtual gimbal, heading and altitude guidance, and approach control
  based on apparent target size. Speed is influenced through `TRIM_THROTTLE`.
- `tzi2.py`: a variant that sends a direct GUIDED airspeed command.
- `tzi2.1.py`: the final original variant using standard `MAV_CMD_DO_CHANGE_SPEED`.
- `goat_gimbal_reference.py`: a reference implementation using a fixed airspeed
  target.

`goat_gimbal` and `tzi.py` share the same optical guidance foundation. Their main
operational difference is the speed channel. `goat_gimbal` sends an airspeed
target, while `tzi.py` influences speed through `TRIM_THROTTLE`. The latter also
controls approach using the target bounding box size. The archived original
`goat_gimbal` implementation does not include that closed-loop approach layer.

## System architecture

1. **Detection:** receive a ROS camera frame, detect the target with YOLO or
   OpenCV, track it with SiamRPN, and publish its bounding box to the Redis
   `tracker_bbox` channel.
2. **Guidance:** compensate pixel errors for aircraft motion using a virtual
   gimbal, then generate heading, altitude, and version-dependent throttle or
   airspeed commands.
3. **Flight control:** send commands to ArduPlane over MAVLink.
4. **Simulation:** run pursuing and target aircraft in Gazebo with ArduPilot
   SITL, and monitor them through MAVProxy or QGroundControl.

## Erenimbus environment

The reproducible simulation package is under `bumblebee/`. Its pursuing aircraft
model is Erenimbus.

```bash
cd bumblebee
./start_simulation.sh              # With the GUI
./start_simulation.sh --headless   # Without the GUI
./bumblebee_guidance.sh --verify
```

The environment manages two ArduPlane SITL instances, missions, Redis mission
state, the camera and bounding-box bridge, and acceptance tools. See
[`bumblebee/README.md`](bumblebee/README.md) for details.

## Research extensions

[`future_research/`](future_research/) contains experimental range estimation
from 1 Hz target position telemetry, sensor fusion work, detection code,
simulation helpers, tests, and technical reports.

The completed part of this work uses telemetry-derived range for
**range-dependent control along the altitude axis**. Heading is still derived
from the optical image. Other control axes and integrated real-flight
validation remain under development. The archive records Tarık Z. İnci's final
handover of this research and provides a technical starting point for the team's
continuing work.

## Repository layout

| Path | Purpose |
| --- | --- |
| `goat_gimbal_raw/` | Original `tzi.py`, `tzi2.py`, and `tzi2.1.py` sources |
| `bumblebee/` | Erenimbus Gazebo and ArduPlane SITL environment |
| `future_research/` | Experimental range and telemetry fusion with supporting material |
| `detect_env/`, `detect2_env/`, `detectCV_env/` | Detection pipeline variants |
| `guidance4_env/`, `guidance9_env/` | Earlier packaged `tzi` environments |
| `libraries/` | Shared MAVLink helpers |

## Safety and reproducibility

This software is intended for research. Before real flight, validate connection
ports, camera intrinsics and mounting angles, the flight envelope, altitude
limits, failsafes, and ArduPilot command compatibility on the actual platform.
Experimental `future_research` code should not be treated as flight-ready.

## License and third-party sources

The project is published with the team's approval under the
**GNU General Public License v3.0 or later (GPL-3.0-or-later)**. Third-party
sources and licenses are documented in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

The complete license is in [`LICENSE`](LICENSE). The Bumblebee CAD and vendor
archive is excluded because redistribution permission is unavailable.
