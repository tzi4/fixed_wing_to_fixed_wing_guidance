# TZI: UAV autonomous target tracking and guidance

The core programs `tzi.py` and `tzi2.py` were developed by Tarık Z. İnci.
`tzi.py` was the team's first complete implementation combining fixed-wing
to fixed-wing optical lock, tracking, and approach control based on apparent
target size. It provided the technical foundation for the later visual
guidance programs.

This branch contains the historical simulation environment. Read INSTALL.md
for setup, required external assets, and validation. The packaged Erenimbus
environment is on the `erenimbus` branch.

The system combines camera detection, target tracking, visual guidance,
and a two-aircraft Gazebo / ArduPilot SITL environment.

## Architecture

1. Detection
   Frame publishers acquire Gazebo camera images through ROS. YOLO detects
   the target and produces bounding boxes. SiamRPN tracks the target across
   frames, and Redis forwards the output to guidance.

2. Guidance
   A virtual gimbal converts bounding boxes into angular errors. PID control
   generates heading, altitude, and speed commands for the flight controller
   through MAVLink. `tzi.py` controls speed through `TRIM_THROTTLE`.
   `tzi2.py` uses direct airspeed commands.

3. Simulation
   Gazebo provides physics and custom worlds. ArduPilot SITL runs both
   aircraft with separate parameter files. MAVProxy and QGroundControl
   provide telemetry and monitoring.

## Usage

After completing INSTALL.md:

1. Start the simulation:
   bash guidance9_env/guidance9.sh

2. Start one detection pipeline:
   bash detect2_env/detect2.sh

3. Check the connection ports before loading parameters:
   python3 set_params.py

4. Start guidance:
   PYTHONPATH="$PWD/libraries:$PYTHONPATH" python3 guidance9_env/tzi2.py

## License

The project is released with team approval under GNU General Public License
version 3.0 or later (GPL-3.0-or-later). See THIRD_PARTY_NOTICES.md for source
attribution and third-party licenses.
