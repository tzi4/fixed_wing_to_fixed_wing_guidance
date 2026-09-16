# Third-party notices

This repository includes the third-party sources listed below or files derived
from them. The repository is licensed under GPL-3.0-or-later. Copyright and
license notices for the individual components are also retained.

## ArduPilot `plane_follow.lua`

`bumblebee/plane_follow.lua` is derived from ArduPilot's
`libraries/AP_Scripting/applets/plane_follow.lua` and contains local
modifications.

- Source: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/plane_follow.lua>
- Project: <https://github.com/ArduPilot/ardupilot>
- License: GNU General Public License v3.0 or later
- Copyright holders: ArduPilot contributors. See the upstream history for details.

The license text is in `LICENSES/ARDUPILOT-GPL-3.0.txt`.

## Intelligent Quads `iq_sim`

The following Gazebo models contain copies or modified versions of fixed-wing
model files from `Intelligent-Quads/iq_sim`:

- `bumblebee/models/emir_aircraft_temp/`
- `bumblebee/models/purple_target/`
- `detectCV_env/gazebo-plane2_model/`

The Erenimbus copy changes the camera configuration. The target copy changes
visual materials and model URIs. Many mesh files are identical to the upstream
versions.

- Source: <https://github.com/Intelligent-Quads/iq_sim>
- Reviewed upstream commit: `13e72512cd9748b12a4aa1cac15a2e34b1cdcfd6`
- License: MIT
- Copyright: Copyright (c) 2020 Intelligent-Quads

The MIT license text is in `LICENSES/IQ_SIM-MIT.txt`.

## External dependencies

ArduPilot, the Gazebo Classic plugin, ROS, Redis, MAVProxy, QGroundControl, and
Python packages are obtained from their official sources during installation.
They are not vendored here. Each dependency retains its own license. See
`bumblebee/INSTALL.md` and `requirements.txt` for pinned source versions.
