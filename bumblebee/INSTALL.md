# Installing Erenimbus / Bumblebee on a new computer

This guide installs the Gazebo Classic and ArduPlane SITL environment from the `erenimbus` branch on a clean computer. The active pursuer model is **Erenimbus**. Its configuration uses the `BUMBLEBEE_*` environment variables.

## 1. Verified platform

The following combination works on the reference machine:

| Component | Verified version |
| --- | --- |
| Operating system | Ubuntu 20.04.6 LTS (native Linux recommended) |
| ROS | Noetic |
| Gazebo | Gazebo Classic 11.15.1 |
| Python | 3.8.10 |
| ArduPilot | `7351a858b5940156e2957403ed3d575a4546ddbb` |
| Classic ArduPilot Gazebo add-on | `khancyr/ardupilot_gazebo@a28cab40f939a42d4845390ed5ace6d36618385c` |
| MAVProxy / pymavlink | 1.8.71 / 2.4.41 |

This environment uses the Gazebo Classic plugin names (`libArduPilotPlugin.so`, `libLiftDragPlugin.so`). The `ArduPilot/ardupilot_gazebo` project targets Gazebo Sim and is not a drop-in replacement. This environment uses the pinned Classic plugin from `khancyr`.

This is a legacy platform. Reproduce the reference setup with the versions above before attempting a migration to a newer Ubuntu release.

## 2. System packages

Install ROS Noetic Desktop Full using the official Ubuntu instructions: <https://wiki.ros.org/noetic/Installation/Ubuntu>

Then install the required packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake g++ git ccache pkg-config \
  gazebo11 libgazebo11-dev \
  ros-noetic-gazebo-ros ros-noetic-gazebo-msgs \
  ros-noetic-cv-bridge ros-noetic-sensor-msgs \
  python3-pip python3-venv python3-opencv \
  redis-server xterm iproute2 util-linux
```

The ArduPilot prerequisite installer supplies additional build dependencies. See the official Linux build instructions: <https://ardupilot.org/dev/docs/building-setup-linux.html>

## 3. Clone the repository

The repository is public and can be cloned without special access.

```bash
mkdir -p "$HOME/work"
cd "$HOME/work"
git clone --branch erenimbus \
  https://github.com/tzi4/fixed_wing_to_fixed_wing_guidance.git
cd fixed_wing_to_fixed_wing_guidance
```

## 4. Install the pinned ArduPilot version

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
cd "$HOME/ardupilot"
git checkout 7351a858b5940156e2957403ed3d575a4546ddbb
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

After the installer finishes, open a new terminal and build ArduPlane SITL:

```bash
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf plane
test -x build/sitl/bin/arduplane
```

Do not run `waf` commands with `sudo`. The launcher leaves the external ArduPilot checkout unchanged. The required base parameters are included in `bumblebee/params/gazebo-plane-base.parm`.

## 5. Build the Gazebo Classic plugin

```bash
git clone https://github.com/khancyr/ardupilot_gazebo.git \
  "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout a28cab40f939a42d4845390ed5ace6d36618385c
mkdir -p build
cd build
cmake ..
cmake --build . --parallel 4
test -f libArduPilotPlugin.so
```

The launcher loads the plugin directly from `build/`, so `sudo make install` is unnecessary. The repository includes the active models, worlds, meshes, and parameters. A separate `iq_sim` clone is unnecessary.

## 6. Python environment

Create a virtual environment with access to system packages so that ROS Python modules are available:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing_guidance"
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r bumblebee/requirements.txt
```

Activate the virtual environment in each new terminal.

## 7. Check the installation

If you used the default directories:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing_guidance/bumblebee"
./scripts/doctor.sh
```

For custom locations, set the corresponding variables:

```bash
export ARDUPILOT_DIR="/opt/ardupilot"
export ARDUPILOT_GAZEBO_DIR="/opt/ardupilot_gazebo"
export ROS_SETUP="/opt/ros/noetic/setup.bash"
./scripts/doctor.sh
```

For GUI installation, also specify the QGroundControl AppImage path:

```bash
export QGC_BIN="$HOME/Applications/QGroundControl.AppImage"
./scripts/doctor.sh --gui
```

Official QGroundControl installation instructions: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html>

## 8. First run

First check the base environment in headless mode:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing_guidance/bumblebee"
./start_simulation.sh --headless
```

The launcher starts the ROS master, Redis, Gazebo, two ArduPlane instances, MAVProxy outputs, mission plans, and the bbox bridge. Start guidance in a separate terminal:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing_guidance/bumblebee"
source ../.venv/bin/activate
python3 teva.py --camera-profile sim
```

Stop the environment:

```bash
./stop.sh
```

To operate with GUI and QGroundControl:

```bash
./start_simulation.sh
```

## 9. Acceptance test

After the first successful headless run, run two successive acceptance runs:

```bash
BUMBLEBEE_VERIFY_DURATION=180 BUMBLEBEE_VERIFY_RUNS=2 \
  ./bumblebee_guidance.sh --verify
```

Results are written under `reports/`. Both runs must pass before the installation is considered equivalent to the reference setup.

## 10. Troubleshooting

- **`libArduPilotPlugin.so` missing:** Build the pinned Classic plugin as described in step 5 and check `ARDUPILOT_GAZEBO_DIR`.
- **Missing `rospy` or `cv_bridge`:** Source `/opt/ros/noetic/setup.bash` and recreate the virtual environment with `--system-site-packages`.
- **No heartbeat:** Check that no other processes are using ports `14551`, `14553`, `14561`, `5760`, `5770`, `9002` and `9012`.
- **GUI does not start:** Verify with `--headless` first; then check `QGC_BIN`, `DISPLAY`, video card driver and AppImage execution permission.
- **Stale processes:** Use `./stop.sh`; Do not shut down processes with random `killall` commands.

After installation, return to the main [`README.md`](README.md) document for operation, port and mission details.
