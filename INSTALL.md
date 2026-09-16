# Installing the classic TZI environment on `main`

This document describes the historical `guidance4/guidance9` and YOLO/SiamRPN runtimes on the `main` branch. The recommended and better packaged environment for new installations is the `bumblebee/INSTALL.md` document on the `erenimbus` branch.

## Important status information

The `main` branch alone is not a complete distribution package. Source codes and world files are in Git; The following large or machine-specific assets have been intentionally omitted from Git:

| External asset | Expected location | Availability |
| --- | --- | --- |
| SiamRPN weight | `detect*/model.pth` | Ignored by Git. Obtain from the team archive |
| YOLO v10 weight | `detect_env/v10_final.pt` | Ignored by Git. Obtain from the team archive |
| YOLO best weight | `detect2_env/best.pt` | Ignored by Git. Obtain from the team archive |
| `gazebo-plane`, `gazebo-plane2` models | `iq_sim/models/` | Absent from this branch. Obtain from the team environment |
| `gazebo-plane9-1/2.parm` | ArduPilot `default_params/` | Absent from this branch. Obtain from the team environment |
| `gazebo-plane` frame registration | ArduPilot `vehicleinfo.py` | Requires external ArduPilot patch |

Without these files, the source code can be examined and guidance logic can be run with dummy bbox; full camera + dual plane simulation cannot be installed. A third-party model should not be downloaded without verifying the original training source and license of the weights.

## 1. Reference platform

The historical environment was developed on Ubuntu 20.04, ROS Noetic, Gazebo Classic 11 and Python 3.8. Verified source commit for ArduPilot:

```text
7351a858b5940156e2957403ed3d575a4546ddbb
```

Pinned repository and commit for the Gazebo Classic plugin:

```text
https://github.com/khancyr/ardupilot_gazebo.git
a28cab40f939a42d4845390ed5ace6d36618385c
```

This old Classic add-on is not the same product as the current Gazebo Sim add-on.

## 2. System packages

Install ROS Noetic Desktop Full with official instruction: <https://wiki.ros.org/noetic/Installation/Ubuntu>

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git ccache pkg-config \
  gazebo11 libgazebo11-dev \
  ros-noetic-gazebo-ros ros-noetic-cv-bridge ros-noetic-sensor-msgs \
  python3-pip python3-venv redis-server xterm
```

## 3. Installing the repo and Python environment

The repository is public.

```bash
mkdir -p "$HOME/work"
cd "$HOME/work"
git clone --branch main \
  https://github.com/tzi4/fixed_wing_to_fixed_wing_guidance.git
cd fixed_wing_to_fixed_wing_guidance
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

`--system-site-packages` is required for ROS modules to appear from the virtual environment.

## 4. ArduPilot and Gazebo Classic plugin

ArduPilot installation:

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
cd "$HOME/ardupilot"
git checkout 7351a858b5940156e2957403ed3d575a4546ddbb
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

After opening a new terminal:

```bash
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf plane
```

Classic Gazebo plugin:

```bash
git clone https://github.com/khancyr/ardupilot_gazebo.git \
  "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout a28cab40f939a42d4845390ed5ace6d36618385c
mkdir -p build
cd build
cmake ..
cmake --build . --parallel 4
```

General Linux/SITL installation document of ArduPilot: <https://ardupilot.org/dev/docs/building-setup-linux.html>

## 5. Place classic simulation assets

Place the files from the team archive at the following locations:

```text
$HOME/catkin_ws/src/iq_sim/models/gazebo-plane/
$HOME/catkin_ws/src/iq_sim/models/gazebo-plane2/
$HOME/catkin_ws/src/iq_sim/worlds/airport_guidance9.world
$HOME/ardupilot/Tools/autotest/default_params/gazebo-plane9-1.parm
$HOME/ardupilot/Tools/autotest/default_params/gazebo-plane9-2.parm
```

Install the repository’s world file with:

```bash
cp guidance9_env/airport_guidance9.world \
  "$HOME/catkin_ws/src/iq_sim/worlds/airport_guidance9.world"
```

The ArduPlane frame table in ArduPilot `Tools/autotest/pysim/vehicleinfo.py` must contain the following record:

```python
"gazebo-plane": {
    "waf_target": "bin/arduplane",
    "default_params_filename": "default_params/gazebo-plane.parm",
},
```

After adding the frame entry and parameter files, rebuild ArduPlane. Preserve other frame entries in the external checkout rather than replacing the whole `vehicleinfo.py` file.

## 6. Machine specific paths

`guidance4_env/guidance4.sh` and `guidance9_env/guidance9.sh` contain historical paths `/home/tzi4/...`. Before starting, edit the following values ​​at the beginning of the files into your own directories:

- `WORLD_FILE`
- `PARM_FILE_1`, `PARM_FILE_2`
- `GAZEBO_MODEL_PATH_FIXED`
- QGroundControl AppImage location

These historical launchers use broad `killall` commands. Run them only when no other Gazebo, Python, or MAVProxy workloads are active on the machine.

## 7. Install vision model weights

Place the files needed for the single pipeline you choose:

```text
OpenCV + SiamRPN: detectCV_env/model.pth
YOLO v10 + SiamRPN: detect_env/v10_final.pt and detect_env/model.pth
YOLO best + SiamRPN: detect2_env/best.pt and detect2_env/model.pth
```

`redis_helper.py` is versioned in each detection folder. Since detection launchers use their own folder as the working directory, Python opens its source files from the correct location no matter which directory the repo is cloned to.

## 8. Check the setup

Check the basic simulation scope first:

```bash
./scripts/doctor.sh none
```

Check the selected vision pipeline:

```bash
./scripts/doctor.sh cv
# or: yolo-v10 / yolo-best
```

Resolve all missing-file checks before starting the environment.

## 9. Run the environment

Terminal 1 — two-plane simulation:

```bash
bash guidance9_env/guidance9.sh
```

Terminal 2 — selected detection pipeline:

```bash
bash detectCV_env/detectCV.sh
# or detect_env/detect.sh / detect2_env/detect2.sh
```

Terminal 3 — current classic guidance variant:

```bash
PYTHONPATH="$PWD/libraries:$PYTHONPATH" python3 guidance9_env/tzi2.py
```

Before using `set_params.py`, reconcile its historical ports `5762/5772` with the launcher’s current MAVProxy outputs.

## Reproducibility limits

This branch preserves the historical sources. A complete simulation also requires external model weights and simulation assets. For a new installation, use the `erenimbus` branch and its `bumblebee/INSTALL.md` guide. Use `main` when reproducing the historical experiments.
