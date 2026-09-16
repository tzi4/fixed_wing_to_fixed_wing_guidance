#!/bin/bash
set -euo pipefail

# --- SETTINGS ---
# guidance9.sh: two-plane structure of homing4 + airspeed parameter support of homing8
# EEPROM is not reset (no -w), separate .parm file is loaded for each plane
# TECS_SYNAIRSPEED=1 active (from parm files)
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/airport_guidance9.world"

# Parameter files (separate for each aircraft)
PARM_FILE_1="/home/tzi4/ardupilot/Tools/autotest/default_params/gazebo-plane9-1.parm"
PARM_FILE_2="/home/tzi4/ardupilot/Tools/autotest/default_params/gazebo-plane9-2.parm"

# We add the locations of ArduPilot Models and IQ_SIM models
# FIX: Compound path variable — to be used in xterm as well (to prevent .starter override)
GAZEBO_MODEL_PATH_FIXED="$HOME/ardupilot_gazebo/models:$HOME/catkin_ws/src/iq_sim/models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH_FIXED"

# --- CONTROLS ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: The specified map file was not found!"
    echo "Expected location: $WORLD_FILE"
    exit 1
fi

for pf in "$PARM_FILE_1" "$PARM_FILE_2"; do
    if [ ! -f "$pf" ]; then
        echo "ERROR: Parameter file not found!"
        echo "Expected location: $pf"
        exit 1
    fi
done

echo ">>> CLEANING IS BEING DONE..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. STARTING A GAZEBO
echo ">>> Gazebo Opening..."
echo ">>> World: $WORLD_FILE"

# We open it with the verbose parameter
xterm -T "Gazebo Environment (guidance9)" -e "bash -c 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH_FIXED; \
source /opt/ros/noetic/setup.bash; \
source ~/ardupilot_gazebo/devel/setup.bash; \
roslaunch gazebo_ros empty_world.launch world_name:=$WORLD_FILE verbose:=true; exec bash'" &

echo ">>> Loading map (waiting 10 seconds)..."
sleep 10

# 2. ARDUPILOT SITL LAUNCH (2 Planes, Airspeed Supported)
# NOTE: The -N parameter skips the build process.
# NOTE: -w NONE — EEPROM is not reset (like homing4).
# NOTE: --add-param-file adds airspeed + TECS_SYNAIRSPEED parameters to each plane.

# Array of parameter files (index = instance number)
PARM_FILES=("$PARM_FILE_1" "$PARM_FILE_2")

for i in {0..1}    # <<< 2 AIRPLANE MODES (like guidance4)
do
    SYSID=$((i + 1))
    PARM_FILE="${PARM_FILES[$i]}"
    echo ">>> Initializing Aircraft $SYSID (Instance $i - Port 90${i}2)..."
    echo ">>> Parameter file: $PARM_FILE"
    echo ">>> Airspeed: ARSPD_TYPE=2 + TECS_SYNAIRSPEED=1"

    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    xterm -T "Plane $SYSID (Airspeed)" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
    --add-param-file $PARM_FILE \
    --out=udp:127.0.0.1:14550 \
    $EXTRA_ARGS \
    --custom-location=41.101658,28.545652,0,0 \
    --map --console; \
    exec bash'" &

    sleep 2
done

# 3. QGC
echo ">>> Starting QGroundControl..."
xterm -T "QGC" -e "bash -c 'cd ~/Applications; ./QGroundControl.AppImage; 'exec bash'" &

echo ">>> SYSTEM INITIALIZED (2 Aircraft, Airspeed Sensor, guidance9)."
echo ">>> ARSPD_TYPE=2 + TECS_SYNAIRSPEED=1 active on both aircraft."
echo ">>> Parameter files:"
echo ">>> Plane 1: $PARM_FILE_1"
echo ">>> Plane 2: $PARM_FILE_2"
