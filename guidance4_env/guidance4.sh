#!/bin/bash
set -euo pipefail

# --- SETTINGS ---
# ATTENTION: Verify the accuracy of this file path with the 'ls' command in the terminal!
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/airport_guidance4.world"

# We add the locations of ArduPilot Models and IQ_SIM models
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:~/catkin_ws/src/iq_sim/models

# --- CONTROLS ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: The specified map file was not found!"
    echo "Expected location: $WORLD_FILE"
    exit 1
fi

echo ">>> CLEANING IS BEING DONE..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. STARTING A GAZEBO
echo ">>> Gazebo Opening..."
echo ">>> World: $WORLD_FILE"

# We open it with the verbose parameter
xterm -T "Gazebo Environment" -e "bash -c 'roslaunch gazebo_ros empty_world.launch world_name:=$WORLD_FILE verbose:=true; 'exec bash'" &

echo ">>> Loading map (waiting 10 seconds)..."
sleep 10

# 2. ARDUPILOT SITL INITIALIZATION (Only 2 Airplanes)
# NOTE: The -N parameter skips the build process.

# --- EDITED HERE ---
# If you want 5 planes again, remove the # sign at the beginning of the line below and close the next one.
# for i in {0..4} # <<< 5 AIRPLANE MODE (Off)

for i in {0..1}    # <<< 2 AIRPLANE MODES (Active: Only Instance 0 and 1 work)
do
    SYSID=$((i + 1))
    echo ">>> Initializing Aircraft $SYSID (Instance $i - Port 90${i}2)..."

    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    xterm -T "Plane $SYSID" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
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

echo ">>> SYSTEM INITIALIZED (Only 2 Planes, Flat World Map)."
