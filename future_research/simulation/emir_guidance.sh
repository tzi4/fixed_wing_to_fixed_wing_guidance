#!/bin/bash
set -euo pipefail

# --- SETTINGS ---
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/emir_multi_uav.world" 

# We add the locations of ArduPilot Models and IQ_SIM models
GAZEBO_MODEL_PATH_FIXED="$HOME/ardupilot_gazebo/models:$HOME/catkin_ws/src/iq_sim/models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH_FIXED"

# --- CONTROLS ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: The specified map file was not found!"
    echo "Expected location: $WORLD_FILE"
    exit 1
fi

echo ">>> CLEANING IS BEING DONE..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

# We set Redis (For display mode)
echo ">>> Setting up task Redis: Display"
redis-cli set task Visual || echo "WARNING: Redis may not be working!"

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. STARTING A GAZEBO
echo ">>> Gazebo Opening..."
echo ">>> World: $WORLD_FILE"

xterm -T "Gazebo Environment" -e "bash -c 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH_FIXED; \
source /opt/ros/noetic/setup.bash; \
source ~/ardupilot_gazebo/devel/setup.bash; \
roslaunch iq_sim emir_multi_uav.launch verbose:=true; exec bash'" &

echo ">>> Loading map (waiting 10 seconds)..."
sleep 10

# 2. ARDUPILOT SITL INITIALIZATION (2 Planes)
for i in {0..1}    
do
    SYSID=$((i + 1))
    echo ">>> Initializing Aircraft $SYSID (Instance $i - Port 90${i}2)..."
    
    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    # emir-gazebo-plane for Plane 1 (with camera/color model), emir-gazebo-plane2 for Plane 2 (model without camera)
    MODEL_NAME="emir-gazebo-plane"
    if [ "$i" -eq 1 ]; then
        MODEL_NAME="emir-gazebo-plane2"
    fi

    xterm -T "Plane $SYSID" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
    --out=udp:127.0.0.1:14550 \
    $EXTRA_ARGS \
    --custom-location=41.101658,28.545652,0,0 \
    --mavproxy-args=\"--cmd=\\\"wp load /home/tzi4/guidance/straight.plan\\\"\" \
    --map --console; \
    exec bash'" &
    
    sleep 2
done

# 3. QGC
echo ">>> Starting QGroundControl..."
xterm -T "QGC" -e "bash -c 'cd ~/Applications; ./QGroundControl.AppImage; 'exec bash'" &

echo ">>> SYSTEM INITIALIZED (Emir's Environment, 2 Planes)."
echo ">>> Ready guidance codes:"
echo ">>> cd /home/tzi4/guidance/final_competition/"
echo ">>> python3 tzi_emir.py"
echo ">>> python3 bbox_to_redis.py"
