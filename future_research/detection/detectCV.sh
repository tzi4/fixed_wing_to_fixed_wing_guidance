#!/usr/bin/env bash
# ================================================================
# detectCV.sh — OpenCV + SiamRPN Target Acquisition & Tracking Launcher
# ================================================================
# Uses OpenCV HSV color filtering + SiamRPN tracker instead of YOLO.
# 3 separate windows on detect.sh (frame_publisher, detection, tracker)
# Instead, it runs in ONE window.
#
# Architecture:
#   OpenCV color detection (in place of YOLO) → SiamRPN Tracker → Redis pub
#
# Use:
#   bash detectCV.sh
#
# Requirements:
#   - ROS Noetic (source /opt/ros/noetic/setup.bash)
#   - Redis server (redis-server)
#   - Python3 packages: opencv-python, redis, cv_bridge, rospy, torch
#   - SiamRPN model: model.pth
# ================================================================

set -euo pipefail
SCRIPT_DIR="/home/tzi4/guidance"

echo "================================================================"
echo "  OpenCV + SiamRPN Target Detection & Tracking System v4"
echo "  No YOLO required — OpenCV color detection + SiamRPN tracking"
echo "================================================================"
echo ""

# --- ON CONTROLS ---
# Is Redis working?
if ! redis-cli ping > /dev/null 2>&1; then
    echo ">>> Redis not working, initializing..."
    redis-server --daemonize yes
    sleep 1
    if redis-cli ping > /dev/null 2>&1; then
        echo ">>> Redis initialized."
    else
        echo "ERROR: Failed to initialize Redis!"
        exit 1
    fi
else
    echo ">>> Redis is already working."
fi

echo ""

# --- PREPARE ROS ENVIRONMENT ---
# ROS setup (same as guidance4.sh)
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source ~/catkin_ws/devel/setup.bash 2>/dev/null || true

cd "$SCRIPT_DIR"

echo ">>> Starting OpenCV + SiamRPN Detector..."
echo ">>> Location: $SCRIPT_DIR/detectCV.py"
echo ">>> Model: $SCRIPT_DIR/model.pth"
echo ""

# Launch detector OpenCV+SiamRPN in one window
xterm -T "OpenCV+SiamRPN Detector" -geometry 120x35 -e "bash -c '\
    source /opt/ros/noetic/setup.bash 2>/dev/null; \
    source ~/catkin_ws/devel/setup.bash 2>/dev/null; \
    cd $SCRIPT_DIR; \
    python3 detectCV.py; \
    echo; \
    echo \"Process finished or stopped with Ctrl+C.\"; \
    exec bash'" &

echo ">>> OpenCV + SiamRPN Detector launched!"
echo ""
echo ">>> You can now run tzi.py or tzi2.py."
echo ">>> Redis bbox transmission will be broadcast via 'tracker_bbox' channel."
echo ""
echo "================================================================"
