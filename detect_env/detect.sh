#!/usr/bin/env bash
# This script starts three Python processes in three separate xterm windows.

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo ">>> All processes are started in the $SCRIPT_DIR folder..."

# If ROS/venv is required, remove the # sign at the beginning of this line:
# source "$HOME/catkin_ws/devel/setup.bash"

cd "$SCRIPT_DIR"

echo ">>> Initializing 'frame_publisher'..."
xterm -T "Frame Publisher" -e "bash -c 'python3 frame_publisher.py; \
echo; echo \"Process finished or stopped with Ctrl+C.\"; \
exec bash'" &

sleep 3

echo ">>> Starting 'detection'..."
xterm -T "Detection" -e "bash -c 'python3 detection_final.py; \
echo; echo \"Process finished or stopped with Ctrl+C.\"; \
exec bash'" &

sleep 3

echo ">>> Starting 'tracker'..."
xterm -T "Tracker" -e "bash -c 'python3 tracker_allstar.py; \
echo; echo \"Process finished or stopped with Ctrl+C.\"; \
exec bash'" &

echo ">>> All initialization commands have been sent."
