#!/usr/bin/env bash
# It checks the external environment of the main branch without starting a process.
set -u

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
IQ_SIM_DIR="${IQ_SIM_DIR:-$HOME/catkin_ws/src/iq_sim}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
VISION="${1:-none}"
failed=0

case "$VISION" in
  none|cv|yolo-v10|yolo-best) ;;
  -h|--help)
    echo "Usage: $0 [none|cv|yolo-v10|yolo-best]"
    exit 0
    ;;
  *) echo "Unknown display mode: $VISION" >&2; exit 2 ;;
esac

ok() { printf 'OK      %s\n' "$1"; }
bad() { printf 'MISSING %s\n' "$1" >&2; failed=1; }
check_command() { command -v "$1" >/dev/null 2>&1 && ok "command: $1" || bad "command: $1"; }
check_path() { [[ -e "$1" ]] && ok "file: $1" || bad "file: $1"; }

echo "main (classic TZI) setup check — vision mode: $VISION"
for command_name in python3 gzserver gzclient redis-cli redis-server mavproxy.py xterm; do
  check_command "$command_name"
done
for required_path in \
  "$ROS_SETUP" \
  "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" \
  "$IQ_SIM_DIR/worlds/airport_guidance9.world" \
  "$IQ_SIM_DIR/models/gazebo-plane/model.sdf" \
  "$IQ_SIM_DIR/models/gazebo-plane2/model.sdf" \
  "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane9-1.parm" \
  "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane9-2.parm"; do
  check_path "$required_path"
done

case "$VISION" in
  cv) check_path "$ROOT_DIR/detectCV_env/model.pth" ;;
  yolo-v10)
    check_path "$ROOT_DIR/detect_env/v10_final.pt"
    check_path "$ROOT_DIR/detect_env/model.pth"
    ;;
  yolo-best)
    check_path "$ROOT_DIR/detect2_env/best.pt"
    check_path "$ROOT_DIR/detect2_env/model.pth"
    ;;
esac

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi
if python3 - <<'PY'
import importlib
required = ('cv2', 'cv_bridge', 'numpy', 'pymavlink', 'redis', 'rospy', 'sensor_msgs')
missing = []
for name in required:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f'{name}: {exc}')
if missing:
    raise SystemExit('\n'.join(missing))
PY
then
  ok "basic Python/ROS modules"
else
  bad "basic Python/ROS modules"
fi

if [[ "$failed" -ne 0 ]]; then
  echo "CONCLUSION: there is something missing. Read branch limitations in INSTALL.md." >&2
  exit 1
fi
echo "RESULT: prerequisites for the selected scope are present."
