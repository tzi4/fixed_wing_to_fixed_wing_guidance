#!/usr/bin/env bash
# Kurulumu süreç başlatmadan denetler. Varsayılan headless; --gui QGC/gzclient'i de kontrol eder.
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
QGC_BIN="${QGC_BIN:-$HOME/Applications/QGroundControl.AppImage}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
GUI=0
failed=0

case "${1:-}" in
  '') ;;
  --gui) GUI=1 ;;
  -h|--help) echo "Kullanım: $0 [--gui]"; exit 0 ;;
  *) echo "Bilinmeyen seçenek: $1" >&2; exit 2 ;;
esac

ok() { printf 'OK      %s\n' "$1"; }
bad() { printf 'EKSIK   %s\n' "$1" >&2; failed=1; }

check_command() {
  command -v "$1" >/dev/null 2>&1 && ok "komut: $1" || bad "komut: $1"
}

check_path() {
  [[ -e "$1" ]] && ok "dosya: $1" || bad "dosya: $1"
}

echo "Erenimbus/Bumblebee kurulum denetimi"
echo "  ARDUPILOT_DIR=$ARDUPILOT_DIR"
echo "  ARDUPILOT_GAZEBO_DIR=$ARDUPILOT_GAZEBO_DIR"
echo "  ROS_SETUP=$ROS_SETUP"

for command_name in python3 gzserver redis-cli redis-server mavproxy.py g++ pkg-config setsid flock ss; do
  check_command "$command_name"
done
if [[ "$GUI" -eq 1 ]]; then
  check_command gzclient
  [[ -x "$QGC_BIN" ]] && ok "QGC: $QGC_BIN" || bad "QGC: $QGC_BIN"
fi

for required_path in \
  "$ROS_SETUP" \
  "$ARDUPILOT_DIR/build/sitl/bin/arduplane" \
  "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" \
  "$ROOT_DIR/worlds/temp_multi_uav.world" \
  "$ROOT_DIR/models/emir_ucak_temp/model.sdf" \
  "$ROOT_DIR/models/hedef_mor/model.sdf" \
  "$ROOT_DIR/params/gazebo-plane-base.parm" \
  "$ROOT_DIR/params/temp_emir.parm" \
  "$ROOT_DIR/params/target.parm"; do
  check_path "$required_path"
done

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

if python3 - <<'PY'
import importlib
modules = ('cv2', 'cv_bridge', 'gazebo_msgs', 'matplotlib', 'numpy',
           'pymavlink', 'redis', 'rospy', 'sensor_msgs')
missing = []
for name in modules:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f'{name}: {exc}')
if missing:
    raise SystemExit('\n'.join(missing))
PY
then
  ok "Python/ROS modülleri"
else
  bad "Python/ROS modülleri"
fi

if [[ "$failed" -ne 0 ]]; then
  echo "SONUC: kurulum eksik. INSTALL.md adımlarını ve yukarıdaki yolları kontrol edin." >&2
  exit 1
fi
echo "SONUC: kurulum başlatma için hazır."
