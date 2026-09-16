#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/run"
LOG_DIR="$SCRIPT_DIR/logs"
PID_FILE="$RUN_DIR/pids"
LOCK_FILE="$RUN_DIR/launcher.lock"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
QGC_BIN="${QGC_BIN:-$HOME/Applications/QGroundControl.AppImage}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
# Fighter side (world + parameter + model.sdf to search in preflight) with env
# can be crushed. The default hunter is now ERENIMBUS (models/emir_aircraft_temp);
# The old Bumblebee model was abandoned; The active model is Erenimbus.
# Target side (target.parm, emir-gazebo-plane2) fixed.
WORLD_FILE="${BUMBLEBEE_WORLD:-$SCRIPT_DIR/worlds/temp_multi_uav.world}"
PARAM_FILE="${BUMBLEBEE_PARAM:-$SCRIPT_DIR/params/temp_emir.parm}"
HUNTER_MODEL_SDF="${BUMBLEBEE_HUNTER_MODEL:-$SCRIPT_DIR/models/emir_aircraft_temp/model.sdf}"
TARGET_PARAM_FILE="$SCRIPT_DIR/params/target.parm"
BASE_PARAM_FILE="$SCRIPT_DIR/params/gazebo-plane-base.parm"
# Mission plans are defined from ONE point; Can be crushed with env. Default: two
# The plane is also the same long_straight.plan (same 50 m; separation only from takeoff delay).
# When this file is changed in one place, both environments are affected.
HUNTER_PLAN="${BUMBLEBEE_HUNTER_PLAN:-$SCRIPT_DIR/missions/long_straight.plan}"
TARGET_PLAN="${BUMBLEBEE_TARGET_PLAN:-$SCRIPT_DIR/missions/long_straight.plan}"
HEADLESS=0
MODE=start

usage() {
  echo "Usage: $0 [--headless|--verify|--stop]"
}

if [[ $# -gt 1 ]]; then usage >&2; exit 2; fi
case "${1:-}" in
  '') ;;
  --headless) HEADLESS=1 ;;
  --verify) MODE=verify ;;
  --stop) MODE=stop ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

mkdir -p "$RUN_DIR" "$LOG_DIR" "$RUN_DIR/ros"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  if [[ "$MODE" != stop ]]; then
    echo "The launcher is being used by another process." >&2
    exit 1
  fi
fi

pid_matches() {
  local pid="$1" expected_ticks="$2" actual_ticks
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  actual_ticks="$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || true)"
  [[ "$actual_ticks" == "$expected_ticks" ]]
}

stop_owned() {
  [[ -f "$PID_FILE" ]] || { echo "Bumblebee no process log."; return 0; }
  mapfile -t records < "$PID_FILE"
  local index name pid ticks
  for ((index=${#records[@]}-1; index>=0; index--)); do
    read -r name pid ticks <<< "${records[$index]}"
    if pid_matches "$pid" "$ticks"; then
      echo "$name stopping (PID $pid)"
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  for _ in {1..80}; do
    local alive=0
    for record in "${records[@]}"; do
      read -r name pid ticks <<< "$record"
      pid_matches "$pid" "$ticks" && alive=1
    done
    [[ "$alive" -eq 0 ]] && break
    sleep 0.1
  done
  for record in "${records[@]}"; do
    read -r name pid ticks <<< "$record"
    if pid_matches "$pid" "$ticks"; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  rm -f -- "$PID_FILE"
}

if [[ "$MODE" == stop ]]; then
  stop_owned
  exit 0
fi
if [[ "$MODE" == verify ]]; then
  exec 9>&-
  exec "$SCRIPT_DIR/scripts/verify_suite.sh"
fi

if [[ -f "$PID_FILE" ]]; then
  while read -r _ pid ticks; do
    if pid_matches "$pid" "$ticks"; then
      echo "The Bumblebee system is already working. Use --stop first." >&2
      exit 1
    fi
  done < "$PID_FILE"
  rm -f -- "$PID_FILE"
fi

preflight() {
  local failed=0 command path
  for command in python3 gzserver redis-cli mavproxy.py g++ pkg-config setsid flock ss; do
    command -v "$command" >/dev/null 2>&1 || { echo "Missing command: $command" >&2; failed=1; }
  done
  for path in "$WORLD_FILE" "$PARAM_FILE" "$TARGET_PARAM_FILE" "$BASE_PARAM_FILE" "$ARDUPILOT_DIR/build/sitl/bin/arduplane" "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" "$HUNTER_MODEL_SDF" "$SCRIPT_DIR/models/purple_target/model.sdf" "$ROS_SETUP"; do
    [[ -e "$path" ]] || { echo "Missing dependency: $path" >&2; failed=1; }
  done
  if [[ "$HEADLESS" -eq 0 ]]; then
    command -v gzclient >/dev/null 2>&1 || { echo "Missing command: gzclient" >&2; failed=1; }
    [[ -x "$QGC_BIN" ]] || { echo "QGroundControl not found/cannot run: $QGC_BIN" >&2; failed=1; }
  fi
  [[ "$failed" -eq 0 ]] || return 1
  # (removed bumblebee specific plugin compilation + SDF package verification steps;
  #  Both were only required for the older Bumblebee model and are not included in the distribution.)
  local occupied
  occupied="$(ss -H -lntu | awk '{print $5}' | sed 's/.*://' | grep -E '^(5501|5511|5760|5770|9002|9003|9012|9013|14551|14553|14561)$' || true)"
  if [[ -n "$occupied" ]]; then
    echo "Required UDP ports in use: $(echo "$occupied" | paste -sd, -)" >&2
    return 1
  fi
}

source "$ROS_SETUP"
export GAZEBO_MODEL_PATH="$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models"
export GAZEBO_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
# The vehicle name to be written to the right of the "Code: ..." line in the video overlay. User
# derived from the hunter model path so that the correct name appears without doing anything;
# If BUMBLEBEE_AIRCRAFT is already issued (for example start_simulation.sh) it is left untouched.
if [[ -z "${BUMBLEBEE_AIRCRAFT:-}" ]]; then
  case "$HUNTER_MODEL_SDF" in
    */models/emir_aircraft_temp/*) export BUMBLEBEE_AIRCRAFT="Erenimbus" ;;
  esac
else
  export BUMBLEBEE_AIRCRAFT
fi

export ROS_HOME="$RUN_DIR/ros"
export ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"

start_process() {
  local name="$1" log="$2" pid ticks
  shift 2
  : > "$log"
  setsid "$@" 9>&- >> "$log" 2>&1 &
  pid=$!
  for _ in {1..20}; do
    [[ -r "/proc/$pid/stat" ]] && break
    sleep 0.05
  done
  ticks="$(awk '{print $22}' "/proc/$pid/stat")"
  echo "$name $pid $ticks" >> "$PID_FILE"
  STARTED_PID="$pid"
  STARTED_TICKS="$ticks"
  echo "$name initialized (PID $pid, log: $log)"
}

wait_command() {
  local description="$1" timeout="$2"
  shift 2
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    (( SECONDS >= deadline )) && { echo "$description timeout." >&2; return 1; }
    sleep 0.5
  done
  echo "$description is ready."
}

wait_process_command() {
  local description="$1" timeout="$2" pid="$3" ticks="$4" log="$5"
  shift 5
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "The process ended before $description started (PID $pid)." >&2
      tail -n 30 "$log" >&2 || true
      return 1
    fi
    (( SECONDS >= deadline )) && { echo "$description timeout." >&2; return 1; }
    sleep 0.5
  done
  if ! pid_matches "$pid" "$ticks"; then
    echo "Although $description appeared ready, the process terminated (PID $pid)." >&2
    tail -n 30 "$log" >&2 || true
    return 1
  fi
  echo "$description is ready."
}

models_ready() {
  local output
  output="$(rosservice call /gazebo/get_world_properties 2>/dev/null || true)"
  grep -q 'hunter' <<< "$output" && grep -q 'target' <<< "$output"
}

assert_owned_alive() {
  local name pid ticks
  while read -r name pid ticks; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "$name terminated unexpectedly (PID $pid)." >&2
      return 1
    fi
  done < "$PID_FILE"
}

trap 'echo "Initialization failed; processes for this package are being shut down." >&2; stop_owned >>dev/null 2>&1 || true' ERR INT TERM
preflight
: > "$PID_FILE"

if ! rosparam list >/dev/null 2>&1; then
  start_process roscore "$LOG_DIR/roscore.log" roscore
  wait_command "ROS master" 20 rosparam list
else
  echo "Using existing ROS master."
fi

if ! redis-cli ping 2>/dev/null | grep -q PONG; then
  start_process redis "$LOG_DIR/redis.log" redis-server --port 6379 --save '' --appendonly no
  wait_command "Redis" 15 redis-cli ping
else
  echo "The current Redis is used."
fi
# Every run starts with clean condition. Old competitor telemetry on teva.py side
# Do not mistake it for a new 1 Hz server example; Clear the previous lock and overlay state.
redis-cli del hunter_telemetry rakip_telemetri active_code >/dev/null
redis-cli mset task Visual guid False guid_lock False \
  guid_lock_progress 0.000 >/dev/null

start_process gzserver "$LOG_DIR/gzserver.log" gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD_FILE"
gzserver_pid="$STARTED_PID"
gzserver_ticks="$STARTED_TICKS"
wait_process_command "Gazebo models" 45 "$gzserver_pid" "$gzserver_ticks" "$LOG_DIR/gzserver.log" models_ready
if [[ "$HEADLESS" -eq 0 ]]; then
  start_process gzclient "$LOG_DIR/gzclient.log" gzclient --verbose
fi

if [[ "${BUMBLEBEE_GAZEBO_ONLY:-0}" == 1 ]]; then
  trap - ERR INT TERM
  echo "Only Gazebo diagnostic mode is ready."
  exit 0
fi

mkdir -p "$RUN_DIR/sitl0" "$RUN_DIR/sitl1"
bumblebee_arduplane=(
  "$ARDUPILOT_DIR/build/sitl/bin/arduplane" --model gazebo-plane --speedup 1 --sysid 1 --slave 0
  --defaults "$BASE_PARAM_FILE,$PARAM_FILE"
  --sim-address=127.0.0.1 -I0 --home 41.101658,28.545652,0,0
)
if [[ "${BUMBLEBEE_GDB:-0}" == 1 ]]; then
  bumblebee_arduplane=(gdb -batch -ex 'handle SIGFPE stop nopass' -ex run -ex 'thread apply all bt' --args "${bumblebee_arduplane[@]}")
fi
start_process ardupilot_bumblebee "$LOG_DIR/ardupilot_bumblebee.log" \
  "$SCRIPT_DIR/scripts/ardupilot_supervisor.sh" "$RUN_DIR/sitl0" \
  "${bumblebee_arduplane[@]}"

start_process ardupilot_target "$LOG_DIR/ardupilot_target.log" \
  "$SCRIPT_DIR/scripts/ardupilot_supervisor.sh" "$RUN_DIR/sitl1" \
  "$ARDUPILOT_DIR/build/sitl/bin/arduplane" --model gazebo-plane --speedup 1 --sysid 2 --slave 0 \
  --defaults "$BASE_PARAM_FILE,$TARGET_PARAM_FILE" \
  --sim-address=127.0.0.1 -I1 --home 41.101658,28.545652,0,0

start_process mavproxy_bumblebee "$LOG_DIR/mavproxy_bumblebee.log" \
  "$SCRIPT_DIR/scripts/mavproxy_supervisor.sh" "$RUN_DIR/sitl0" \
  --non-interactive --retries 30 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 \
  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14553

start_process mavproxy_target "$LOG_DIR/mavproxy_target.log" \
  "$SCRIPT_DIR/scripts/mavproxy_supervisor.sh" "$RUN_DIR/sitl1" \
  --non-interactive --retries 30 --master tcp:127.0.0.1:5770 --sitl 127.0.0.1:5511 \
  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14561

python3 "$SCRIPT_DIR/scripts/wait_heartbeat.py" --timeout 60
assert_owned_alive
# Plans from a single source (HUNTER_PLAN/TARGET_PLAN, defined above, env-crushing).
# Since load_plan.py receives a single plan file, two separate calls are made. By default
# both long_straight.plan (same as 50 m); from separation formation.py departure delay.
"$SCRIPT_DIR/load_plan.py" --plan "$HUNTER_PLAN" --ports 14551:1
"$SCRIPT_DIR/load_plan.py" --plan "$TARGET_PLAN" --ports 14561:2
assert_owned_alive

# Colour-detection bridge: camera (/webcam/image_raw) → tracker_bbox (Redis pubsub).
# Since it is pid-file managed, --stop turns it off as well. GUI with window in mode,
# --no-display in headless mode. Totally disabled with BUMBLEBEE_BBOX=0.
# BUMBLEBEE_VIDEO=1 → bbox drawn squares into videos/guidance_<date>.mp4 file
# recorded (independent of external playback, also works headless). Default OFF:
# 720p recording inflates stool fast. With BUMBLEBEE_VIDEO_FPS, the recording speed is diluted.
if [[ "${BUMBLEBEE_BBOX:-1}" != 0 ]]; then
  bbox_command=(python3 "$SCRIPT_DIR/bbox_to_redis.py")
  if [[ "$HEADLESS" -eq 1 ]]; then
    bbox_command+=(--no-display)
  fi
  if [[ "${BUMBLEBEE_VIDEO:-0}" != 0 ]]; then
    if [[ -n "${BUMBLEBEE_VIDEO_PATH:-}" ]]; then
      bbox_command+=(--record "$BUMBLEBEE_VIDEO_PATH")
    else
      bbox_command+=(--record)
    fi
    echo "Video recording on: $SCRIPT_DIR/videos/ (turned off with BUMBLEBEE_VIDEO=0)"
  fi
  start_process bbox "$LOG_DIR/bbox.log" "${bbox_command[@]}"
fi

if [[ "$HEADLESS" -eq 0 ]]; then
  start_process qgroundcontrol "$LOG_DIR/qgroundcontrol.log" "$QGC_BIN"
fi

trap - ERR INT TERM
echo "The Bumblebee system is ready. To stop: $SCRIPT_DIR/bumblebee_guidance.sh --stop"
