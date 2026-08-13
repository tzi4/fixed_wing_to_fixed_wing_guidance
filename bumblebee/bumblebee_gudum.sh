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
# Avci tarafi (dunya + parametre + preflight'ta aranacak model.sdf) env ile
# ezilebilir. Varsayilan avci artik ERENIMBUS'tur (models/emir_ucak_temp);
# Eski Bumblebee modeli terk edildi; aktif model Erenimbus'tur.
# Hedef tarafi (target.parm, emir-gazebo-plane2) sabit.
WORLD_FILE="${BUMBLEBEE_WORLD:-$SCRIPT_DIR/worlds/temp_multi_uav.world}"
PARAM_FILE="${BUMBLEBEE_PARAM:-$SCRIPT_DIR/params/temp_emir.parm}"
HUNTER_MODEL_SDF="${BUMBLEBEE_HUNTER_MODEL:-$SCRIPT_DIR/models/emir_ucak_temp/model.sdf}"
TARGET_PARAM_FILE="$SCRIPT_DIR/params/target.parm"
BASE_PARAM_FILE="$SCRIPT_DIR/params/gazebo-plane-base.parm"
# Görev planları TEK noktadan tanımlanır; env ile ezilebilir. Varsayılan: iki
# uçak da aynı duz_uzun.plan (aynı 50 m; ayrışma yalnız kalkış gecikmesinden).
# Bu dosya tek yerde değiştirilince iki ortam da etkilenir.
HUNTER_PLAN="${BUMBLEBEE_HUNTER_PLAN:-$SCRIPT_DIR/missions/duz_uzun.plan}"
TARGET_PLAN="${BUMBLEBEE_TARGET_PLAN:-$SCRIPT_DIR/missions/duz_uzun.plan}"
HEADLESS=0
MODE=start

usage() {
  echo "Kullanım: $0 [--headless|--verify|--stop]"
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
    echo "Başlatıcı başka bir işlem tarafından kullanılıyor." >&2
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
  [[ -f "$PID_FILE" ]] || { echo "Bumblebee süreç kaydı yok."; return 0; }
  mapfile -t records < "$PID_FILE"
  local index name pid ticks
  for ((index=${#records[@]}-1; index>=0; index--)); do
    read -r name pid ticks <<< "${records[$index]}"
    if pid_matches "$pid" "$ticks"; then
      echo "$name durduruluyor (PID $pid)"
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
      echo "Bumblebee sistemi zaten çalışıyor. Önce --stop kullanın." >&2
      exit 1
    fi
  done < "$PID_FILE"
  rm -f -- "$PID_FILE"
fi

preflight() {
  local failed=0 command path
  for command in python3 gzserver redis-cli mavproxy.py g++ pkg-config setsid flock ss; do
    command -v "$command" >/dev/null 2>&1 || { echo "Eksik komut: $command" >&2; failed=1; }
  done
  for path in "$WORLD_FILE" "$PARAM_FILE" "$TARGET_PARAM_FILE" "$BASE_PARAM_FILE" "$ARDUPILOT_DIR/build/sitl/bin/arduplane" "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" "$HUNTER_MODEL_SDF" "$SCRIPT_DIR/models/hedef_mor/model.sdf" "$ROS_SETUP"; do
    [[ -e "$path" ]] || { echo "Eksik bağımlılık: $path" >&2; failed=1; }
  done
  if [[ "$HEADLESS" -eq 0 ]]; then
    command -v gzclient >/dev/null 2>&1 || { echo "Eksik komut: gzclient" >&2; failed=1; }
    [[ -x "$QGC_BIN" ]] || { echo "QGroundControl bulunamadı/çalıştırılamıyor: $QGC_BIN" >&2; failed=1; }
  fi
  [[ "$failed" -eq 0 ]] || return 1
  # (bumblebee'ye ozel plugin derleme + SDF paket dogrulama adimlari kaldirildi;
  #  ikisi de yalnız eski Bumblebee modeli için gerekliydi ve dağıtıma dahil değildir.)
  local occupied
  occupied="$(ss -H -lntu | awk '{print $5}' | sed 's/.*://' | grep -E '^(5501|5511|5760|5770|9002|9003|9012|9013|14551|14553|14561)$' || true)"
  if [[ -n "$occupied" ]]; then
    echo "Gerekli UDP portları kullanımda: $(echo "$occupied" | paste -sd, -)" >&2
    return 1
  fi
}

source "$ROS_SETUP"
export GAZEBO_MODEL_PATH="$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models"
export GAZEBO_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
# Video overlay'inde "Kod: ..." satirinin sagina yazilacak arac adi. Kullanici
# hicbir sey yapmadan dogru ad gorunsun diye avci model yolundan turetilir;
# BUMBLEBEE_UCAK zaten verilmisse (ornegin temp_basla.sh) ona dokunulmaz.
if [[ -z "${BUMBLEBEE_UCAK:-}" ]]; then
  case "$HUNTER_MODEL_SDF" in
    */models/emir_ucak_temp/*) export BUMBLEBEE_UCAK="Erenimbus" ;;
  esac
else
  export BUMBLEBEE_UCAK
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
  echo "$name başlatıldı (PID $pid, log: $log)"
}

wait_command() {
  local description="$1" timeout="$2"
  shift 2
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    (( SECONDS >= deadline )) && { echo "$description zaman aşımı." >&2; return 1; }
    sleep 0.5
  done
  echo "$description hazır."
}

wait_process_command() {
  local description="$1" timeout="$2" pid="$3" ticks="$4" log="$5"
  shift 5
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "$description başlamadan süreç sonlandı (PID $pid)." >&2
      tail -n 30 "$log" >&2 || true
      return 1
    fi
    (( SECONDS >= deadline )) && { echo "$description zaman aşımı." >&2; return 1; }
    sleep 0.5
  done
  if ! pid_matches "$pid" "$ticks"; then
    echo "$description hazır görünse de süreç sonlandı (PID $pid)." >&2
    tail -n 30 "$log" >&2 || true
    return 1
  fi
  echo "$description hazır."
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
      echo "$name beklenmedik biçimde sonlandı (PID $pid)." >&2
      return 1
    fi
  done < "$PID_FILE"
}

trap 'echo "Başlatma başarısız; bu pakete ait süreçler kapatılıyor." >&2; stop_owned >/dev/null 2>&1 || true' ERR INT TERM
preflight
: > "$PID_FILE"

if ! rosparam list >/dev/null 2>&1; then
  start_process roscore "$LOG_DIR/roscore.log" roscore
  wait_command "ROS master" 20 rosparam list
else
  echo "Mevcut ROS master kullanılıyor."
fi

if ! redis-cli ping 2>/dev/null | grep -q PONG; then
  start_process redis "$LOG_DIR/redis.log" redis-server --port 6379 --save '' --appendonly no
  wait_command "Redis" 15 redis-cli ping
else
  echo "Mevcut Redis kullanılıyor."
fi
# Her kosu temiz durumla baslar. Eski rakip telemetrisi teva.py tarafinda
# yeni 1 Hz sunucu ornegi sanilmasin; onceki kilit/overlay bilgisi de sizmasin.
redis-cli del avci_telemetri rakip_telemetri aktif_kod >/dev/null
redis-cli mset gorev Goruntulu guid False guid_lock False \
  guid_lock_progress 0.000 >/dev/null

start_process gzserver "$LOG_DIR/gzserver.log" gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD_FILE"
gzserver_pid="$STARTED_PID"
gzserver_ticks="$STARTED_TICKS"
wait_process_command "Gazebo modelleri" 45 "$gzserver_pid" "$gzserver_ticks" "$LOG_DIR/gzserver.log" models_ready
if [[ "$HEADLESS" -eq 0 ]]; then
  start_process gzclient "$LOG_DIR/gzclient.log" gzclient --verbose
fi

if [[ "${BUMBLEBEE_GAZEBO_ONLY:-0}" == 1 ]]; then
  trap - ERR INT TERM
  echo "Yalnız Gazebo tanı modu hazır."
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
# Planlar tek kaynaktan (HUNTER_PLAN/TARGET_PLAN, üstte tanımlı, env-ezilebilir).
# load_plan.py tek plan dosyası aldığından iki ayrı çağrı yapılır. Varsayılanda
# ikisi de duz_uzun.plan (aynı 50 m); ayrışma formation.py kalkış gecikmesinden.
"$SCRIPT_DIR/load_plan.py" --plan "$HUNTER_PLAN" --ports 14551:1
"$SCRIPT_DIR/load_plan.py" --plan "$TARGET_PLAN" --ports 14561:2
assert_owned_alive

# Renk-tespit köprüsü: kamera (/webcam/image_raw) → tracker_bbox (Redis pubsub).
# pid-file yönetimli olduğundan --stop bunu da kapatır. GUI modda pencereli,
# headless modda --no-display. BUMBLEBEE_BBOX=0 ile tümden devre dışı.
# BUMBLEBEE_VIDEO=1 → bbox çizili kareler videos/gudum_<tarih>.mp4 dosyasına
# kaydedilir (display'den bağımsız, headless'ta da çalışır). Varsayılan KAPALI:
# 720p kayıt diski hızlı şişirir. BUMBLEBEE_VIDEO_FPS ile kayıt hızı seyreltilir.
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
    echo "Video kaydı açık: $SCRIPT_DIR/videos/ (BUMBLEBEE_VIDEO=0 ile kapatılır)"
  fi
  start_process bbox "$LOG_DIR/bbox.log" "${bbox_command[@]}"
fi

if [[ "$HEADLESS" -eq 0 ]]; then
  start_process qgroundcontrol "$LOG_DIR/qgroundcontrol.log" "$QGC_BIN"
fi

trap - ERR INT TERM
echo "Bumblebee sistemi hazır. Durdurmak için: $SCRIPT_DIR/bumblebee_gudum.sh --stop"
