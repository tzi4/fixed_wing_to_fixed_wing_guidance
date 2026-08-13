#!/usr/bin/env bash
set -u

WORK_DIR="$1"
shift
cd -- "$WORK_DIR" || exit 1
child_pid=''

shutdown() {
  if [[ "$child_pid" =~ ^[0-9]+$ ]]; then
    kill -TERM "$child_pid" 2>/dev/null || true
    wait "$child_pid" 2>/dev/null || true
  fi
  exit 0
}
trap shutdown INT TERM

while true; do
  mavproxy.py "$@" &
  child_pid=$!
  wait "$child_pid"
  status=$?
  child_pid=''
  echo "MAVProxy bağlantısı kapandı (durum $status); yeniden bağlanılıyor..." >&2
  sleep 1
done
