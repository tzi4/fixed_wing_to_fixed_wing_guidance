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

for attempt in 1 2 3; do
  "$@" &
  child_pid=$!
  wait "$child_pid"
  status=$?
  child_pid=''
  if [[ "$status" -eq 0 ]]; then
    exit 0
  fi
  echo "ArduPlane beklenmedik biçimde kapandı (durum $status, deneme $attempt/3)." >&2
  if [[ "$attempt" -lt 3 ]]; then
    sleep 1
  fi
done
exit "$status"
