#!/usr/bin/env bash
# Clean shutdown with a single command. First pid-file based --stop; then contract
# Scan for orphaned processes still listening on ports. Only known for this package
# binaries (gzserver/gzclient/arduplane/mavproxy/QGroundControl/bbox_to_redis)
# kills; It doesn't kill processes it doesn't recognize, it just reports them.
set -u -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

PORTS_RE='^(5760|5770|9002|9003|9012|9013|14551|14553|14561)$'
KNOWN_RE='(gzserver|gzclient|arduplane|mavproxy|QGroundControl|bbox_to_redis)'

"$SCRIPT_DIR/bumblebee_guidance.sh" --stop || true
sleep 0.5

collect_port_pids() {
  ss -H -tulnp 2>/dev/null | while IFS= read -r line; do
    endpoint="$(awk '{print $5}' <<< "$line")"
    port="${endpoint##*:}"
    if [[ "$port" =~ $PORTS_RE ]]; then
      grep -oE 'pid=[0-9]+' <<< "$line" | sed 's/pid=//'
    fi
  done | sort -u
}

mapfile -t port_pids < <(collect_port_pids)

killed=()
leftover_unknown=()
for pid in "${port_pids[@]}"; do
  [[ -n "$pid" && -r "/proc/$pid/cmdline" ]] || continue
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
  if grep -qE "$KNOWN_RE" <<< "$cmd"; then
    kill -TERM "$pid" 2>/dev/null || true
    killed+=("PID $pid -> $cmd")
  else
    leftover_unknown+=("PID $pid -> $cmd")
  fi
done

if ((${#killed[@]})); then
  sleep 1
  for pid in "${port_pids[@]}"; do
    [[ -n "$pid" && -r "/proc/$pid/stat" ]] || continue
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if grep -qE "$KNOWN_RE" <<< "$cmd"; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  echo "Known orphaned processes on contract ports have been killed:"
  printf '  %s\n' "${killed[@]}"
fi

if ((${#leftover_unknown[@]})); then
  echo "WARNING: unknown processes are listening on contract ports (not killed):" >&2
  printf '  %s\n' "${leftover_unknown[@]}" >&2
fi

remaining="$(ss -H -tuln 2>/dev/null | awk '{print $5}' | sed 's/.*://' \
  | grep -E '^(5760|5770|9002|9003|9012|9013|14551|14553|14561)$' | sort -u | paste -sd, - || true)"
if [[ -n "$remaining" ]]; then
  echo "WARNING: contract ports still listened to: $remaining" >&2
  exit 1
fi
echo "Clear: no one is listening on the contract ports."
