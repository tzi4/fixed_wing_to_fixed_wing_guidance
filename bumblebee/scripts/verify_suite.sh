#!/usr/bin/env bash
set -u -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DURATION="${BUMBLEBEE_VERIFY_DURATION:-180}"
RUNS="${BUMBLEBEE_VERIFY_RUNS:-2}"
active=0

cleanup() {
  if [[ "$active" -eq 1 ]]; then
    "$ROOT_DIR/bumblebee_guidance.sh" --stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

for ((run=1; run<=RUNS; run++)); do
  echo "=== Verification run $run/$RUNS ==="
  if ! "$ROOT_DIR/bumblebee_guidance.sh" --headless; then
    echo "Running $run: system failed to initialize" >&2
    exit 1
  fi
  active=1

  # The starter loaded the straight plans and read them back; return and waypoint acceptance
  # Switch to the safe closed route now. Weathering is preserved: hunter 65 m
  # (safe_closed_route_hunter.plan), target 60 m (safe_closed_route.plan).
  if ! "$ROOT_DIR/load_plan.py" --plan "$ROOT_DIR/missions/safe_closed_route_hunter.plan" --ports 14551:1; then
    exit 1
  fi
  if ! "$ROOT_DIR/load_plan.py" --plan "$ROOT_DIR/missions/safe_closed_route.plan" --ports 14561:2; then
    exit 1
  fi
  # --delay: CONSTANT parameter of acceptance scenario; user interactive
  # independent of the default (formation.py). Between bends on a closed route
  # Since it narrows down, the low latency min can reduce the decomposition below the 20 m threshold;
  # so the acceptance test pins its own delay. The partnership of two environments
  # does not affect (only --verify binds the acceptance team).
  if ! "$ROOT_DIR/formation.py" --delay 12 --yes; then
    exit 1
  fi
  if ! "$ROOT_DIR/verify_flight.py" --duration "$DURATION" --report-dir "$ROOT_DIR/reports/run_$run"; then
    echo "Running $run did not pass acceptance requirements" >&2
    exit 1
  fi

  "$ROOT_DIR/bumblebee_guidance.sh" --stop
  active=0
done

echo "Two consecutive validation runs have passed."
