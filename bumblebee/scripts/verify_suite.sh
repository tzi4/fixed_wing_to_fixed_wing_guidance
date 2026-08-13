#!/usr/bin/env bash
set -u -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DURATION="${BUMBLEBEE_VERIFY_DURATION:-180}"
RUNS="${BUMBLEBEE_VERIFY_RUNS:-2}"
active=0

cleanup() {
  if [[ "$active" -eq 1 ]]; then
    "$ROOT_DIR/bumblebee_gudum.sh" --stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

for ((run=1; run<=RUNS; run++)); do
  echo "=== Doğrulama koşusu $run/$RUNS ==="
  if ! "$ROOT_DIR/bumblebee_gudum.sh" --headless; then
    echo "Koşu $run: sistem başlatılamadı" >&2
    exit 1
  fi
  active=1

  # Başlatıcı dumduz planlarını yükleyip geri okudu; dönüş ve waypoint kabulü
  # için şimdi güvenli kapalı rotaya geç. Ayrışma korunur: avcı 65 m
  # (safe_closed_route_hunter.plan), hedef 60 m (safe_closed_route.plan).
  if ! "$ROOT_DIR/load_plan.py" --plan "$ROOT_DIR/missions/safe_closed_route_hunter.plan" --ports 14551:1; then
    exit 1
  fi
  if ! "$ROOT_DIR/load_plan.py" --plan "$ROOT_DIR/missions/safe_closed_route.plan" --ports 14561:2; then
    exit 1
  fi
  # --delay: kabul senaryosunun SABİT parametresi; kullanıcının interaktif
  # varsayılanından (formation.py) bağımsızdır. Kapalı rotada virajlar araları
  # daralttığından düşük gecikme min ayrışmayı 20 m eşiğinin altına indirebilir;
  # bu yüzden kabul testi kendi gecikmesini pinler. İki ortamın ortaklığını
  # etkilemez (yalnız --verify kabul takımını bağlar).
  if ! "$ROOT_DIR/formation.py" --delay 12 --yes; then
    exit 1
  fi
  if ! "$ROOT_DIR/verify_flight.py" --duration "$DURATION" --report-dir "$ROOT_DIR/reports/run_$run"; then
    echo "Koşu $run kabul koşullarını geçemedi" >&2
    exit 1
  fi

  "$ROOT_DIR/bumblebee_gudum.sh" --stop
  active=0
done

echo "İki ardışık doğrulama koşusu geçti."
