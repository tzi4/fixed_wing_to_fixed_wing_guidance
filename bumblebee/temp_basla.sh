#!/usr/bin/env bash
# =====================================================================
# temp_basla.sh - ANA BASLATICI (Erenimbus + goruntulu gudum)
# =====================================================================
# Bu paketin TEK giris noktasidir. Avci ucak models/emir_ucak_temp
# ("Erenimbus" - Emir'in 1.5 kg, calistigi bilinen gazebo-plane turevi);
# kameraya bumblebee'nin gercek donanim spec'i tasinmistir
# (1920x1080, hfov 0.42 rad, near 0.1, 30 Hz, /webcam/image_raw).
#
# TARIHCE: bu dosya bumblebee modeline karsi A/B kontrol ortami olarak
# dogmustu. Bumblebee model.sdf'i cozulemeyen ucus davranisi nedeniyle
# Eski Bumblebee modeli terk edildi; bu Erenimbus ortamı ana ortam oldu.
#
# KULLANIM:
#   ./temp_basla.sh              # GUI (gzclient + QGC + pencereli bbox)
#   ./temp_basla.sh --headless   # GUI yok, bbox --no-display
#   ./temp_basla.sh --stop       # bu paketin baslattigi her seyi kapatir
#   BUMBLEBEE_VIDEO=1 ./temp_basla.sh   # bbox'li kamera videosu kaydet
#   BUMBLEBEE_VIDEO=1 BUMBLEBEE_VIDEO_PATH=videos/2026-08-11 ./temp_basla.sh
#
# Not: cekirdek baslatici (bumblebee_gudum.sh) ve BUMBLEBEE_* env adlari
# eski isimlerini koruyor - kod icinde tek tek degistirmek yerine bilerek
# birakildi; islev olarak Erenimbus ortamini kurarlar.
# =====================================================================
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Bu uc deger artik cekirdegin de varsayilanidir; yine de acikca yaziliyor ki
# ortamin neyi ucurdugu tek bakista gorunsun.
export BUMBLEBEE_WORLD="$SCRIPT_DIR/worlds/temp_multi_uav.world"
export BUMBLEBEE_PARAM="$SCRIPT_DIR/params/temp_emir.parm"
export BUMBLEBEE_HUNTER_MODEL="$SCRIPT_DIR/models/emir_ucak_temp/model.sdf"
# Video overlay'inde "Kod: ..." satirinin sagina yazilacak arac adi.
export BUMBLEBEE_UCAK="${BUMBLEBEE_UCAK:-Erenimbus}"

exec "$SCRIPT_DIR/bumblebee_gudum.sh" "$@"
