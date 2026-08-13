#!/usr/bin/env bash
# ================================================================
# detectCV.sh — OpenCV + SiamRPN Hedef Tespit & Takip Başlatıcı
# ================================================================
# YOLO yerine OpenCV HSV renk filtreleme + SiamRPN tracker kullanır.
# detect.sh'deki 3 ayrı pencere (frame_publisher, detection, tracker)
# yerine TEK BİR pencerede çalışır.
#
# Mimari:
#   OpenCV Renk Tespiti (YOLO yerine) → SiamRPN Tracker → Redis pub
#
# Kullanım:
#   bash detectCV.sh
#
# Gereksinimler:
#   - ROS Noetic (source /opt/ros/noetic/setup.bash)
#   - Redis sunucusu (redis-server)
#   - Python3 paketleri: opencv-python, redis, cv_bridge, rospy, torch
#   - SiamRPN model: model.pth
# ================================================================

set -euo pipefail
SCRIPT_DIR="/home/tzi4/gudum"

echo "================================================================"
echo "  OpenCV + SiamRPN Hedef Tespit & Takip Sistemi v4"
echo "  YOLO gerektirmez — OpenCV renk tespiti + SiamRPN takip"
echo "================================================================"
echo ""

# --- ÖN KONTROLLER ---
# Redis çalışıyor mu?
if ! redis-cli ping > /dev/null 2>&1; then
    echo ">>> Redis çalışmıyor, başlatılıyor..."
    redis-server --daemonize yes
    sleep 1
    if redis-cli ping > /dev/null 2>&1; then
        echo ">>> Redis başlatıldı."
    else
        echo "HATA: Redis başlatılamadı!"
        exit 1
    fi
else
    echo ">>> Redis zaten çalışıyor."
fi

echo ""

# --- ROS ORTAMINI HAZIRLA ---
# ROS setup (gudum4.sh ile aynı)
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source ~/catkin_ws/devel/setup.bash 2>/dev/null || true

cd "$SCRIPT_DIR"

echo ">>> OpenCV + SiamRPN Dedektörü başlatılıyor..."
echo ">>> Konum: $SCRIPT_DIR/detectCV.py"
echo ">>> Model: $SCRIPT_DIR/model.pth"
echo ""

# Tek pencerede OpenCV+SiamRPN dedektörünü başlat
xterm -T "OpenCV+SiamRPN Dedektoru" -geometry 120x35 -e "bash -c '\
    source /opt/ros/noetic/setup.bash 2>/dev/null; \
    source ~/catkin_ws/devel/setup.bash 2>/dev/null; \
    cd $SCRIPT_DIR; \
    python3 detectCV.py; \
    echo; \
    echo \"İşlem bitti veya Ctrl+C ile durduruldu.\"; \
    exec bash'" &

echo ">>> OpenCV + SiamRPN Dedektörü başlatıldı!"
echo ""
echo ">>> Artık tzi.py veya tzi2.py çalıştırabilirsiniz."
echo ">>> Redis 'tracker_bbox' kanalı üzerinden bbox verisi yayınlanacak."
echo ""
echo "================================================================"
