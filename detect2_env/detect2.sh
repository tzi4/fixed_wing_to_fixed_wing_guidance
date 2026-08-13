#!/usr/bin/env bash
# Bu betik, üç Python işlemini üç ayrı xterm penceresinde başlatır.

set -euo pipefail
SCRIPT_DIR="/home/tzi4/gudum"

echo ">>> Tüm işlemler $SCRIPT_DIR klasöründe başlatılıyor..."

# ROS / venv gerekiyorsa bu satırın başındaki # işaretini kaldırın:
# source "/home/tzi4/catkin_ws/devel/setup.bash"

cd "$SCRIPT_DIR"

echo ">>> 'frame_publisher' başlatılıyor..."
xterm -T "Frame Publisher" -e "bash -c 'python3 frame_publisher.py; \
echo; echo \"İşlem bitti veya Ctrl+C ile durduruldu.\"; \
exec bash'" &

sleep 3

echo ">>> 'detection' başlatılıyor..."
xterm -T "Detection" -e "bash -c 'python3 detection_son2.py; \
echo; echo \"İşlem bitti veya Ctrl+C ile durduruldu.\"; \
exec bash'" &

sleep 3

echo ">>> 'tracker' başlatılıyor..."
xterm -T "Tracker" -e "bash -c 'python3 tracker_allstar.py; \
echo; echo \"İşlem bitti veya Ctrl+C ile durduruldu.\"; \
exec bash'" &

echo ">>> Tüm başlatma komutları gönderildi."