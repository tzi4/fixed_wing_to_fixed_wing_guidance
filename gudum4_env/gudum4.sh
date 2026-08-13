#!/bin/bash
set -euo pipefail

# --- AYARLAR ---
# DİKKAT: Bu dosya yolunun doğruluğunu terminalde 'ls' komutu ile teyit et!
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/airport_gudum4.world" 

# ArduPilot Modelleri ve IQ_SIM modellerinin olduğu yerleri ekliyoruz
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:~/catkin_ws/src/iq_sim/models

# --- KONTROLLER ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "HATA: Belirtilen harita dosyası bulunamadı!"
    echo "Aranan yol: $WORLD_FILE"
    exit 1
fi

echo ">>> TEMİZLİK YAPILIYOR..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. GAZEBO BAŞLATMA
echo ">>> Gazebo Açılıyor..."
echo ">>> Harita: $WORLD_FILE"

# verbose parametresi ile açıyoruz
xterm -T "Gazebo Environment" -e "bash -c 'roslaunch gazebo_ros empty_world.launch world_name:=$WORLD_FILE verbose:=true; exec bash'" &

echo ">>> Harita yükleniyor (10sn bekleniyor)..."
sleep 10

# 2. ARDUPILOT SITL BAŞLATMA (Sadece 2 Uçak)
# NOT: -N parametresi build işlemini atlar.

# --- DÜZENLEME BURADA YAPILDI ---
# Eğer tekrar 5 uçak istersen alttaki satırın başındaki # işaretini kaldır, bir sonrakini kapat.
# for i in {0..4}  # <<< 5 UÇAK MODU (Kapalı)

for i in {0..1}    # <<< 2 UÇAK MODU (Aktif: Sadece Instance 0 ve 1 çalışır)
do
    SYSID=$((i + 1))
    echo ">>> Uçak $SYSID (Instance $i - Port 90${i}2) Başlatılıyor..."
    
    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    xterm -T "Plane $SYSID" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
    --out=udp:127.0.0.1:14550 \
    $EXTRA_ARGS \
    --custom-location=41.101658,28.545652,0,0 \
    --map --console; \
    exec bash'" &
    
    sleep 2
done

# 3. QGC
echo ">>> QGroundControl Başlatılıyor..."
xterm -T "QGC" -e "bash -c 'cd ~/Applications; ./QGroundControl.AppImage; exec bash'" &

echo ">>> SİSTEM BAŞLATILDI (Sadece 2 Uçak, Düz Dünya Haritası)."
