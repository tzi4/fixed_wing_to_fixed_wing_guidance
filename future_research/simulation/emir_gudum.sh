#!/bin/bash
set -euo pipefail

# --- AYARLAR ---
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/emir_multi_uav.world" 

# ArduPilot Modelleri ve IQ_SIM modellerinin olduğu yerleri ekliyoruz
GAZEBO_MODEL_PATH_FIXED="$HOME/ardupilot_gazebo/models:$HOME/catkin_ws/src/iq_sim/models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH_FIXED"

# --- KONTROLLER ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "HATA: Belirtilen harita dosyası bulunamadı!"
    echo "Aranan yol: $WORLD_FILE"
    exit 1
fi

echo ">>> TEMİZLİK YAPILIYOR..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

# Redis ayarını yapıyoruz (Görüntülü mod için)
echo ">>> Redis görevi ayarlanıyor: Goruntulu"
redis-cli set gorev Goruntulu || echo "UYARI: Redis çalışmıyor olabilir!"

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. GAZEBO BAŞLATMA
echo ">>> Gazebo Açılıyor..."
echo ">>> Harita: $WORLD_FILE"

xterm -T "Gazebo Environment" -e "bash -c 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH_FIXED; \
source /opt/ros/noetic/setup.bash; \
source ~/ardupilot_gazebo/devel/setup.bash; \
roslaunch iq_sim emir_multi_uav.launch verbose:=true; exec bash'" &

echo ">>> Harita yükleniyor (10sn bekleniyor)..."
sleep 10

# 2. ARDUPILOT SITL BAŞLATMA (2 Uçak)
for i in {0..1}    
do
    SYSID=$((i + 1))
    echo ">>> Uçak $SYSID (Instance $i - Port 90${i}2) Başlatılıyor..."
    
    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    # Uçak 1 için emir-gazebo-plane (kameralı/renkli model), Uçak 2 için emir-gazebo-plane2 (kamerasız model)
    MODEL_NAME="emir-gazebo-plane"
    if [ "$i" -eq 1 ]; then
        MODEL_NAME="emir-gazebo-plane2"
    fi

    xterm -T "Plane $SYSID" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
    --out=udp:127.0.0.1:14550 \
    $EXTRA_ARGS \
    --custom-location=41.101658,28.545652,0,0 \
    --mavproxy-args=\"--cmd=\\\"wp load /home/tzi4/gudum/dumduz.plan\\\"\" \
    --map --console; \
    exec bash'" &
    
    sleep 2
done

# 3. QGC
echo ">>> QGroundControl Başlatılıyor..."
xterm -T "QGC" -e "bash -c 'cd ~/Applications; ./QGroundControl.AppImage; exec bash'" &

echo ">>> SİSTEM BAŞLATILDI (Emir'in Ortamı, 2 Uçak)."
echo ">>> Hazır güdüm kodları:"
echo ">>> cd /home/tzi4/gudum/final_savasan/"
echo ">>> python3 tzi_emir.py"
echo ">>> python3 bbox_to_redis.py"
