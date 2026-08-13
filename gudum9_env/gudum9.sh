#!/bin/bash
set -euo pipefail

# --- AYARLAR ---
# gudum9.sh: gudum4'ün iki-uçaklı yapısı + gudum8'in airspeed parametre desteği
# EEPROM sıfırlanmaz (-w yok), her uçağa ayrı .parm dosyası yüklenir
# TECS_SYNAIRSPEED=1 aktif (parm dosyalarından)
WORLD_FILE="/home/tzi4/catkin_ws/src/iq_sim/worlds/airport_gudum9.world" 

# Parametre dosyaları (her uçak için ayrı)
PARM_FILE_1="/home/tzi4/ardupilot/Tools/autotest/default_params/gazebo-plane9-1.parm"
PARM_FILE_2="/home/tzi4/ardupilot/Tools/autotest/default_params/gazebo-plane9-2.parm"

# ArduPilot Modelleri ve IQ_SIM modellerinin olduğu yerleri ekliyoruz
# DÜZELTME: Birleşik path değişkeni — xterm içinde de kullanılacak (.bashrc override'ını engellemek için)
GAZEBO_MODEL_PATH_FIXED="$HOME/ardupilot_gazebo/models:$HOME/catkin_ws/src/iq_sim/models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH_FIXED"

# --- KONTROLLER ---
if [ ! -f "$WORLD_FILE" ]; then
    echo "HATA: Belirtilen harita dosyası bulunamadı!"
    echo "Aranan yol: $WORLD_FILE"
    exit 1
fi

for pf in "$PARM_FILE_1" "$PARM_FILE_2"; do
    if [ ! -f "$pf" ]; then
        echo "HATA: Parametre dosyası bulunamadı!"
        echo "Aranan yol: $pf"
        exit 1
    fi
done

echo ">>> TEMİZLİK YAPILIYOR..."
killall -9 gzserver gzclient mavproxy.py sim_vehicle.py python3 xterm 2>/dev/null || true

source /opt/ros/noetic/setup.bash
source ~/ardupilot_gazebo/devel/setup.bash

# 1. GAZEBO BAŞLATMA
echo ">>> Gazebo Açılıyor..."
echo ">>> Harita: $WORLD_FILE"

# verbose parametresi ile açıyoruz
xterm -T "Gazebo Environment (gudum9)" -e "bash -c 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH_FIXED; \
source /opt/ros/noetic/setup.bash; \
source ~/ardupilot_gazebo/devel/setup.bash; \
roslaunch gazebo_ros empty_world.launch world_name:=$WORLD_FILE verbose:=true; exec bash'" &

echo ">>> Harita yükleniyor (10sn bekleniyor)..."
sleep 10

# 2. ARDUPILOT SITL BAŞLATMA (2 Uçak, Airspeed Destekli)
# NOT: -N parametresi build işlemini atlar.
# NOT: -w YOK — EEPROM sıfırlanmaz (gudum4 gibi).
# NOT: --add-param-file ile her uçağa airspeed + TECS_SYNAIRSPEED parametreleri yüklenir.

# Parametre dosyaları dizisi (index = instance numarası)
PARM_FILES=("$PARM_FILE_1" "$PARM_FILE_2")

for i in {0..1}    # <<< 2 UÇAK MODU (gudum4 gibi)
do
    SYSID=$((i + 1))
    PARM_FILE="${PARM_FILES[$i]}"
    echo ">>> Uçak $SYSID (Instance $i - Port 90${i}2) Başlatılıyor..."
    echo ">>> Parametre dosyası: $PARM_FILE"
    echo ">>> Airspeed: ARSPD_TYPE=2 + TECS_SYNAIRSPEED=1"
    
    EXTRA_ARGS=""
    if [ "$i" -eq 0 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553"
    elif [ "$i" -eq 1 ]; then
        EXTRA_ARGS="--out=udp:127.0.0.1:14561"
    fi

    xterm -T "Plane $SYSID (Airspeed)" -e "bash -c 'cd ~/ardupilot; \
    Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-plane \
    -I$i \
    --sysid $SYSID \
    -N \
    --add-param-file $PARM_FILE \
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

echo ">>> SİSTEM BAŞLATILDI (2 Uçak, Airspeed Sensörlü, gudum9)."
echo ">>> ARSPD_TYPE=2 + TECS_SYNAIRSPEED=1 her iki uçakta aktif."
echo ">>> Parametre dosyaları:"
echo ">>>   Uçak 1: $PARM_FILE_1"
echo ">>>   Uçak 2: $PARM_FILE_2"
