# Erenimbus/Bumblebee ortamını yeni bilgisayara kurma

Bu rehber `erenimbus` dalındaki aktif Gazebo Classic + ArduPlane SITL ortamını
temiz bir bilgisayarda kurmak içindir. Klasör ve `BUMBLEBEE_*` değişken adları
tarihsel sebeple korunmuştur; uçurulan aktif avcı modeli **Erenimbus**'tur.

## 1. Doğrulanmış platform

Referans makinede aşağıdaki kombinasyon çalışmaktadır:

| Bileşen | Doğrulanan değer |
| --- | --- |
| İşletim sistemi | Ubuntu 20.04.6 LTS (native Linux önerilir) |
| ROS | Noetic |
| Gazebo | Gazebo Classic 11.15.1 |
| Python | 3.8.10 |
| ArduPilot | `7351a858b5940156e2957403ed3d575a4546ddbb` |
| Classic ArduPilot Gazebo eklentisi | `khancyr/ardupilot_gazebo@a28cab40f939a42d4845390ed5ace6d36618385c` |
| MAVProxy / pymavlink | 1.8.71 / 2.4.41 |

Bu ortam Gazebo Classic eklenti adlarını (`libArduPilotPlugin.so`,
`libLiftDragPlugin.so`) kullanır. Yeni `ArduPilot/ardupilot_gazebo` projesi
Gazebo Sim içindir ve bu paketle doğrudan değiştirilemez. Burada eski
`khancyr` deposunun sabitlenmiş commit'i bilinçli olarak kullanılır.

ROS Noetic ve Ubuntu 20.04 artık eski bir uyumluluk hattıdır. Yeni Ubuntu'ya
taşımak ayrı bir migrasyon çalışmasıdır; ilk tekrar üretim için yukarıdaki
platform kullanılmalıdır.

## 2. Sistem paketleri

Önce ROS Noetic Desktop Full'ü resmi ROS Ubuntu kurulum yönergesiyle kurun:
<https://wiki.ros.org/noetic/Installation/Ubuntu>

Ardından gereken paketleri yükleyin:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake g++ git ccache pkg-config \
  gazebo11 libgazebo11-dev \
  ros-noetic-gazebo-ros ros-noetic-gazebo-msgs \
  ros-noetic-cv-bridge ros-noetic-sensor-msgs \
  python3-pip python3-venv python3-opencv \
  redis-server xterm iproute2 util-linux
```

ArduPilot'in desteklediği paketleri kendi kurulum aracı tamamlayacaktır.
ArduPilot'in güncel genel yönergesi:
<https://ardupilot.org/dev/docs/building-setup-linux.html>

## 3. Repoyu klonlama

Repo herkese açıktır; ek bir GitHub erişim yetkisi gerekmez.

```bash
mkdir -p "$HOME/work"
cd "$HOME/work"
git clone --branch erenimbus \
  https://github.com/tzi4/fixed_wing_to_fixed_wing.git
cd fixed_wing_to_fixed_wing
```

## 4. ArduPilot'i sabit sürümde kurma

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
cd "$HOME/ardupilot"
git checkout 7351a858b5940156e2957403ed3d575a4546ddbb
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Kurulum aracından sonra yeni terminal açın ve ArduPlane SITL'i derleyin:

```bash
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf plane
test -x build/sitl/bin/arduplane
```

`waf` komutlarını `sudo` ile çalıştırmayın. Ortam, dış ArduPilot deposuna özel
parametre veya `vehicleinfo.py` yaması yazmaz; gerekli taban parametre
`bumblebee/params/gazebo-plane-base.parm` içinde sürümlenmiştir.

## 5. Gazebo Classic ArduPilot eklentisini kurma

```bash
git clone https://github.com/khancyr/ardupilot_gazebo.git \
  "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout a28cab40f939a42d4845390ed5ace6d36618385c
mkdir -p build
cd build
cmake ..
cmake --build . --parallel "$(nproc)"
test -f libArduPilotPlugin.so
```

`sudo make install` gerekli değildir; başlatıcı eklentiyi doğrudan `build/`
altından yükler. Aktif uçak modelleri, world dosyaları, mesh'ler ve parametreler
bu repoda bulunduğu için ayrıca `iq_sim` klonlamak gerekmez.

## 6. Python ortamı

ROS Python paketlerinin görünmesi için sanal ortamı sistem paketlerini görerek
oluşturun:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing"
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r bumblebee/requirements.txt
```

Her yeni terminalde önce sanal ortamı etkinleştirin.

## 7. Kurulum denetimi

Varsayılan dizinleri kullandıysanız:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing/bumblebee"
./scripts/doctor.sh
```

Farklı dizin kullandıysanız değişkenleri açıkça verin:

```bash
export ARDUPILOT_DIR="/opt/ardupilot"
export ARDUPILOT_GAZEBO_DIR="/opt/ardupilot_gazebo"
export ROS_SETUP="/opt/ros/noetic/setup.bash"
./scripts/doctor.sh
```

GUI kurulumu için ayrıca QGroundControl AppImage yolunu belirtin:

```bash
export QGC_BIN="$HOME/Applications/QGroundControl.AppImage"
./scripts/doctor.sh --gui
```

QGroundControl kurulumu için resmi yönerge:
<https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html>

## 8. İlk çalıştırma

Önce headless modla temel ortamı doğrulayın:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing/bumblebee"
./temp_basla.sh --headless
```

Başlatıcı; ROS master, Redis, Gazebo, iki ArduPlane örneği, MAVProxy çıkışları,
görev planları ve bbox köprüsünü hazırlar. Güdüm ayrı terminalde başlatılır:

```bash
cd "$HOME/work/fixed_wing_to_fixed_wing/bumblebee"
source ../.venv/bin/activate
python3 teva.py --camera-profile sim
```

Kapatma:

```bash
./kapat.sh
```

GUI ve QGroundControl ile çalıştırmak için:

```bash
./temp_basla.sh
```

## 9. Kabul testi

İlk başarılı headless çalışmadan sonra iki ardışık kabul koşusunu çalıştırın:

```bash
BUMBLEBEE_VERIFY_DURATION=180 BUMBLEBEE_VERIFY_RUNS=2 \
  ./bumblebee_gudum.sh --verify
```

Sonuçlar `reports/` altında oluşur. Her iki koşu da geçmeden kurulum referans
ortamla eşdeğer kabul edilmemelidir.

## 10. Sık sorunlar

- **`libArduPilotPlugin.so` eksik:** 5. adımdaki Classic eklenti deposunu doğru
  commit'te derleyin ve `ARDUPILOT_GAZEBO_DIR` değerini kontrol edin.
- **`rospy` veya `cv_bridge` bulunamıyor:** `/opt/ros/noetic/setup.bash`
  dosyasını source edin; venv'i `--system-site-packages` ile yeniden oluşturun.
- **Heartbeat yok:** `14551`, `14553`, `14561`, `5760`, `5770`, `9002` ve
  `9012` portlarını başka süreçlerin kullanmadığını kontrol edin.
- **GUI açılmıyor:** Önce `--headless` ile doğrulayın; sonra `QGC_BIN`,
  `DISPLAY`, ekran kartı sürücüsü ve AppImage çalıştırma iznini kontrol edin.
- **Eski süreç kaldı:** `./kapat.sh` kullanın; süreçleri rastgele `killall`
  komutlarıyla kapatmayın.

Kurulumdan sonra operasyon, port ve görev ayrıntıları için ana
[`README.md`](README.md) belgesine dönün.
