# TZI — Sabit Kanatlı İHA Optik Takip ve Güdüm Sistemi

Bu repo, bir sabit kanatlı İHA'nın başka bir sabit kanatlı İHA'yı kamera
üzerinden tespit etmesi, optik olarak kilitlemesi ve otonom biçimde takip edip
yaklaşması için geliştirilen güdüm yazılımlarını ve çift uçaklı simülasyon
ortamını içerir.

Projenin merkezindeki `tzi.py` ve devam sürümleri **Tarık Z. İnci** tarafından
geliştirilmiştir. `tzi.py`, takım tarihinde sabit kanattan sabit kanada optik
kilit, yatay/dikey takip ve yaklaşma davranışlarını tek çalışan akışta bir araya
getiren ilk tam teşekküllü koddur. Bugün kullanılan `goat_gimbal` dâhil takımın
görüntülü güdüm ailesinin ana teknik atası bu koddur.

## Nereden başlanmalı?

Tarihsel temel ve kodların birbirleriyle ilişkisi için önce
[`goat_gimbal_raw/`](goat_gimbal_raw/) klasörüne bakın:

- `tzi.py`: sanal gimbal + heading + irtifa + görüntü büyüklüğüne bağlı
  yaklaşma; hız etkisi `TRIM_THROTTLE` üzerinden uygulanır.
- `tzi2.py`: doğrudan GUIDED hava hızı komutu kullanan varyant.
- `tzi2.1.py`: standart `MAV_CMD_DO_CHANGE_SPEED` kullanan son ham varyant.
- `goat_gimbal_reference.py`: daha sonraki sabit hava hızı kullanan referans.

`goat_gimbal` ile `tzi.py` aynı optik güdüm temelini paylaşır. Başlıca
operasyonel fark, `goat_gimbal` hattının hava hızı hedefi göndermesi;
`tzi.py`nin ise bu etkiyi `TRIM_THROTTLE` parametresiyle gerçekleştirmesidir.
Ayrıca ham `goat_gimbal` sürümünde kapalı çevrim yaklaşma yokken `tzi.py`, hedef
kutusunun büyüklüğünü kullanarak yaklaşmayı da kontrol eder. Bu yönüyle
`tzi.py`, tarihsel olarak daha bütünlüklü takip çözümüdür.

## Sistem mimarisi

1. **Algılama:** ROS kamera karesi alınır; YOLO veya OpenCV ile hedef bulunur,
   SiamRPN ile izlenir ve bbox Redis `tracker_bbox` kanalına yayınlanır.
2. **Güdüm:** Piksel hatası sanal gimbal ile gövde hareketinden arındırılır;
   heading, irtifa ve sürüme göre throttle/airspeed komutları üretilir.
3. **Uçuş katmanı:** Komutlar MAVLink üzerinden ArduPlane'e gönderilir.
4. **Simülasyon:** Gazebo + ArduPilot SITL üzerinde avcı ve hedef olmak üzere
   iki sabit kanatlı araç çalıştırılır; MAVProxy/QGroundControl ile izlenir.

## Aktif Erenimbus ortamı

Yeniden üretilebilir güncel simülasyon paketi `bumblebee/` altındadır. Klasör
adı geçmişten korunmuştur; aktif avcı modeli Erenimbus'tur.

```bash
cd bumblebee
./temp_basla.sh              # GUI ile
./temp_basla.sh --headless   # GUI olmadan
./bumblebee_gudum.sh --verify
```

Ortam; iki ArduPlane SITL örneğini, görevleri, Redis görev durumunu, kamera/bbox
köprüsünü ve kabul araçlarını birlikte yönetir. Ayrıntılar için
[`bumblebee/README.md`](bumblebee/README.md) dosyasına bakın.

## Gelecek araştırmalar

[`future_research/`](future_research/) altında hedefin sunucudan gelen 1 Hz 3B
telemetrisini menzil için kullanan deneysel sürümler, füzyon çalışmaları,
algılama kodları, simülasyon yardımcıları, testler ve teknik raporlar bulunur.

Bu hattın tamamlanmış kısmı, hedef telemetrisinden çıkarılan menzilin yalnız
**irtifa eksenindeki mesafeye bağlı kontrole** uygulanmasıdır. Heading hâlâ
optik görüntüden üretilir; diğer eksenler ve bütünleşik gerçek-uçuş doğrulaması
geliştirme aşamasındadır. Bu arşiv, Tarık Z. İnci'nin araştırma hattındaki son
teslimi ve takımın gelecekte sürdürebileceği teknik başlangıç noktasıdır.

## Repo haritası

| Yol | Amaç |
| --- | --- |
| `goat_gimbal_raw/` | `tzi.py`, `tzi2.py`, `tzi2.1.py` ham tarihsel kaynakları |
| `bumblebee/` | Aktif Erenimbus Gazebo/ArduPlane SITL ortamı |
| `future_research/` | Bitmemiş mesafe/telemetri füzyonu ve destek arşivi |
| `detect_env/`, `detect2_env/`, `detectCV_env/` | Algılama zinciri varyantları |
| `gudum4_env/`, `gudum9_env/` | Önceki paketlenmiş `tzi` çalışma ortamları |
| `libraries/` | Ortak MAVLink yardımcıları |

## Güvenlik ve yeniden üretilebilirlik

Bu yazılım araştırma amaçlıdır. Gerçek uçuş öncesinde bağlantı portları, kamera
intrinsikleri/montaj açısı, uçuş zarfı, irtifa sınırları, failsafe'ler ve
ArduPilot komut uyumluluğu platform üzerinde yeniden doğrulanmalıdır. Deneysel
`future_research` kodları doğrudan uçuşa hazır kabul edilmemelidir.

## Lisans ve açık kaynak durumu

Bu proje takım onayıyla **GNU General Public License v3.0 veya sonrası
(GPL-3.0-or-later)** altında yayımlanır. Üçüncü taraf kaynakları ve lisansları
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) içinde belgelenmiştir.

Tam lisans metni [`LICENSE`](LICENSE) dosyasındadır. Dağıtım izni bulunmayan
Bumblebee CAD/üretici arşivi bu açık kaynak sürüme dahil edilmemiştir.
