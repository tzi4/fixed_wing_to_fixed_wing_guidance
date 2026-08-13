# Erenimbus — Gazebo / ArduPlane SITL görüntülü güdüm ortamı

Bu klasör, `final_savasan/emir_gudum.sh` akışının (avcı SysID 1 + hedef
`emir-gazebo-plane2` SysID 2, aynı home, aynı portlar, Redis `gorev=Goruntulu`,
`tracker_bbox` kanalı) bağımsız eşleniğidir. Üst dizindeki veya `iq_sim`
altındaki dosyaları değiştirmez.

**Avcı modeli:** `models/emir_ucak_temp` — "Erenimbus", Emir'in 1,5 kg'lık,
**çalıştığı bilinen** `emir-gazebo-plane` modelinin kopyası. Kopyadaki tek
değişiklik kameradır: 1920×1080, `hfov` 0,42 rad, `near` 0,1, 30 Hz,
`/webcam/image_raw`. Parametreler `params/temp_emir.parm` (yalnız `MAV_SYSID 1`,
`TECS_SYNAIRSPEED 1`, `AHRS_WIND_MAX 1`, `ARSPD_TYPE 0`), dünya
`worlds/temp_multi_uav.world`.

> **Bumblebee modeli terk edildi (29 Temmuz 2026).** Çözülemeyen uçuş davranışı
> nedeniyle aktif avcı Erenimbus'a geçirildi. Dağıtım izni bulunmayan eski
> Bumblebee CAD/üretici paketi bu açık kaynak depoya dahil edilmemiştir; aktif
> akışın hiçbir yeri o pakete ihtiyaç duymaz.

## Hızlı kullanım

Yeni bir bilgisayarda ilk kurulum için önce [`INSTALL.md`](INSTALL.md)
rehberini izleyin. Rehber, doğrulanmış sürümleri, haricî bağımlılıkları,
ortam değişkenlerini ve kurulum denetimini içerir.

```bash
./temp_basla.sh              # GUI: gzclient + QGroundControl + pencereli bbox
./temp_basla.sh --headless   # agent-loop / headless: GUI yok, bbox --no-display
./temp_basla.sh --stop       # bu paketin başlattığı her şeyi kapatır
./kapat.sh                   # tek komutla temiz kapatma (--stop + port taraması)
./bumblebee_gudum.sh --verify   # 2 ardışık headless kabul koşusu
BUMBLEBEE_VIDEO=1 ./temp_basla.sh   # bbox çizili kamera videosu kaydet
```

`temp_basla.sh` **ana giriş noktasıdır**; çekirdek başlatıcı
`bumblebee_gudum.sh`'a `exec` ile delege eder. Çekirdek Gazebo'yu, iki ArduPlane
SITL'ini, Redis görev anahtarını, görev planlarını ve renk-tespit köprüsünü
(bbox) başlatır. `--stop` ve `kapat.sh` süreçleri yalnız bu paketin `run/pids`
süreç grubu üzerinden durdurur; `kapat.sh` ek olarak sözleşme portlarında kalan
bilinen artık süreçleri de temizler.

`--verify`, güvenli kapalı rotayı iki ayrı temiz başlatmada çalıştırır ve her
koşunun JSON/CSV raporunu `reports/` altına yazar.

Çekirdek başlatıcının adı ve `BUMBLEBEE_*` env adları **bilerek** korundu (kod
içinde tek tek değiştirmek yerine); işlev olarak Erenimbus ortamını kurarlar.
Avcı tarafı yine env ile ezilebilir — varsayılanlar artık Erenimbus'tur:

| Env | Varsayılan |
| --- | --- |
| `BUMBLEBEE_WORLD` | `worlds/temp_multi_uav.world` |
| `BUMBLEBEE_PARAM` | `params/temp_emir.parm` |
| `BUMBLEBEE_HUNTER_MODEL` | `models/emir_ucak_temp/model.sdf` |

Port sözleşmesi:

| Araç | SysID | MAVLink | Ek MAVLink | QGC | Gazebo FDM |
|---|---:|---:|---:|---:|---:|
| Avcı (Erenimbus) | 1 | 14551 | 14553 | 14550 | 9002 |
| Hedef (`emir-gazebo-plane2`) | 2 | 14561 | - | 14550 | 9012 |

İki MAVProxy akışı da QGroundControl'ün varsayılan UDP dinleme portu 14550'ye
ek bir kopya gönderir. Otomasyon portları 14551/14553/14561 değişmemiştir.
`tzi_emir.py` 14553'e bağlanır.

## Görev rotası ve ayrışma

Varsayılan rota `missions/duz_uzun.plan`'dır ve **iki uçağa da aynı dosya**
yüklenir (aynı 50 m irtifa, kuzeye ~10/50/100/200 km düz waypoint'ler). Ayrışma
yalnız **kalkış gecikmesinden** gelir (`formation.py --delay`; kullanıcı
varsayılanı ~2 s ≈ 36 m öndelik).

Plan yolları başlatıcıda **tek noktadan** tanımlıdır ve env ile ezilebilir:

- `BUMBLEBEE_HUNTER_PLAN` — avcı planı (varsayılan `missions/duz_uzun.plan`)
- `BUMBLEBEE_TARGET_PLAN` — hedef planı (varsayılan `missions/duz_uzun.plan`)

Alternatif planlarda avcı hedeften **5 m yukarıda** uçar (kısa düz rota
`dumduz_hunter.plan` 55 m / `dumduz.plan` 50 m; kapalı rota
`safe_closed_route_hunter.plan` 65 m / `safe_closed_route.plan` 60 m); +12 s
gecikmeyle ~200 m ark-boyu ayrışma üretirler. Kabul takımı (`--verify`) kapalı
rotayı ve **pinlenmiş 12 s** gecikmeyi kullanır — interaktif `formation.py`
varsayılanından (2,0 s) bağımsızdır. Fiziksel çarpışma gerçekçilik için açık.

## Renk-tespit köprüsü (bbox → tracker_bbox)

`bbox_to_redis.py`, `/webcam/image_raw`'den hedefi HSV eşikleriyle bulur ve
`[x,y,w,h,horizontal_cov,validity]` JSON'unu Redis `tracker_bbox` kanalına
yayınlar. Başlatıcı bunu otomatik başlatır (GUI'de pencereli, headless'ta
`--no-display`); `BUMBLEBEE_BBOX=0` ile kapatılır. Güdüm kodu otomatik
başlatılmaz; kullanıcı/agent elle çalıştırır.

### Hedef rengi: MOR (2026-07-28)

Hedef uçak **kırmızıdan mora** alındı (`models/hedef_mor`, dünyalarda
`model://hedef_mor`). Kırmızı HSV penceresi arka planla çakışıyordu: Gazebo
`<sky>` kubbesi ufuk bandında BGR~(127,127,255) render ediyor ve uçak yatınca
**tam kadraj genişliğinde (w~1920) sahte hedef** üretiyordu; zemin dokusu da
sıyırma açısında kırmızı-uçlu çizgi artefaktları veriyordu. Ölçüm (3 kayıt,
900 kare, 1,99e8 kromatik piksel): gökyüzü H≈112 (%54), çim H≈47 (%26), pist
H≈32 (%14), **kırmızı pencere %1,9**, **mor pencere %0,0012**.

Bu sayede dünya gerçekçi kaldı: `worlds/temp_multi_uav.world` içinde `<sky>` +
pist + çim aynen duruyor. `worlds/ab_temiz_emir.world` (gökyüzü/zemin görseli
silinmiş geçici yama) **artık gerekli değil**; yalnız kırmızı hedefli eski
koşuları yeniden üretebilmek için kıyas amacıyla duruyor.

| Ortam değişkeni | Varsayılan | Etkisi |
| --- | --- | --- |
| `BUMBLEBEE_TARGET_COLOR` | `purple` | `red` → eski kırmızı pencereye birebir dönüş |
| `BUMBLEBEE_HSV_SMIN` / `_VMIN` | renge göre | S/V alt sınırını ezer (uzakta hedef kaçıyorsa gevşetmek için) |

Mor pencere: `H 140-160, S>=120, V>=60` (Gazebo/Purple = RGB 1,0,1 → H=150).
Kırmızı pencere değişmedi: `H 0-10 + 170-180, S>=70, V>=50`.

### Overlay: "Kod: ..." satırı + uçak adı

Video/pencere overlay'inde `Kod: <çalışan script'ler>` satırının **sağına**,
kadrajın sağ kenarına hizalı olarak o an uçan aracın adı yazılır
(`... +4     [Erenimbus]`). Genişlikler `cv2.getTextSize` ile ölçülür; `Kod:`
listesi uzarsa arada en az 24 px boşluk kalacak şekilde `...` ile kırpılır.
Ad kaynağı sırayla: `BUMBLEBEE_UCAK` env → `BUMBLEBEE_HUNTER_MODEL` yolundaki
model klasörü (`models/emir_ucak_temp` → `Erenimbus`) → ikisi de yoksa ad
yazılmaz. `temp_basla.sh` bunu kendisi ayarlar. Yalnız `canvas`'a çizilir;
yayınlanan `tracker_bbox` verisine etkisi yoktur.

> `models/hedef_mor/model.config` Gazebo tarafından XML olarak ayrıştırılır;
> açıklama metnine açılı parantez yazılırsa `model://hedef_mor` çözülemez ve
> **hedef uçak hiç yüklenmez** (9012 bağlantısı kurulmaz, heartbeat gelmez).

Görev araçları:

```bash
# Varsayılan: iki uçağa da duz_uzun.plan (başlatıcı otomatik yükler)
./load_plan.py --plan missions/duz_uzun.plan --ports 14551:1
./load_plan.py --plan missions/duz_uzun.plan --ports 14561:2
./formation.py --yes                 # kullanıcı varsayılanı --delay 2.0 s
./bbox_to_redis.py --no-display      # DISPLAY yoksa otomatik headless
python3 tzi_emir.py                  # görüntülü güdüm (14553)
./verify_flight.py --duration 180 --report-dir reports/manual

# Alternatif planı env ile seç (örn. 5 m ayrışmalı kısa düz rota):
BUMBLEBEE_HUNTER_PLAN=missions/dumduz_hunter.plan \
BUMBLEBEE_TARGET_PLAN=missions/dumduz.plan ./temp_basla.sh --headless
```

## Güdüm kodları

| Dosya | Ne yapar |
| --- | --- |
| `goat_cam_offset.py` | Aktif görüntülü güdüm (kamera ofset tabanlı) |
| `tzi_emir.py` | `tracker_bbox` dinleyen güdüm (14553) |
| `plane_follow.lua` | ArduPilot Lua tarafı takip script'i |
| `old_guidance/goat_gimbal_ucak.py` | Önceki sanal-gimbal güdümü (referans) |
| `tools/gudum_geri_cozum.py` | Güdüm çıkışını uçağın `.BIN` logundan geri çözer |
| `tools/kamera_aci_testi.py` | Kamera↔otopilot açısı sabit mi, güdüm CSV'sinden |

## Kabul raporları

`verify_flight.py` şu koşulları raporlar: doğru SysID heartbeat, arm ve AUTO,
avcı için en az üç waypoint ilerleme, 12–30 m/s seyir airspeed'i, sonlu/kararlı
attitude, 200 m altında irtifa, kritik failsafe/crash/EKF mesajı olmaması ve
`/webcam/image_raw` için **1920×1080** kesintisiz akış. Ayrıca iki aracın
`GLOBAL_POSITION_INT` konumlarından anlık 3B mesafe hesaplanır; rapor alanı
`min_separation_m`, eşik ≥ 20 m (altına düşerse FAIL). `tracker_bbox` kanalına
ilk 60 s'de gelen mesaj sayısı `tracker_bbox_messages_60s` alanında yumuşak
metrik olarak raporlanır. Tam kabul için `--verify` komutundaki iki ardışık
koşunun da geçmesi gerekir.

## Veri klasörleri (git'e girmez)

| Klasör | İçerik |
| --- | --- |
| `ucus_loglari/` | Güdüm koşu çıktıları: `flight_log_guided_*.csv`, `irtifa_farkı_*.log` |
| `27_july_logs/` | 27 Temmuz gerçek uçuşları: 10 × `.BIN` + `code_logs/` |
| `july_1_logs/` | 1 Temmuz uçuşları — **farklı kart/uçak** (AHRS_TRIM_Y −2,01) |
| `new_verification_logs/` | 23 Temmuz doğrulama uçuşları |
| `reports/`, `run/`, `logs/`, `arsiv/` | Kabul raporları, süreç/PID durumu, ham loglar, yedekler |
