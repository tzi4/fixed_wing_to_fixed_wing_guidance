# 🎯 Güdüm Kodları Füzyon Analizi & Uygulama Planı
> **Hazırlayan:** Claude Opus 4.6 (Thinking)  
> **Tarih:** 2026-06-22  
> **Kapsam:** 5 güdüm kodunun detaylı karşılaştırması ve füzyon stratejisi

5 kodun detaylıca incelenmesi, güçlü/zayıf yanlarının tespiti ve tek bir üstün koda ulaşmak için füzyon stratejisi.

---

## 1. Kod Karşılaştırma Tablosu

### 1.1 Genel Bakış

| Özellik | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | kamp_basi_emir.py |
|---|---|---|---|---|---|
| **Test durumu** | ✅ Simülasyonda çalıştı | ❌ Hiç test edilmedi | ❌ Test edilmedi | ✅ Havada test edildi | ✅ Havada test edildi |
| **Mimari** | System→tziGuidance + TestCommander | System→tziGuidance + TestCommander | System→tziGuidance + TestCommander | Monolitik (Redis+Mav+Controller) | Monolitik (Redis+Mav+Controller) |
| **Çözünürlük** | 640×480 | 640×480 | 1280×720 | 1280×720 | 1280×720 |
| **Hata biçimi** | Piksel | Piksel | Açısal (derece) | Açısal (derece) | Açısal (derece) |
| **Airspeed yöntemi** | TRIM_THROTTLE | DO_CHANGE_SPEED (178) | DO_CHANGE_SPEED (178) | DO_CHANGE_SPEED (178) — sabit | DO_CHANGE_SPEED (178) — PID |
| **CSV Log** | ❌ | ❌ | ❌ | ✅ | ✅ |
| **Coasting/Failsafe** | ❌ | ❌ | ✅ | ✅ | ✅ |

---

### 1.2 Heading Kontrolü (Detaylı)

| Parametre | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | kamp_basi_emir.py |
|---|---|---|---|---|---|
| **Kontrolcü tipi** | PD | PD | PD (açısal) | P + Rate PID | P + Rate PID |
| **Hata kaynağı** | `u_virt - center` (px) | `u_virt - center` (px) | `atan(px/fx)` (°) | `atan(px/fx)` (°) | `atan(px/fx)` (°) |
| **Kp** | 0.03125 px⁻¹ | 0.0469 px⁻¹ | 0.255 °⁻¹ | 3.0 °⁻¹ | 3.0 °⁻¹ |
| **Kd** | 0.02 | 0.02 | 0.163 | 0 | 0 |
| **Deadzone** | Yok | Yok | 0.78° | 0.78° | 0.78° |
| **Heading rate** | **40** deg/s (sabit!) | **40** deg/s (sabit!) | **40** deg/s (sabit!) | 0.85–5.0 (dinamik PID) | 0.85–**7.5** (dinamik PID) |
| **Max delta** | ±10° | ±15° | ±10° | ±35° | ±35° |
| **param1** | 1 (raw heading) | 1 (raw heading) | 1 (raw heading) | 0 (course over ground) | 0 (course over ground) |
| **Low-pass deriv** | Yok | Yok | Yok | ✅ α=0.05 | ✅ α=0.05 |

> ⚠️ **param1 farkı kritik!** tzi kodları `param1=1` (raw magnetic heading = burun yönü) kullanırken, emir kodları `param1=0` (course over ground) kullanıyor. Rüzgarlı havada COG, rüzgardan etkilenir ve burun yönünden sapabilir. Havada hangisinin daha iyi çalıştığı test edilmeli.

---

### 1.3 Altitude Kontrolü (Detaylı)

| Parametre | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | kamp_basi_emir.py |
|---|---|---|---|---|---|
| **Kontrolcü tipi** | PD | PD | PD (açısal) | PD (Kd=0.12) | PD (Kd=0.1) + Rate PID |
| **Kp** | 0.0208 px⁻¹ | 0.0417 px⁻¹ | 0.170 °⁻¹ | 0.7 °⁻¹ | 1.5 °⁻¹ |
| **Kd** | 0.01 | 0.01 | 0.0816 | 0.12 | 0.1 |
| **Max delta** | ±5 m | ±10 m | ±2 m | ±10 m | ±10 m |
| **Alt frame** | **AMSL** | **AMSL** | **AMSL** | **Relative** | **Relative** |
| **Climb rate** | 0 (default) | 0 (default) | 0 (default) | **0.8 m/s (sabit)** | **0.3–5.0 m/s (dinamik PID)** |

> ⚠️ **Altitude frame tutarsızlığı!** tzi kodları `loc_msg.alt` (AMSL) okuyup komut gönderirken, emir kodları `msg.relative_alt` (AGL) okuyor. ArduPlane'in `GUIDED_CHANGE_ALTITUDE` komutu COMMAND_INT'e çevrilirken default frame `MAV_FRAME_GLOBAL_RELATIVE_ALT` kullanır. Bu nedenle **relative alt okuyan emir yaklaşımı daha tutarlı**. tzi kodlarında AMSL gönderilmesi yükseklik sapmasına neden olabilir.

---

### 1.4 Airspeed Kontrolü (Detaylı)

| Parametre | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | kamp_basi_emir.py |
|---|---|---|---|---|---|
| **Yöntem** | **TRIM_THROTTLE** (param_set) | DO_CHANGE_SPEED | DO_CHANGE_SPEED | DO_CHANGE_SPEED **(SABİT)** | DO_CHANGE_SPEED **(PID)** |
| **PID giriş** | bbox sqrt_area (px) | bbox sqrt_area (px) | bbox angular (°) | — (sabit 20 m/s) | coverage % (EMA) |
| **Kp / Ki / Kd** | 4.7 / 0.45 / 0.6 | 0.78 / 0.075 / 0.10 | 6.377 / 0.610 / 0.814 | — | 0.2 / 0.03 / 0 |
| **Base** | 50 (throttle) | 15 m/s | 15 m/s | 20 m/s | 17 m/s |
| **Range** | 50–127 (throttle) | 10–22.8 m/s | 10–22.8 m/s | — | 14–22 m/s |
| **Slew rate** | Yok | Yok | ✅ 2.0 m/s² | — | ✅ 1.0 m/s/s |
| **GUIDED init** | Yok | Yok | ✅ (mevcut hız alınır) | — | Yok |
| **Integral anti-windup** | Asimetrik clamp | Asimetrik clamp | Asimetrik clamp | — | Band-limited + clamp |

> 📝 **TRIM_THROTTLE vs DO_CHANGE_SPEED:** TRIM_THROTTLE, TECS cruise throttle parametresini değiştirerek dolaylı hız kontrolü yapar — TECS'i bypass etmez ama aralık dar ve etkisi non-linear. `DO_CHANGE_SPEED (178)`, TECS'e doğrudan hedef hız vererek daha güvenli bir kontrol sağlar (stall koruması devam eder). tzi.py'nin TRIM_THROTTLE ile başarılı olması, muhtemelen simülasyonda rüzgar/türbülans olmamasından kaynaklanıyor.

---

## 2. Mimari Karşılaştırma

### tzi Mimarisi (tzi.py, tzi2.1.py, tzi_final.py)

```
┌─────────────┐     ┌────────────────┐     ┌─────────────────┐
│ RedisListener│────▶│ VisionProcessor│────▶│ guide_aircraft() │
│ (Thread)     │bbox │ (30 Hz Thread) │     │ PID/PD hesapla  │
└─────────────┘     └────────────────┘     └────────┬────────┘
                                                     │ update()
                    ┌────────────────┐               │
                    │ MAVLinkReader  │     ┌─────────▼──────────┐
                    │ (cache Thread) │     │  TestCommander      │
                    └────────────────┘     │  (5 Hz Thread)      │
                                           │  heading+alt+speed  │
                                           └─────────────────────┘
```

**Avantajlar:**
- Komut gönderimi (5 Hz) ve görsel işlem (30 Hz) tamamen ayrışmış
- MAVLink cache'den okuma → race condition yok
- TestCommander bbox gelmese bile son hedefleri göndermeye devam eder (inherent coasting)
- Temiz kalıtım: `System` → `tziGuidance`

### Emir Mimarisi (goat_gimbal.py, kamp_basi_emir.py)

```
┌─────────────────┐     ┌──────────────────┐
│ RedisListener    │────▶│   data_queue      │
│ (Thread)         │     │   (Queue)         │
└─────────────────┘     └────────┬──────────┘
                                  │ get()
┌─────────────────┐     ┌────────▼──────────┐
│ MavlinkManager  │◀───▶│ AutopilotController│
│ (Thread + Lock)  │     │ (Main Thread loop) │
└─────────────────┘     └───────────────────┘
```

**Avantajlar:**
- Queue-based veri akışı (producer-consumer pattern)
- CSV loglama (offline analiz için mükemmel)
- Coasting/Failsafe mekanizması açıkça kodlanmış
- Heading rate ve altitude rate dinamik PID ile kontrol

**Dezavantajlar:**
- MavlinkManager'da `recv_match` lock altında → bu, komut gönderirken telemetri okumasını bloke edebilir
- Komut gönderimi ana döngüye bağlı (10 Hz limiti var ama bbox gelmezse coasting'e düşer)

---

## 3. Kritik Bulgular

### 🔴 goat_gimbal.py'de Heading/Altitude Kapalı!

```python
# goat_gimbal.py, satır 549-551:
# self.mavlink.send_heading_target(hdg, rate)     # ← KAPALI!
# self.mavlink.send_altitude_target(alt)            # ← KAPALI!
self.mavlink.send_airspeed_target(spd)              # ← SADECE BU AKTİF
```

Bu, goat_gimbal.py'nin **havadaki testinde sadece airspeed gönderildiğini** gösteriyor. Heading ve altitude kontrolü yapılmamış. Bu "havada çalıştı" bilgisi bağlamında önemli bir detay.

### 🟢 kamp_basi_emir.py 3 Eksende Aktif

```python
# kamp_basi_emir.py, satır 668-670:
self.mavlink.send_heading_target(target_heading, heading_rate)  # ✅ AKTİF
self.mavlink.send_altitude_target(target_alt, altitude_rate)     # ✅ AKTİF
self.mavlink.send_speed_target(cmd_speed)                        # ✅ AKTİF
```

Bu, havada test edilen ve **3 eksende de aktif komut gönderen tek kod**.

### 🟡 tzi.py Heading Rate = 40 deg/s

tzi.py'deki `_send_heading` fonksiyonunda `param3=40` (heading rate) sabit olarak 40 deg/s gönderiliyor. Bu çok agresif bir dönüş hızı. Simülasyonda çalışsa bile havada tehlikeli olabilir. kamp_basi_emir'in dinamik heading rate yaklaşımı (0.85–7.5 deg/s) daha güvenli.

### 🟡 tzi_final.py'de Heading/Altitude Override

tzi_final.py TestCommander satır 181-182:
```python
hdg = 0.0   # ← OVERRIDE! PID çıkışı bypass ediliyor
alt = 40.0  # ← OVERRIDE! PID çıkışı bypass ediliyor
```

Bu, tzi_final.py'nin sadece **airspeed tuning** için kullanıldığını gösteriyor. Heading ve altitude PID'si hiç test edilmemiş.

---

## 4. Füzyon Stratejisi — Opus 4.6 Önerisi

### Önerilen Yaklaşım: **tzi Mimarisi + Emir PID Mantığı**

```
tzi.py Mimarisi ──────────────┐
(Thread Ayrımı, TestCommander) │
                                │
kamp_basi_emir.py ─────────────┼──▶ FÜZYON KODU (tzi_fusion.py)
(Havada çalışan PID, Rate PID) │
                                │
tzi_final.py ──────────────────┤
(Açısal hata, Deadzone, Slew)  │
                                │
goat_gimbal.py ────────────────┘
(CSV Loglama, LP Derivative)
```

### Her Kaynaktan Ne Alınacak?

| Kaynak Kod | Alınacak Özellik | Neden |
|---|---|---|
| **tzi.py** | Mimari yapı (System→Guidance, TestCommander thread, MAVLink reader) | Yazılımsal olarak en temiz, thread ayrımı mükemmel |
| **kamp_basi_emir.py** | Heading PID + Rate PID, Altitude PD + Rate PID, Coverage-based Speed PID, Anti-windup, Coasting/Failsafe | Havada test edilmiş, 3 eksende çalışıyor |
| **tzi_final.py** | Açısal hata hesabı (`atan`), Deadzone mekanizması, Slew rate limiti, GUIDED entry initialization | Kamera bağımsız, güvenli geçişler |
| **goat_gimbal.py** | CSV loglama (FlightLogger sınıfı), Low-pass filtered derivative | Offline analiz için vazgeçilmez |

---

## 5. TestCommander Güncellemesi

Mevcut tzi TestCommander'ı sadece `(heading, altitude, throttle/airspeed)` gönderiyor. Füzyon kodu için **heading_rate** ve **altitude_rate** de eklenecek:

```python
# Güncellenmiş TestCommander.update() imzası:
def update(self, heading_deg, heading_rate_dps, alt_m, alt_rate_mps, airspeed_ms):
    with self.lock:
        self.target_heading_deg = float(heading_deg)
        self.target_heading_rate = float(heading_rate_dps)
        self.target_alt_m = float(alt_m)
        self.target_alt_rate = float(alt_rate_mps)
        self.target_airspeed_ms = float(airspeed_ms)
        self.active = True
```

```python
# Güncellenmiş _send_heading():
def _send_heading(self, heading_deg, heading_rate_dps):
    self.master.mav.command_long_send(
        ...,
        43002,
        0,
        1,                  # param1: raw magnetic heading (tzi yaklaşımı)
        heading_deg,        # param2: hedef heading
        heading_rate_dps,   # param3: DİNAMİK rate (emir yaklaşımı)
        0, 0, 0, 0
    )
```

---

## 6. Uygulama Planı (4 Faz)

### Faz 1: Simülasyon Benchmark (1 gün)
1. `kamp_basi_emir.py`'yi simülasyonda çalıştır (3 eksende aktif)
2. `tzi.py`'yi simülasyonda çalıştır (TRIM_THROTTLE ile)
3. CSV loglardan performans metriklerini çıkar

### Faz 2: Füzyon Kodu Yazımı (2–3 gün) ✅ TAMAMLANDI
> **`tzi_fusion.py` Opus 4.6 tarafından oluşturuldu.**

### Faz 3: Havada Test (1–2 gün)
Sequential tuning ile güvenli havada doğrulama:
```
Test 1: heading=sabit, altitude=sabit → SADECE airspeed PID
Test 2: airspeed=sabit, altitude=sabit → SADECE heading PID  
Test 3: airspeed=sabit, heading=sabit → SADECE altitude PID
Test 4: heading+altitude aktif → airspeed sabit
Test 5: Üçü birlikte → TAM KONTROL
```

### Faz 4: Offline PID Tuning (1–2 gün, opsiyonel ama çok değerli)
1. **Log toplama:** Füzyon kodu ile birkaç uçuş yap, CSV logları topla
2. **System identification:** Step response analizi (komut → gerçek yanıt)
3. **Transfer fonksiyonu:** Her eksen için model fit et
4. **PID tuning:** Ziegler-Nichols / Python `control` kütüphanesi ile optimal Kp, Ki, Kd
5. **Doğrulama:** Yeni parametrelerle simülasyonda test → havada doğrula

---

## 7. Ek Fikirler & Gelişmiş Stratejiler

### 💡 Fikir 1: Dual-Mode Airspeed Kontrolü
TRIM_THROTTLE simülasyonda çalıştığına göre, bunu bir "fallback" olarak tutabiliriz:
```
if DO_CHANGE_SPEED çalışıyorsa:
    airspeed = DO_CHANGE_SPEED PID
else if airspeed yanıt vermiyorsa (> 5s stale):
    airspeed = TRIM_THROTTLE PID (tzi.py mode)
```

### 💡 Fikir 2: Adaptive Heading Rate
Hedef uzaktayken agresif, yakınken yumuşak dönüş:
- Yakın hedef (< 5°): 0.85–5 deg/s (emir yaklaşımı)
- Uzak hedef (> 15°): 10–40 deg/s (tzi yaklaşımı)

### 💡 Fikir 3: Kalman Filter ile Hedef Takibi
BBox titreşimini azaltmak ve hedef kaybolduğunda tahmin yürütmek için basit Kalman filter:
- State: `[target_x, target_y, target_vx, target_vy, target_area]`
- Coasting sırasında: Kalman predict ile tahmini hedef konumu → PID'ye besle

---

## 8. Açık Sorular

> **1. goat_gimbal havada tam olarak ne test etti?**
> Heading ve altitude komutları kapalıyken sadece airspeed mi gönderildi? Yoksa havadaki testte farklı bir versiyon mu kullanıldı?

> **2. kamp_basi_emir havadaki performansı nasıldı?**
> 3 eksende aktif çalışırken hangi sorunlar yaşandı?

> **3. param1: raw heading (1) vs course over ground (0)**
> Rüzgarlı havada hangisi tercih edilmeli?

> **4. Gerçek kamera intrinsik parametreleri nedir?**
> - goat_gimbal: fx=3045.737, fy=3045.565 (HFOV=0.415 rad)
> - tzi_final: f=4711.91
> - tzi.py: f=467.7 (640×480 için)
> - **Doğru kalibrasyon verisi hangisi?**

> **5. Altitude referansı: AMSL vs Relative?**
> ArduPlane GUIDED_CHANGE_ALTITUDE default frame `MAV_FRAME_GLOBAL_RELATIVE_ALT` → **relative önerilir.**

---

## 9. Oluşturulan Füzyon Kodu

`tzi_fusion.py` dosyası `/home/tzi4/gudum/` klasöründe oluşturulmuştur.

### Kullanılan kaynaklar (her koddan alınan):
| Kaynak | Alınan |
|---|---|
| tzi.py | Mimari, TestCommander, virtual gimbal, MAVLink reader thread |
| kamp_basi_emir.py | Heading P+Rate PID, Altitude PD+Rate PID, Coverage-based speed PID, anti-windup, coasting/failsafe |
| tzi_final.py | Açısal hata (atan), deadzone 0.78°, slew rate 1 m/s/s, GUIDED entry init |
| goat_gimbal.py | CSV FlightLogger, low-pass filtered derivative α=0.05 |

### Doğrulama:
- ✅ Python syntax kontrolü geçti
- ✅ Tüm modül importları başarılı
- ⏳ Simülasyon testi yapılmadı (Faz 3)
- ⏳ Havada test yapılmadı (Faz 3)

---

*Bu rapor ve `tzi_fusion.py` kodu Claude Opus 4.6 (Thinking) tarafından hazırlanmıştır.*
