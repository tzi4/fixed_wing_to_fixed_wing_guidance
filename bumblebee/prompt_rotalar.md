# Görev: Görüntülü güdüm deney ortamı — senaryo üretimi, skorlama ve hız kontrolü altyapısı

Bu prompt, YENİ bir oturumda çalışacak ajana verilecektir. Amaç kontrolcünün
kendisini yazmak DEĞİL; kontrolcünün üzerinde güvenle deney yapılabilecek,
hile yapılamayan, ezberlenemeyen DENEY ALTYAPISINI kurmaktır. Kontrolcü
geliştirme (hız kontrolcüsü dahil) bu altyapı hazır olduktan sonra ayrı
deneylerle yürüyecek.

Temel ilke (Karpathy autoresearch düzeni): araştırmacı ajan yalnızca
kontrolcü dosyalarını değiştirir; senaryo üreteci, test koşucusu, skorlayıcı
ve güvenlik izleyicisi DEĞİŞTİRİLEMEZ katmandır. Hakem ile araştırmacı aynı
ajan olmamalıdır.

---

## 1. Ortam gerçekleri (2026-07-25 itibarıyla, ölçülmüş durum)

- Çalışma dizini `/home/tzi4/gudum/bumblebee`; Türkçe çalış/raporla.
- Başlatıcı: `./bumblebee_gudum.sh --headless` (GUI için `basla.sh`); tam
  temizlik `./kapat.sh`. Portlar tek ortama izin verir → testler SERİ, her
  testten önce/sonra temiz durum. Plan seçimi yalnız
  `BUMBLEBEE_HUNTER_PLAN`/`BUMBLEBEE_TARGET_PLAN` env değişkenleriyle.
- Kalkış düzeni: `formation.py --yes [--delay N]` — mantığı DOĞRULANDI
  (hedef önce, gecikme doğru yerde). Varsayılan delay 2.0 kullanıcının
  değeridir; senaryo koşucusu kendi delay'ini parametre olarak geçebilir.
- Avcı modeli: `model://bumblebee` — gerçek 12.0 kg çift-motor model, gerçek
  autotune parametreleri, 7 gerçek uçuş loguna karşı doğrulanmış (seyir
  throttle 0.01-0.04 içinde, roll-rate rise 0.24-0.26 s ≈ sim 0.26 s).
  `tools/validate_package.py` HER değişiklikten sonra exit 0 kalmalı.
  Model fiziğine (kütle/atalet/LiftDrag/collision/parm tune) DOKUNMA.
- Kamera: burun önünde (0.68, 0, 0), 1280×720 @30 Hz, **FOV 0.27 rad — ÇOK
  DAR: yatay ±7.74°, dikey ±4.37°**. Görüntü `/webcam/image_raw`.
- Tespit köprüsü `bbox_to_redis.py`: kırmızı hedefi bulunca Redis
  `tracker_bbox` kanalına `[x, y, w, h, yatay_kapsama_%, validity]` yayınlar.
  Publish, pencere/display'den BAĞIMSIZDIR (GUI/headless farkı yalnız
  görselleştirme). **Tespit yokken mesaj YAYINLANMAZ** — mesaj yokluğu
  "hedef kayıp" demektir; skorlayıcı bunu açıkça ele almalı.
- Mevcut güdüm: `tzi_emir.py` (MAVLink 14553) — yalnız heading + altitude
  komutluyor. Hız ekseni HİÇ yok; bu altyapının birinci müşterisi hız
  kontrolcüsü olacak.
- İtki eklentisinde rölanti ölü bandı: komut edilen gaz ~%18'in altındayken
  itki SIFIR (motor-kapalı süzülme gibi davranır). Seyir ~%57-70 bandında
  etkisi yok; ama düşük gazlı alçalma senaryoları tasarlanırken bilinmeli.
- Bu makinede `jq` YOK — JSON işlemek için python3 kullan.
- Git: `loop_testing` dalında çalışılıyor; commit+push serbest, `main`'e
  asla push/merge/force yok; kurallar `github_hakkinda.md`'de. Değiştirdiğin
  mevcut dosyaları önce `arsiv/yedek_onceki_5/` altına yedekle.
- Yazma izni: yeni oturumlarda Write/Edit doğrudan çalışır; engellenirse
  Bash+python exact-string-replace/heredoc yöntemi kullanılabilir.

## 2. HIZ GERÇEKLERİ — hız kontrolcüsü tasarımını belirleyen ölçümler

Bu bölüm bu ortamda yapılmış ölçümlere dayanır (hız kampanyası verisi:
`reports/speed_tests/`, 2026-07-25); varsayım değildir. NOT: Daha eski
"SITL göstergesi %22 düşük okur, ×1.22 dönüşümü gerekir" bulgusu YANLIŞ
teşhisti — o davranış hız döngüsünün hiç kapalı olmamasının artefaktıydı
(madde 1). Eski notlarda ×1.22 görürsen yok say.

1. **Ana anahtar: `TECS_SYNAIRSPEED`.** İki uçakta da `ARSPD_TYPE 0`
   (gerçek pitot yok; 1 yapmak SITL'de PANIC ile çöker). SYNAIRSPEED 0
   iken TECS gaz kontrolünü hava-hızsız yola düşürür (`_SKE_weighting=0`):
   hız komutları `COMMAND_ACK=ACCEPTED` döner ama uçağa HİÇ etki etmez —
   uçak `TRIM_THROTTLE` trim hızında sabit uçar. Düzeltme uygulandı:
   avcıda `params/bumblebee.parm` → `TECS_SYNAIRSPEED 1`; hedefte yeni
   `params/target.parm` (SYNAIRSPEED 1 + `AIRSPEED_CRUISE 20`) launcher'ın
   `--defaults` zincirine eklendi. Bu düzeltmeyle **hız komutu iki uçakta
   da tutuyor** (IAS komuta kilitlenir). AMA sentetik IAS'in yapısı şu:
   `IAS = |V_gps − W_EKF3|` (`AP_AHRS::_airspeed_EAS()`); rüzgarsız
   world'de bile EKF3 rüzgar tahmini düz bacakta gözlenemeyip kaydığı için
   IAS ile GPS arasında uçak-başına FARKLI bir ofset oluşur (ölçüm: avcı
   +0.29, hedef −2.19 m/s; yani **aynı IAS komutu ≠ aynı yer hızı**).
   `AHRS_WIND_MAX 1` (iki parm'a eklendi, 2026-07-25) bu hatayı ±1 m/s'ye
   sınırlar (AP_Int8 — 1 en küçük etkili değer). Dönüşler EKF tahminini
   yeniden oynatır; ofset sabit değildir. Dönüşüm çarpanı GEREKMİYOR;
   `speed_source` soyutlamasını kur (SIM: GPS yer hızı, gerçek uçuş:
   pitot), loglara iki sinyali de yaz ve yer-hızı hedefi isteyen kontrolcü
   kendi (IAS−GPS) ofsetini canlı ölçüp komuta eklesin —
   `formation.py --speed`'in `equalise_speed()` fonksiyonu bu desenin
   çalışan örneğidir (kalkışta iki uçağı ≤0.33 m/s farkla eşitledi).
   NOT: bu kurulumda gerçek pitot simüle EDİLEMEZ (Gazebo backend'i
   `update_eas_airspeed()`'i hiç çağırmıyor; ARSPD_TYPE 100 de 0 okur).
2. **Komut deseni: yalnız `MAV_CMD_GUIDED_CHANGE_SPEED` (43000) kullan**
   (param1=0 airspeed, param2=m/s, param3=ivme; heading 43002, irtifa
   43001 — 43001 irtifayı HOME'a göre alır, eski tzi2.py'deki "AMSL"
   yorumu yanlış). `DO_CHANGE_SPEED` (178) de çalışır AMA 43000 bir kez
   kullanıldıktan sonra mod yeniden girilene dek 178'i ezer — ikisini
   KARIŞTIRMA. Basamak oturma süreleri: avcı 2-13 s, hedef 7-17 s.
3. **Zarf dışı komut KIRPILMAZ, REDDEDİLİR** (`MAV_RESULT_FAILED`) ve uçak
   sessizce eski hızında kalır. Bu yüzden güdüm, komutu göndermeden ÖNCE
   kendisi [min+1, max-1] aralığına kırpmalı; zarf koruması ayrıca
   DEĞİŞTİRİLEMEZ güvenlik katmanında da olsun. Zarflar: avcı
   AIRSPEED_MIN 15 / MAX 24; hedef 9 / 22 (fabrika değerleri — jenerik
   parm'daki eski `ARSPD_FBW_*`/`TRIM_ARSPD_CM` adları 4.7'de sessizce
   yok sayılır). Hedef 22 m/s'de gaz %97 → üst zarfta itki rezervi yok.
   Gerçek loglarda sürdürülen en düşük düz-uçuş hızı 13.7-16.3 m/s idi;
   avcı alt sınırını senaryolarda buna göre seç.
4. **Kovalama geometrisi artık doğrudan kontrol edilebilir.** Eski ölçüm
   (avcı 25.6 vs hedef 20.2 m/s, `--delay 2`'de t≈13 s'de sollama)
   açık-döngü rejimindeydi; artık iki uçağa da hız komutu geçtiği için
   hedef-önde geometri delay + hız komutu kombinasyonuyla senaryo
   parametresi olarak kurulur (hedefe hız override'ı artık GERÇEKTEN
   çalışıyor; SYNAIRSPEED 0 iken komutlar ACCEPTED dönüp hiç etki
   etmiyordu).
5. **EEPROM tuzağı:** `run/sitl*/eeprom.bin` parm dosyasını gölgeler
   (`--defaults` yalnız varsayılanları yükler; EEPROM'daki kayıtlı değer
   kazanır — avcıda dosya 24 derken canlı AIRSPEED_MAX 20 çıktı, 22/24
   komutları bu yüzden reddediliyordu). Parm değiştirince canlı değeri
   `param fetch` ile doğrula; gerekirse eeprom.bin'i BİLEREK sil.
   Gerçek-uçuş tarafındaki benzer tuzak: `ARSPD_AUTOCAL` açıkken pitot
   oranı uçuştan uçuşa kayabilir (%2.6 ölçüldü) — gerçek-log
   karşılaştırmalarında ARSPD_RATIO'ya bak.
6. **Hazır test aracı:** `command_sender.py` (repo kökünde) —
   `./command_sender.py --connect udpin:127.0.0.1:14553 --sysid 1
   --heading 90 --alt 80 --speed 22 --hold 60` deseniyle heading+irtifa+
   hız gönderir, ACK'leri basar, CSV loglar. Güdüm kodu aynı gönderme
   desenini kullansın. Doğrulanmış birleşik test: avcı hdg 90/alt 80/
   hız 22 → 89-90° / 80.0 m / IAS 22.00; hedef hdg 180/alt 60/hız 18 →
   180° / 60.0 m / IAS 18.00.

## 3. Kurulacak altyapı — rol ayrımı

```
Araştırmacı ajan  →  yalnız şunları değiştirir:
                       guidance_lab/controller.py
                       guidance_lab/controller_config.yaml
Değiştirilemez katman (araştırmacı ajan DOKUNAMAZ, seed'leri GÖREMEZ):
                       guidance_lab/scenario_generator.py
                       guidance_lab/run_experiment.py
                       guidance_lab/evaluator.py
                       guidance_lab/safety_monitor.py
                       guidance_lab/hidden_tests/        (validation seed'leri)
```

Akış: araştırmacı hipotez yazar → controller'ı değiştirir → run_experiment
statik kontrol + smoke + development testlerini koşar → umut varsa
validation → kabul/ret → git checkpoint. Bu görevde SEN değiştirilemez
katmanı inşa ediyorsun; controller.py başlangıçta mevcut tzi_emir.py
mantığının (heading+altitude) temiz bir sarmalayıcısı olabilir.

Başlangıçta araştırmacının arama alanını dar tut (kademe 1-2): PD/PID
kazançları, filtre sabitleri, gain scheduling, feedforward, hedef açısal hız
tahmini, gecikme telafisi. MPC/öğrenilmiş modeller sonraki kademeler.

## 4. Senaryo dili — waypoint listesi değil, parametrik tanım

Rastgele waypoint üretme: uçamayan dönüşler, tekrar eden hareketler ve
kapsanmayan uzay üretir. Bunun yerine hedef hareketi MOTION PRIMITIVE
dizisiyle tanımlanır ve iki yoldan uygulanır: (a) plan dosyası üretimi
(mevcut `load_plan.py` MISSION_ITEM_INT altyapısı — düz/dönüşlü kaba
rotalar için), (b) hedefe GUIDED komut akışı (turn-rate/climb-rate kontrolü
gereken primitive'ler için — önerilen).

Primitive seti (başlangıç): sabit heading+altitude · sabit turn rate ·
sabit climb · koordineli dönüş+tırmanış · S-turn · dönüş yönü değişimi ·
hızlanma/yavaşlama rampası · yaklaşma/uzaklaşma.

Senaryo örneği (YAML; FOV gerçeğine dikkat — başlangıç azimut/elevasyonu
dar konide seçilmezse senaryo "yeniden edinme" ailesine girer):

```yaml
initial:
  relative_azimuth_deg: 5        # yatay FOV ±7.74
  relative_elevation_deg: -2     # dikey FOV ±4.37
  range_m: 150                   # 80-400 aralığından örnekle
  target_speed_mps: 20           # hedefe 43000 ile komutlanır (zarf 9-22)
  pursuer_speed_mps: 20          # avcı komutu (zarf 15-24; IAS≈GPS, bkz §2)
  formation_delay_s: 12
maneuvers:
  - {start_s: 10, duration_s: 15, turn_rate_deg_s: 4,  climb_rate_mps: 0}
  - {start_s: 30, duration_s: 10, turn_rate_deg_s: -6, climb_rate_mps: 1.5}
environment:
  wind_speed_mps: 0              # ilk sürümde 0; DR bölümüne bak
  detector_dropout_prob: 0.0
```

Fiziksel geçerlilik filtresi zorunlu: `turn_rate ≤ g·tan(bank_max)/V`,
tırmanma ≤ ~5 m/s (loglardan: 4.9-5.7), hız zarf içinde. Geçersiz senaryo
üretilirse örnekleyici düzeltir, koşucuya ulaşmaz.

**Üç senaryo kümesi:**
1. Development: 20 SABİT regresyon senaryosu + her deneyde yeniden
   örneklenen 30 rastgele senaryo. Araştırmacı ayrıntılı sonuç görür.
2. Validation (~100, gizli seed): araştırmacı yalnız toplam sonucu görür;
   sadece development'ı geçen adaylarda koşulur.
3. Human holdout: kullanıcının elle hazırladığı/uçurduğu profiller; seyrek,
   insan kontrolünde.

Her GERÇEK başarısızlık (hedef kaybı, gate ihlali) kalıcı regresyon
senaryosu olarak development kümesine eklenir.

## 5. Skorlama

Normalize piksel hatası (çözünürlükten bağımsız):
`e(t) = sqrt(((x-cx)/(W/2))^2 + ((y-cy)/(H/2))^2)`

Senaryo başına metrikler: ortalama e · p95(e) · merkez bölgede geçen süre ·
hedef-görüntü-dışı süresi · kayıptan sonra yeniden edinme süresi · komut
düzgünlüğü (heading/alt/hız setpoint rate) · saturation süresi · min/maks
mesafe · zarf ihlalleri.

Birleşik kayıp (başlangıç ağırlıkları; kalibre edilebilir):
`L = 0.35·tracking + 0.25·worst_tail + 0.15·lost + 0.10·reacq + 0.10·smooth + 0.05·saturation`
`worst_tail` = en kötü %20 senaryonun ortalaması — kolay senaryolarda puan
toplamak yetmesin.

**HARD GATE'ler skordan AYRIDIR — biri ihlal edilirse aday diskalifiye:**
stall altına düşme · aşırı bank/pitch · minimum ayrım mesafesi ihlali ·
NaN/exception · sürekli saturation · hedefi T saniyeden uzun kaybetme ·
uçuş alanı dışına çıkma · komutta tehlikeli sıçrama.

Bilgi ayrımı: **kontrolcü girdisi = yalnız görüntü/bbox + izin verilen
telemetri; evaluator girdisi = simülasyon ground truth'unun tamamı**
(Gazebo get_model_state / MAVLink GLOBAL_POSITION_INT — bu oturumda ikisi
de çalışır durumda doğrulandı; GPS logger deseni: tek `udpin:14550` soketi,
SysID ile ayır — port çakışması yaşanmaz).

## 6. Kabul mekanizması ve git düzeni

Aday ile şampiyon AYNI seed'lerde koşulur. Kabul koşulları:
1. Hard gate ihlali: 0.
2. Development ortalama kaybı ≥ %2 iyileşmiş.
3. En kötü %20 senaryoda kötüleşme yok.
4. Hiçbir senaryo ailesinde > %5 gerileme yok.
5. Validation'da iyileşme korunmuş.

Her deney için yapılandırılmış kayıt (JSON; ajan serbest-metnine güvenme):
`experiment_id, parent_commit, hypothesis, changed_files, scenario_seeds,
dev_metrics, val_metrics, worst_scenarios, runtime, accepted, reason`.

Branch düzeni: `main/champion` yerine bu repoda `loop_testing` üstünde
`experiment/NNNNNN` dalları; kabul edilen aday loop_testing'e merge edilir,
reddedilen arşivlenir/silinir. `main`'e asla dokunulmaz.

## 7. Test hunisi ve metamorfik testler

Sim ~gerçek zamanlı çalışır (her tam senaryo dakikalar sürer) — huni şart:
1. **Statik:** import/interface/NaN/limit kontrolleri (saniyeler).
2. **Smoke:** 5 kısa senaryo (sağ, sol, tırmanış, alçalma, uzaklaşan hedef).
   Başarısızsa hemen ret.
3. **Development:** 20 sabit + 30 örneklenmiş tam SITL senaryosu.
4. **Validation:** yalnız umut veren adaylara.
5. **Adversarial (sonraki aşama):** önce basit yöntem — random search +
   başarısız senaryo mutasyonu; CMA-ES/Bayesian sonra. LLM senaryo AİLESİ
   önerir, sayısal arama ailenin içindeki en zor parametreyi bulur.
6. HITL/gerçek uçuş: insan kontrolünde, seyrek (shadow-mode önerisi:
   aday komut üretir ama yalnız loglanır).
- Opsiyonel araştırma görevi: SITL `--speedup` + Gazebo adım hızlandırma
  denenmemiş — huniyi hızlandırabilir; kalibrasyonu bozmadığı gösterilmeden
  skor koşularında KULLANMA.

**Metamorfik testler (ucuz ve çok güçlü — ilk sürümde şu beşi):**
1. Yatay ayna: senaryo sağ-sol aynalanır → skor ~aynı, heading komut işareti
   dönmeli. 2. Dünya-heading dönüşü: aynı görev farklı mutlak yönde → sonuç
   değişmemeli. 3. Frame-rate (30→20→15 FPS) → kazançlar zaman-başına
   tanımlıysa dayanır. 4. Latency taraması (kamera gecikmesi kademeli
   artar). 5. Sıfır-hata sükuneti: hedef merkezde sabitken kontrolcü
   salınım üretmemeli.

## 8. Domain randomization — başlangıçta DAR

İlk sürümde yalnız: bbox jitter (±birkaç px), tek-frame dropout
(olasılık ~0.02), kamera gecikmesi (sabit + küçük varyans), hafif rüzgâr.
Dağılım sınırlarını kafadan değil ölçümden al: detector gecikme/dropout
istatistiklerini önce mevcut ortamda ÖLÇ, dağılımı ona uydur. Geniş
randomizasyon (kütle/CG/atalet) SONRAKİ aşama — erken açılırsa aşırı
muhafazakâr kontrolcü üretir.

## 9. İlk prototip kapsamı (bu görevin tesliminde olması gerekenler)

- `guidance_lab/` iskeleti: değiştirilemez katman + controller sarmalayıcı.
- Senaryo şeması (YAML) + örnekleyici + fiziksel geçerlilik filtresi +
  20 sabit regresyon senaryosu (çeşitlilik: düz takip, iki yönlü dönüş,
  tırmanış/alçalma, uzaklaşan hedef, delay varyasyonu).
- Koşucu: ortamı başlatır (headless), senaryoyu uygular, bbox + GPS ground
  truth + komutları zaman damgalı toplar, temiz kapatır (kapat.sh deseni).
- Evaluator: metrikler + birleşik kayıp + hard gate'ler; senaryo başına ve
  toplu JSON rapor.
- 5 senaryoluk smoke + 5 metamorfik test.
- Aday-şampiyon karşılaştırma scripti (aynı seed'ler, tablo + karar).
- KISA kullanım dokümanı (`guidance_lab/README.md`).
- Hız ekseni: koşucu/evaluator hız sinyallerini (GPS + IAS) baştan
  loglasın ki hız kontrolcüsü deneyleri altyapı değişikliği istemesin;
  `speed_source` soyutlaması bu görevde kurulsun ve hız gönderimi
  `command_sender.py` deseniyle (yalnız 43000) yapılsın.

## 10. Eski Rota 2/3/4 içeriği

`plan_rota_testleri.md` hâlâ referanstır; Rota 2 (büyük daire) ve Rota 3
(sürekli tırmanan hedef) bu çerçevede birer SENARYO AİLESİ olarak
devşirilmelidir (ayrı prosedür olarak koşturulmaz). `verify_flight.py`
(--require-waypoints 1 uzun bacaklarda) ve `tools/plan_dogrula.py` fikri
senaryo geçerlilik filtresinin parçası olabilir.

## Rapor formatı

Her aşamada: ne kuruldu, hangi testle doğrulandı (gerçek çıktıyla), bilinen
sınırlar. Uydurma sonuç yok; koşulmayan şey "koşulmadı" diye yazılır.
Bittiğinde: 20 regresyon senaryosunun listesi + smoke/metamorfik testlerin
gerçek koşu sonuçları + örnek bir aday-şampiyon karşılaştırma çıktısı.
