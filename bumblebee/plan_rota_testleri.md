# Güdüm Test Rotaları Planı — Rota 2/3/4

> Yürütücü agent için. Türkçe çalış/raporla. Çalışma dizini: `/home/tzi4/gudum/bumblebee`.
> Git yok (push/PR/commit yapma). `~/catkin_ws`, `~/ardupilot*` dizinlerine yazma.
> Değiştireceğin her mevcut dosyayı önce `arsiv/yedek_onceki_3/` altına yedekle.

## Bağlam ve mevcut durum

Ortam, `emir_gudum.sh` eşleniği olarak kuruldu ve doğrulandı. Kritik gerçekler:

- İki başlatıcı TEK çekirdeği sarar: `basla.sh` (GUI) / `basla_headless.sh` → `bumblebee_gudum.sh`.
  TEK FARK GUI'dir; bunu bozma. Ayarlar tek kaynaktan: plan yolları çekirdeğin başında
  `HUNTER_PLAN`/`TARGET_PLAN` (env `BUMBLEBEE_HUNTER_PLAN`/`BUMBLEBEE_TARGET_PLAN` ile
  ezilebilir; varsayılan `missions/duz_uzun.plan`, iki uçak da). Kalkış gecikmesi tek
  kaynak: `formation.py --delay` varsayılanı **2.0** (KULLANICININ değeri — dokunma).
- Kabul takımı (`bumblebee_gudum.sh --verify` → `scripts/verify_suite.sh`) kendi kapalı
  rotasını ve SABİT `--delay 12`'yi kullanır — kullanıcı ayarlarından bilinçli yalıtıldı.
- Port sözleşmesi SABİT: avcı SysID 1 → 14551+14553, hedef SysID 2 → 14561, QGC 14550,
  FDM 9002/9012. Güdüm `tzi_emir.py` 14553'e bağlanır; bbox köprüsü otomatik başlar
  (`tracker_bbox` Redis kanalı, format [x,y,w,h,cov,validity]).
- `load_plan.py` her zaman MISSION_ITEM_INT gönderir (uzak/hassas koordinatlar sorunsuz).
- Rota 1 (duz_uzun) TAMAM: kuzeye ~200 km, iki uçak aynı 50 m, 2 s gecikme → havada
  ~66 m sabit ayrışma, bbox 60 s'de 1592 tespit. Amaç: hedef tam öndeyken güdümün
  komut ekonomisi. Test raporu: `reports/duz_uzun_test/verification_20260722_165124.json`
  (passed:False yalnız `waypoint_progress` artefaktı — 180 s'de 10 km'lik ilk leg'de
  wp3'e ulaşılamaz; uzun düz rotalarda `--require-waypoints 1` kullan).

## Ortak kurallar (her rota için)

1. Yeni rota = `missions/` altında yeni `.plan` dosyası. Rota seçimi YALNIZ env
   değişkenleriyle veya çekirdekteki tek varsayılan satırla yapılır — sarmalayıcıları
   çatallama, ikinci bir launcher yazma.
2. Plan formatı: QGC `.plan` (frame 3 rel-alt, cmd 22 takeoff + cmd 16 waypoint),
   home `41.101658,28.545652`, cruiseSpeed 18.
3. Test reçetesi (headless): portlar boşken
   `BUMBLEBEE_HUNTER_PLAN=... BUMBLEBEE_TARGET_PLAN=... ./basla_headless.sh` →
   `./formation.py --yes` (varsayılan gecikme) → `./verify_flight.py --duration N
   --require-waypoints 1 --report-dir reports/<rota_adi>` → `./kapat.sh`.
4. Kabul: çarpışma yok (min_separation_m ≥ 20; senaryo gereği daha az bekleniyorsa
   raporda açıkça gerekçele), bbox tespiti sürüyor, attitude kararlı, kabul takımı
   (`--verify`) bozulmadı (her rota işinden sonra bir kez koştur).
5. Eşik/gecikme/parametre sessizce gevşetme; sapmaları raporla.

## Rota 2 — Büyük daire (yatay sabit tutma testi)

Amaç: hedef geniş bir dairede dönerken güdümün hedefi yatayda merkezde tutabilmesi.

- `missions/daire.plan`: merkez ~home'un 3 km kuzeyi, yarıçap 600–800 m, 12–16
  waypoint'lik çokgen (saat yönü veya tersi; ArduPlane 18 m/s'de min dönüş yarıçapı
  ~35 m, 600 m rahat). İrtifa 60 m sabit. Son waypoint'ten ilkine `DO_JUMP` (cmd 177)
  ile sonsuz döngü — 180+ s test kesintisiz dönsün.
- Avcı için ayrı dosya GEREKMEZ: aynı daire planı iki uçağa da (2 s gecikme ark-boyu
  ayrışmayı korur — rota 1 ile aynı desen). İstenirse `daire_hunter.plan` ile avcıyı
  5 m yukarı al; önce aynı-plan varyantını dene.
- Metrik: bbox x-merkezinin kadraj merkezinden sapması (px, zaman serisi) — tzi_emir
  logundan veya `tracker_bbox` kaydından türet; ayrıca komut sayısı.

## Rota 3 — Sürekli tırmanan hedef (dikey takip testi)

Amaç: sürekli yükselen hedefi dikeyde yakalayabilme.

- `missions/tirmanis.plan`: kuzeye düz, her ~2 km'de bir waypoint, her leg'de +100 m
  (50→150→250→350→450 m; 18 m/s'de ~0.9 m/s sürekli tırmanma — TECS için rahat).
- DİKKAT — iki ön koşul (bu rotadan ÖNCE yap):
  a) `verify_flight.py`'deki 200 m tavan kontrolü senaryo parametresi olmalı: `--max-alt`
     bayrağı ekle (varsayılan 200 → kabul takımı etkilenmez; tirmanis testinde 500 ver).
  b) SITL `RTL_ALT` ve batarya failsafe'lerinin 450 m'de tetiklenmediğini kontrol et.
- Avcıya aynı plan (2 s gecikme); metrik: bbox y-merkez sapması + irtifa farkı zaman serisi.

## Rota 4 — Serbest path (kullanıcı çizer, son test)

- Kullanıcı QGC'de path çizer → `.plan` olarak kaydeder → `missions/serbest.plan`.
- Yükleme: `BUMBLEBEE_HUNTER_PLAN=missions/serbest.plan BUMBLEBEE_TARGET_PLAN=missions/serbest.plan ./basla_headless.sh`
  (loader INT gönderdiğinden uzak/hassas koordinatlar sorunsuz).
- Yürütücünün işi: kullanıcının dosyasını `tools/` altına eklenecek küçük bir
  `plan_dogrula.py` ile denetle (frame 3 mü, takeoff var mı, home doğru mu, irtifalar
  makul mü) ve standart test reçetesini koştur. Kullanıcı dosyasını DEĞİŞTİRME;
  sorun varsa raporla.

## Sıra ve teslim

Rota 2 → (ön koşullar) → Rota 3 → Rota 4 altyapısı. Her rotadan sonra: test JSON yolu,
min_separation_m, bbox sayısı, metrik zaman serisinin özeti, `--verify` durumu.
Son rapor: değişen dosyalar + yedek yeri, rota başına sonuç tablosu, kullanıcıya kalanlar.

Not: Bu plan yeni bir oturumda çalıştırılırsa Write/Edit doğrudan çalışır
(`.claude/settings.json` içinde `bgIsolation: none` ayarlı). Bu oturum içinde
çalıştırılırsa staging (`$CLAUDE_JOB_DIR/tmp/stage/`) + `cp` gerekir.
