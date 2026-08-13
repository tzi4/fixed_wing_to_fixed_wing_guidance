#!/usr/bin/env python3
"""
Kamera <-> otopilot açısı sabit mi? Sanal gimbal gerçekten çalışıyor mu?

Bu test SADECE güdüm CSV'si ile yapılabilir (uçağın .BIN logunda kamera verisi yok).
Emir'den gelen flight_log_guided_*.csv dosyasını verince çalışır.

Kullanım:
    python3 tools/kamera_aci_testi.py flight_log_guided_YYYYMMDD_HHMMSS.csv

Matematik (goat_gimbal_ucak.py'deki stabilize_pixel için türetildi):

  raw_error_y_deg  = -(phi - theta + eps)        # ham piksel hatası
  stab_error_y_deg = -(phi + b + eps)            # sanal gimbal çıkışı

    phi   = hedefin yer eksenindeki yükselme açısı
    theta = uçağın GERÇEK pitch'i
    b     = otopilotun bildirdiği pitch hatası (bildirilen - gerçek)
    eps   = kameranın gövdeye göre pitch montaj açısı (aşağı bakma +)

Buradan üç bağımsız test çıkıyor:

  TEST 1 (RİJİTLİK): d(raw_error_y_deg)/d(pitch_deg) = +1.000 olmalı.
      Sapma varsa -> kamera gövdeye göre kıpırdıyor, ya da fy yanlış.
      Sanal gimbalin çalışması için ASIL gereken şart budur.

  TEST 2 (DE-ROTASYON): stab_error_y_deg, pitch_deg ile korelasyonsuz olmalı.
      Korelasyon varsa -> gimbal pitch'i tam temizleyemiyor.

  TEST 3 (SABİT OFSET): stab_error_y_deg'in ortalaması = -(phi + b + eps).
      Hedef ortalamada ufuk hizasındaysa (phi~0) bu doğrudan (b + eps) verir.
      b ve eps'i BİRBİRİNDEN AYIRMAK mümkün değildir - ikisi matematiksel
      olarak aynı şeyi yapar. Zaten ayırmaya gerek yok: tek bir sayı ikisini
      birden düzeltir.

  VARYANS ORANI: var(stab)/var(raw). Gimbal çalışıyorsa <<1 olmalı.
"""
import sys, csv, math
import numpy as np

def load(path):
    rows = list(csv.DictReader(open(path)))
    def col(name):
        out = []
        for r in rows:
            try: out.append(float(r[name]))
            except (ValueError, KeyError, TypeError): out.append(np.nan)
        return np.array(out)
    d = {k: col(k) for k in ['elapsed_s','pitch_deg','roll_deg','yaw_deg',
                             'raw_error_y_deg','stab_error_y_deg',
                             'raw_error_x_deg','stab_error_x_deg',
                             'bbox_center_y','stab_y','current_alt_m']}
    d['target_found'] = np.array([r.get('target_found','0') for r in rows])
    d['mode'] = np.array([r.get('flight_mode','') for r in rows])
    return d

def hedef_var(d):
    """Sadece hedefin gerçekten görüldüğü ve sayıların anlamlı olduğu satırlar."""
    m = (d['target_found'] == '1')
    for k in ['pitch_deg','raw_error_y_deg','stab_error_y_deg']:
        m &= np.isfinite(d[k])
    m &= np.abs(d['raw_error_y_deg']) < 20
    m &= np.abs(d['stab_error_y_deg']) < 20
    return m

def hp(x, w):
    """Yüksek geçiren: hedefin yavaş hareketini at, pitch salınımını bırak."""
    k = np.ones(w)/w
    return x - np.convolve(x, k, mode='same')

def main(path):
    d = load(path)
    m = hedef_var(d)
    n = int(m.sum())
    print(f"Dosya: {path}")
    print(f"Hedef görülen örnek sayısı: {n} / {len(d['elapsed_s'])}")
    if n < 300:
        print("YETERSİZ VERİ (>=300 gerekli). Test yapılamıyor."); return

    t   = d['elapsed_s'][m]
    pit = d['pitch_deg'][m]
    rol = d['roll_deg'][m]
    raw = d['raw_error_y_deg'][m]
    stb = d['stab_error_y_deg'][m]
    rawx= d['raw_error_x_deg'][m]
    stbx= d['stab_error_x_deg'][m]

    dt = np.median(np.diff(t)) if len(t) > 1 else 0.033
    W = max(5, int(round(3.0/dt)))     # 3 sn yüksek geçiren penceresi

    print(f"pitch aralığı: {pit.min():.1f}° .. {pit.max():.1f}°  (std {pit.std():.2f}°)")
    if pit.std() < 0.8:
        print("UYARI: pitch neredeyse hiç değişmemiş; TEST 1/2 anlamsız olur.")

    # ---- TEST 1: RİJİTLİK ----
    print("\n=== TEST 1 — RİJİTLİK (kamera gövdeye sabit mi?) ===")
    a = hp(raw, W); b_ = hp(pit, W)
    if b_.std() > 1e-6:
        slope, icept = np.polyfit(b_, a, 1)
        r = np.corrcoef(b_, a)[0,1]
        print(f"  d(raw_error_y)/d(pitch) = {slope:+.3f}   (BEKLENEN: +1.000)")
        print(f"  korelasyon r = {r:+.3f}   (|r| > 0.7 olmalı ki eğim anlamlı olsun)")
        if abs(r) < 0.7:
            print("  -> Korelasyon zayıf: hedef çok hareketli ya da pitch salınımı yok. Sonuç güvenilmez.")
        elif abs(slope - 1.0) < 0.12:
            print("  -> GEÇTİ. Kamera gövdeye rijit bağlı, ölçek doğru.")
        elif slope < 0.88:
            print(f"  -> KALDI. Eğim 1'den küçük: kamera pitch'le birlikte KISMEN dönüyor")
            print(f"     (montaj esniyor) ya da fy gerçekte {slope:.3f}x daha küçük.")
        else:
            print(f"  -> KALDI. Eğim 1'den büyük: fy fazla küçük girilmiş olabilir.")

    # ---- TEST 2: DE-ROTASYON ----
    print("\n=== TEST 2 — DE-ROTASYON (gimbal pitch'i temizliyor mu?) ===")
    c = hp(stb, W)
    if b_.std() > 1e-6:
        slope2, _ = np.polyfit(b_, c, 1)
        r2 = np.corrcoef(b_, c)[0,1]
        print(f"  d(stab_error_y)/d(pitch) = {slope2:+.3f}   (BEKLENEN: 0.000)")
        print(f"  korelasyon r = {r2:+.3f}   (BEKLENEN: ~0)")
        if abs(slope2) < 0.15: print("  -> GEÇTİ. Pitch temizlenmiş.")
        else: print(f"  -> KALDI. Pitch'in {abs(slope2)*100:.0f}%'i stabilize çıkışına sızıyor.")

    vr = np.var(c)/np.var(a) if np.var(a) > 0 else float('nan')
    print(f"  varyans oranı var(stab)/var(raw) = {vr:.3f}   (BEKLENEN: <0.25)")
    if vr < 0.25: print("  -> Sanal gimbal dikey eksende iş görüyor.")
    elif vr < 1.0: print("  -> Kısmen çalışıyor.")
    else: print("  -> Gimbal dikey eksende İŞE YARAMIYOR (hatta kötüleştiriyor).")

    # ---- TEST 3: SABİT OFSET ----
    print("\n=== TEST 3 — SABİT OFSET (b + eps) ===")
    med = np.median(stb); mean = np.mean(stb)
    print(f"  stab_error_y_deg: medyan {med:+.2f}°  ortalama {mean:+.2f}°  std {stb.std():.2f}°")
    print(f"  -> toplam dikey sapma (b + eps) ≈ {-med:+.2f}°  (hedef ortalama ufuk hizasındaysa)")
    print(f"  -> goat_gimbal_ucak.py Kp_alt=2.5 ile irtifa komut sapması: {abs(med)*2.5:.1f} m")
    print(f"     (clamp aralığı -5..+2 m — {'SATÜRE OLUR' if abs(med)*2.5 > 2 else 'sınır içinde'})")
    print(f"  -> eşdeğer piksel kayması (fy=4510): {abs(med)*math.pi/180*4510:.0f} px")

    # ---- Yatay eksen (karşılaştırma için) ----
    print("\n=== YATAY EKSEN (karşılaştırma) ===")
    ax = hp(rawx, W); cx = hp(stbx, W); br = hp(rol, W)
    vrx = np.var(cx)/np.var(ax) if np.var(ax) > 0 else float('nan')
    print(f"  varyans oranı var(stab_x)/var(raw_x) = {vrx:.3f}")
    print(f"  stab_error_x_deg medyan {np.median(stbx):+.2f}°")
    if br.std() > 1e-6:
        print(f"  d(stab_error_x)/d(roll) = {np.polyfit(br, cx, 1)[0]:+.3f}  (BEKLENEN: 0.000)")

    # ---- Ofset kayıyor mu? ----
    print("\n=== OFSET ZAMAN İÇİNDE SABİT Mİ? ===")
    q = np.array_split(np.arange(len(stb)), 4)
    meds = [float(np.median(stb[i])) for i in q if len(i) > 30]
    print("  çeyrek medyanlar: " + "  ".join(f"{v:+.2f}°" for v in meds))
    if len(meds) > 1:
        rng = max(meds) - min(meds)
        print(f"  yayılım {rng:.2f}°")
        print("  -> " + ("SABİT. Statik kalibrasyon sorunu, tek sayıyla düzeltilir."
                         if rng < 1.0 else
                         "KAYIYOR. Montaj esniyor ya da pitch hatası uçuş koşuluyla değişiyor."))

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for p in sys.argv[1:]:
        main(p); print("\n" + "="*66 + "\n")
