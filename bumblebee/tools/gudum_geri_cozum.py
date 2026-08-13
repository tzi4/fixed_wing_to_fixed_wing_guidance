#!/usr/bin/env python3
"""
Görüntülü güdümün çıkışını UÇAĞIN KENDİ .BIN LOGUNDAN geri çöz.

Güdüm CSV'si olmasa bile çalışır: goat_gimbal_ucak.py'nin gönderdiği
MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) ve _CHANGE_HEADING (43002) komutları
ArduPilot tarafından MAVC mesajına yazılıyor.

Geri çözüm zinciri (Ki_alt = 0 olduğu için integral terimi yok):

    MAVC[43001].Z          = target_alt          (gönderilen mutlak irtifa)
    POS.RelHomeAlt         = current_alt         (güdümün okuduğu irtifa)
    cmd_alt_m              = target_alt - current_alt        <- PID çıkışı
    p_y                    = -cmd_alt_m / Kp_alt             <- deadzone'lu açı hatası
    stab_error_y_deg       ≈ p_y + sign(p_y)*deadzone        <- sanal gimbal çıkışı

    MAVC[43002].P2         = target_heading
    cmd_head_deg           = wrap180(target_heading - yaw)
    p_x                    = cmd_head_deg / Kp_heading
    stab_error_x_deg       ≈ p_x + sign(p_x)*deadzone

Kullanım:  python3 tools/gudum_geri_cozum.py <log.BIN> [...]
"""
import sys, os, math
import numpy as np
from pymavlink import mavutil

KP_ALT, KP_HDG, DEADZONE = 2.5, 3.0, 0.4
CLAMP_LO, CLAMP_HI = -5.0, 2.0      # goat_gimbal_ucak.py son sürüm
FY = 4510.0                          # gerçek kamera kalibrasyonu

def wrap180(a): return (a + 180.0) % 360.0 - 180.0

def analiz(path):
    m = mavutil.mavlink_connection(path)
    alt_cmd=[]; hdg_cmd=[]; pos=[]; att=[]; gps=[]; mode=[]
    while True:
        x = m.recv_match(type=['MAVC','POS','ATT','GPS','MODE'])
        if x is None: break
        t = x.get_type()
        if t=='MAVC':
            if x.Cmd==43001: alt_cmd.append((x.TimeUS/1e6, x.Z, x.P3))
            elif x.Cmd==43002: hdg_cmd.append((x.TimeUS/1e6, x.P2, x.P3))
        elif t=='POS': pos.append((x.TimeUS/1e6, x.RelHomeAlt))
        elif t=='ATT': att.append((x.TimeUS/1e6, x.Pitch, x.Roll, x.Yaw))
        elif t=='GPS': gps.append((x.TimeUS/1e6, x.Spd))
        elif t=='MODE': mode.append((x.TimeUS/1e6, x.Mode))

    print(f"\n{'='*70}\n{os.path.basename(path)}")
    if not alt_cmd:
        print("  Güdüm komutu yok (bu logda görüntülü güdüm çalışmamış).")
        return

    ac=np.array(alt_cmd); hc=np.array(hdg_cmd); po=np.array(pos); at=np.array(att); gp=np.array(gps)

    # --- GUIDED süresi ve KİLİT ORANI ---
    guided_s = 0.0
    if mode:
        tm=[q[0] for q in mode]+[at[-1,0]]
        for i,(t0,md) in enumerate(mode):
            if int(md)==15: guided_s += tm[i+1]-t0
    span = ac[-1,0]-ac[0,0]
    print(f"  GUIDED süresi: {guided_s:.0f} s | güdüm komutu: {len(ac)} adet | "
          f"komut aralığı: {span:.0f} s")
    # kod 0.1 s'de bir gönderiyor -> teorik max 10 Hz; gerçekleşen oran = kilit süresi
    kilit_s = len(ac)*0.1
    print(f"  >>> KİLİT SÜRESİ ≈ {kilit_s:.0f} s  "
          f"(GUIDED'ın %{100*kilit_s/max(guided_s,1):.1f}'i)")
    # kesintisiz kilit blokları
    dt_cmd=np.diff(ac[:,0])
    blocks=np.split(np.arange(len(ac)), np.where(dt_cmd>0.5)[0]+1)
    bl=sorted((len(b)*0.1 for b in blocks), reverse=True)
    print(f"  kilit bloğu sayısı: {len(bl)} | en uzun: {bl[0]:.1f} s | "
          f"medyan: {np.median(bl):.1f} s | >5s olan: {sum(1 for x in bl if x>5)}")

    # --- param3 (tırmanma hızı limiti) ---
    p3=np.unique(ac[:,2])
    print(f"  MAV_CMD_GUIDED_CHANGE_ALTITUDE param3 (tırmanma hızı): {p3}"
          f"  {'-> LİMİT YOK' if np.allclose(p3,0) else ''}")
    if len(hc): print(f"  CHANGE_HEADING param3 (dönüş hızı): "
                      f"{np.min(hc[:,2]):.2f} .. {np.max(hc[:,2]):.2f} °/s")

    # --- PID çıkışını geri çöz ---
    cur = np.interp(ac[:,0], po[:,0], po[:,1])
    cmd = ac[:,1] - cur
    ok = np.abs(cmd) < 60          # zemin güvenliği (target_alt<10 -> 10) saçmalarını at
    cmd = cmd[ok]; tt = ac[ok,0]
    print(f"\n  --- DİKEY EKSEN ---")
    print(f"  cmd_alt_m: medyan {np.median(cmd):+.2f} m | ort {cmd.mean():+.2f} | "
          f"std {cmd.std():.2f} | p5 {np.percentile(cmd,5):+.2f} | p95 {np.percentile(cmd,95):+.2f}")
    sat_lo = float((cmd <= CLAMP_LO+0.05).mean()*100)
    sat_hi = float((cmd >= CLAMP_HI-0.05).mean()*100)
    disari = float(((cmd < CLAMP_LO-0.1)|(cmd > CLAMP_HI+0.1)).mean()*100)
    print(f"  clamp(-5,+2) satürasyonu: alt sınırda %{sat_lo:.1f}, üst sınırda %{sat_hi:.1f}")
    print(f"  clamp DIŞINDA kalan: %{disari:.1f}  "
          f"-> {'CLAMP YOKTU (bu kod sürümünde)' if disari>5 else 'clamp aktifti'}")

    # açı hatasına çevir
    p_y = -cmd/KP_ALT
    stab_y = p_y + np.sign(p_y)*DEADZONE
    print(f"  geri çözülen stab_error_y_deg: medyan {np.median(stab_y):+.2f}° | "
          f"ort {stab_y.mean():+.2f}° | std {stab_y.std():.2f}°")
    print(f"  NOT: bu KAPALI DÖNGÜ artık hatasıdır, (b+eps) DEĞİLDİR. Plant bir")
    print(f"       integratör olduğu için P-kontrol kalıcı hatayı sıfıra sürer;")
    print(f"       (b+eps) sapması hata sinyalinde değil, UÇAĞIN HEDEFE GÖRE")
    print(f"       KONUMUNDA görünür. (b+eps) ancak ham piksel verisiyle ölçülür.")

    # --- YATAY EKSEN (karşılaştırma) ---
    if len(hc):
        yaw = np.interp(hc[:,0], at[:,0], at[:,3])
        ch = np.array([wrap180(a-b) for a,b in zip(hc[:,1], yaw)])
        ok2 = np.abs(ch) < 40
        ch = ch[ok2]
        p_x = ch/KP_HDG
        stab_x = p_x + np.sign(p_x)*DEADZONE
        print(f"\n  --- YATAY EKSEN ---")
        print(f"  cmd_head_deg: medyan {np.median(ch):+.2f}° | std {ch.std():.2f}")
        print(f"  geri çözülen stab_error_x_deg: medyan {np.median(stab_x):+.2f}° | "
              f"std {stab_x.std():.2f}°")
        print(f"  (aynı kapalı-döngü uyarısı geçerli)")

    # --- DE-ROTASYON TESTİ: stab_error_y, pitch ile korele mi? ---
    pit = np.interp(tt, at[:,0], at[:,1])
    if len(tt) > 200 and pit.std() > 0.5:
        # yavaş bileşeni at (hedefin gerçek hareketi), hızlı salınıma bak
        w = 31
        k = np.ones(w)/w
        hp = lambda v: v - np.convolve(v, k, mode='same')
        a_, b_ = hp(stab_y), hp(pit)
        if b_.std() > 1e-6:
            sl = np.polyfit(b_, a_, 1)[0]; r = np.corrcoef(b_, a_)[0,1]
            print(f"\n  --- DE-ROTASYON TESTİ ---")
            print(f"  d(stab_error_y)/d(pitch) = {sl:+.3f}  (BEKLENEN 0.000) | r = {r:+.3f}")
            print(f"  UYARI: kapalı döngüde bu test KONFONDE. Hata -> irtifa komutu")
            print(f"  -> pitch nedenselliği zaten negatif korelasyon üretir. Temiz")
            print(f"  rijitlik testi için ham piksel gerekir: tools/kamera_aci_testi.py")

    # --- sapma zamanla kayıyor mu ---
    q = np.array_split(np.arange(len(stab_y)), 4)
    meds = [float(np.median(stab_y[i])) for i in q if len(i)>20]
    if len(meds)>1:
        print(f"  çeyrek medyanları: " + " ".join(f"{v:+.2f}°" for v in meds)
              + f"  (yayılım {max(meds)-min(meds):.2f}°)")

if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    for p in sys.argv[1:]: analiz(p)
