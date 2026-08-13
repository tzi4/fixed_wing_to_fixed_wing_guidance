#!/usr/bin/env python3
"""
PID GRAFIKLERI + DURUST HUKUM — goat_cam_offset.py CSV'sinden.

NEDEN VAR: "durum nasil?" -> "iyi" cevabi, ucak yatay eksende limit-cevrimi
yaparken bile verilebiliyordu. Bu arac yorum yapmaz, OLCER: her eksende hata
nasil degisiyor, komut ne kadar kuvvetli uygulaniyor, komut doyuyor mu,
salinimin PERIYODU ne. Yer gercegi (tools/gercek_konum_logger.py) verilirse
en kritik ayrimi da yapar: hedef DUMDUZ giderken biz saliniyor muyuz?

Kullanim:
    python3 tools/pid_grafik.py ucus_loglari/flight_log_guided_*.csv
    python3 tools/pid_grafik.py <gudum.csv> --gercek ucus_loglari/gercek_konum_*.csv
    python3 tools/pid_grafik.py <gudum.csv> --outdir reports/pid_2026-07-29
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- Referans taban cizgileri (olculmus, uydurma degil) ---
# Gercek ucus (july_1_logs 79/80): cmd_heading std 12.3-12.8 deg, periyot 4.5-6.0 s.
# Sim'de IYI kol (Erenimbus A/B, 2026-07-28): cmd std 0.50, |hata| 0.44, osilasyon yok.
# Sim'de KOTU kol (bumblebee):              cmd std 18.65, |hata| 5.74, periyot 1.99 s.
REF = {
    'gercek_cmd_std': (12.3, 12.8),
    'gercek_periyot': (4.5, 6.0),
    'iyi_hata_abs': 0.44,
    'kotu_hata_abs': 5.74,
    'kotu_periyot': 1.99,
}

# --- SALINIM KAPISI: "salinim var" demeye ne zaman hakkimiz var? ---
# Yanlis alarm gozlendi: dikey eksende ort |hata| 0.30 deg, zamanin %94.2'si
# deadzone icinde, komut std 0.25 m, doygunluk %0.1 iken rapor "baskin periyot
# 1.25 s / isaret degisimi 178.4 /dk" yaziyordu. O sayilar sifir etrafindaki
# OLCUM GURULTUSUNUN sifir gecisleriydi; FFT tepesi toplam gucun yalnizca
# %6.7'sini tasiyordu. Temiz kolda kurt masali anlatan arac, aracsizliktan kotudur.
# Bu yuzden periyot / isaret-degisimi rakamlari IKI kapidan gecmeden "salinim"
# diye sunulmaz:
#
# 1) GENLIK kapisi. Olcut: ortalamasi cikarilmis hatanin std'si. Sinusoidde
#    tepe = std*sqrt(2) oldugundan std >= deadzone demek tepe >= 1.41*deadzone,
#    yani sinyal deadzone bandini gercekten terk ediyor demek. std, tek karelik
#    tespit sicramalarindan sismesin diye %1-%99 kirpilmis sinyalde hesaplanir.
#    Olculen degerler (deadzone 0.4 deg): temiz kol dikey 0.19, yatay 0.08
#    (KALIR); T1A dikey 1.08, bumblebee yatay 5.56, dikey 2.05 (GECER).
# 2) BASKINLIK kapisi. FFT tepesinin (+-2 bin) toplam guce orani >= %20 olmali.
#    Beyaz gurultude bu oran ~%1 mertebesindedir; olculmus gercek salinimlar
#    %26-%75 (T1A dikey %74.6, bumblebee yatay %30.2, dikey %26.5), yanlis alarm
#    veren temiz kol ise %6.7 uretmisti. %20 esigi ikisinin ortasinda ve gurultu
#    tabanindan bir kat buyuklugu uzakta.
#
# Ayrica isaret degisimi HISTEREZIS ile sayilir: bir yon degisimi ancak sinyal
# yeni yonde +-band'i asarsa sayilir (band = max(deadzone, genlik/2)). Gurultu
# bandindan hic cikmayan sifir gecisleri sayilmaz. Kapisiz (ham) sayilar yine
# raporda durur ama "TESHIS DEGERI YOK" diye isaretlenir; hicbir sey silinmez.
GENLIK_ESIK_KAT = 1.0      # genlik esigi = KAT * deadzone
BASKIN_ESIK_PCT = 20.0     # tepe +-2 bin gucunun toplam guce orani [%]
MIN_GECIS = 3              # periyot iddiasi icin gereken en az band disi yon degisimi


def read_csv(path):
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"HATA: {path} bos.")
    cols = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            v = (r.get(k) or '').strip()
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(np.nan)
        cols[k] = np.array(vals)
    cols['_raw'] = rows
    return cols


def col(d, name, default=None):
    v = d.get(name)
    if v is None or np.all(np.isnan(v)):
        return default
    return v


def _kirpilmis_std(xc):
    """%1-%99 kirpilmis std — tek karelik tespit sicramasi genligi sismesin."""
    if len(xc) < 5:
        return float(np.std(xc))
    lo, hi = np.percentile(xc, [1.0, 99.0])
    return float(np.std(np.clip(xc, lo, hi)))


def _histerezis_gecisler(t, x, band):
    """Sinyalin +-band'i ASARAK yon degistirdigi anlar.

    Duz sifir gecisi sayimi gurultuyu salinim sayar (temiz kolda 178 /dk cikti).
    Burada bir yon degisimi ancak sinyal yeni yonde band'i asarsa sayilir;
    gurultu bandindan hic cikmayan gecisler yok sayilir.
    """
    ts = []
    durum = 0
    for i in range(len(x)):
        v = x[i]
        if v > band and durum <= 0:
            if durum != 0:
                ts.append(t[i])
            durum = 1
        elif v < -band and durum >= 0:
            if durum != 0:
                ts.append(t[i])
            durum = -1
    return np.array(ts)


def osilasyon_olc(t, x, deadzone):
    """Salinim olcumu + KAPI karari (bkz. dosya basindaki SALINIM KAPISI notu).

    'salinim_var' False iken periyot ve isaret-degisimi sayilari salinim
    gostergesi DEGILDIR; 'kisa_gerekce'/'gerekce' hangi kapinin kaldigini soyler.
    """
    s = {
        'genlik': None, 'genlik_esigi': GENLIK_ESIK_KAT * deadzone,
        'genlik_tamam': False, 'baskin_tamam': False, 'salinim_var': False,
        'periyot_zc': None, 'periyot_fft': None, 'tepe_guc_pct': None,
        'gecis_sayisi': 0, 'isaret_degisim_dk': None, 'band': None,
        'periyot_zc_ham': None, 'isaret_degisim_dk_ham': None,
        'sure_s': 0.0, 'gerekce': 'olcum yapilamadi',
        'kisa_gerekce': 'olcum yapilamadi',
    }
    good = ~np.isnan(x)
    t, x = t[good], x[good]
    if len(x) < 20 or len(t) < 2:
        s['gerekce'] = s['kisa_gerekce'] = 'olcum yapilamadi (ornek < 20)'
        return s
    dur = float(t[-1] - t[0])
    s['sure_s'] = dur
    xc = x - np.mean(x)
    genlik = _kirpilmis_std(xc)
    s['genlik'] = genlik
    s['genlik_tamam'] = genlik >= s['genlik_esigi']
    if genlik < 1e-9:
        s['gerekce'] = s['kisa_gerekce'] = 'salinim yok (sinyal sabit)'
        return s

    # ~0.3 s'lik hareketli ortalama: kare kare olcum gurultusu yon degisimi saymasin
    dt_med = float(np.median(np.diff(t)))
    w = max(3, int(round(0.3 / dt_med))) if dt_med > 0 else 3
    if w % 2 == 0:
        w += 1
    if len(xc) > w:
        kern = np.ones(w) / w
        xs = np.convolve(xc, kern, mode='same')
        xs[:w] = xc[:w]; xs[-w:] = xc[-w:]
    else:
        xs = xc

    # HAM (kapisiz) sayilar — eski davranis. Silinmez, ama teshis degeri yoktur.
    isaret = np.sign(xs)
    ham_idx = np.where(np.diff(isaret) != 0)[0]
    if len(ham_idx) >= 3:
        s['periyot_zc_ham'] = 2.0 * float(np.median(np.diff(t[ham_idx])))
    if dur > 0:
        s['isaret_degisim_dk_ham'] = len(ham_idx) / dur * 60.0

    # HISTEREZIS: gurultu bandindan cikmayan gecisler sayilmaz
    band = max(deadzone, 0.5 * genlik)
    s['band'] = band
    ts = _histerezis_gecisler(t, xs, band)
    s['gecis_sayisi'] = int(len(ts))
    if dur > 0:
        s['isaret_degisim_dk'] = len(ts) / dur * 60.0
    if len(ts) >= MIN_GECIS:
        # ardisik band disi yon degisimleri arasi = yarim periyot
        s['periyot_zc'] = 2.0 * float(np.median(np.diff(ts)))

    # FFT (esit araliga yeniden orneklenmis)
    if len(t) > 32 and dur > 1e-6:
        n = min(4096, max(64, len(t)))
        ti = np.linspace(t[0], t[-1], n)
        xi = np.interp(ti, t, xc)
        dt = ti[1] - ti[0]
        sp = np.abs(np.fft.rfft(xi * np.hanning(n))) ** 2
        fr = np.fft.rfftfreq(n, dt)
        m = fr > 0.05  # 20 s'den uzun trendleri ele
        if np.any(m) and np.sum(sp[m]) > 0:
            k = int(np.argmax(sp[m]))
            f0 = fr[m][k]
            # tepe +- 2 bin'deki gucun toplam guce orani
            spm = sp[m]
            lo, hi = max(0, k - 2), min(len(spm), k + 3)
            s['tepe_guc_pct'] = float(100.0 * np.sum(spm[lo:hi]) / np.sum(spm))
            if f0 > 0:
                s['periyot_fft'] = 1.0 / f0
    s['baskin_tamam'] = (s['tepe_guc_pct'] is not None
                         and s['tepe_guc_pct'] >= BASKIN_ESIK_PCT)

    # --- KAPI KARARI: her iki kosul da ayri ayri yargilanir ve raporlanir ---
    if not s['genlik_tamam']:
        s['kisa_gerekce'] = 'salinim yok (genlik deadzone altinda)'
        s['gerekce'] = ('GENLIK kapisi KALDI: kirpilmis std %.2f deg < esik %.2f deg '
                        '(= deadzone). Sinyal olcum gurultusu bandindan cikmiyor.'
                        % (genlik, s['genlik_esigi']))
    elif s['tepe_guc_pct'] is None:
        s['kisa_gerekce'] = 'baskin frekans yok (FFT yapilamadi)'
        s['gerekce'] = 'BASKINLIK kapisi KALDI: kayit FFT icin cok kisa.'
    elif not s['baskin_tamam']:
        s['kisa_gerekce'] = ('baskin frekans yok (tepe gucu %%%.1f)' % s['tepe_guc_pct'])
        s['gerekce'] = ('BASKINLIK kapisi KALDI: FFT tepesi (+-2 bin) toplam gucun '
                        'yalnizca %%%.1f\'ini tasiyor, esik %%%.0f. Genis bantli '
                        'gurultu, tek frekansli salinim degil.'
                        % (s['tepe_guc_pct'], BASKIN_ESIK_PCT))
    elif len(ts) < MIN_GECIS:
        s['kisa_gerekce'] = 'salinim yok (band disi yon degisimi yetersiz)'
        s['gerekce'] = ('Band disi yon degisimi %d < %d: sinyal +-%.2f deg bandini '
                        'tekrarli bicimde terk etmiyor.' % (len(ts), MIN_GECIS, band))
    else:
        s['salinim_var'] = True
        s['kisa_gerekce'] = 'salinim VAR'
        s['gerekce'] = ('GECTI: genlik %.2f >= %.2f deg VE tepe gucu %%%.1f >= %%%.0f VE '
                        '%d band disi yon degisimi.'
                        % (genlik, s['genlik_esigi'], s['tepe_guc_pct'],
                           BASKIN_ESIK_PCT, len(ts)))
    return s


def axis_stats(t, err, cmd_raw, cmd, sat, deadzone=0.4):
    s = {}
    good = ~np.isnan(err)
    e = err[good]
    s['n'] = int(len(e))
    if len(e) == 0:
        return s
    s['hata_ort_abs'] = float(np.mean(np.abs(e)))
    s['hata_std'] = float(np.std(e))
    s['hata_rms'] = float(np.sqrt(np.mean(e ** 2)))
    s['hata_maks_abs'] = float(np.max(np.abs(e)))
    s['deadzone_ici_pct'] = float(100.0 * np.mean(np.abs(e) < deadzone))
    # salinim metrikleri KAPILI gelir: kapi kalirsa periyot/isaret-degisimi
    # sayilari 'salinim gostergesi degil' diye isaretlenir (bkz. osilasyon_olc)
    s.update(osilasyon_olc(t, err, deadzone))
    dur = float(t[good][-1] - t[good][0]) if len(t[good]) > 1 else 0.0
    s['sure_s'] = dur
    if cmd is not None:
        c = cmd[~np.isnan(cmd)]
        if len(c):
            s['cmd_std'] = float(np.std(c))
            s['cmd_ort_abs'] = float(np.mean(np.abs(c)))
    if sat is not None:
        sv = sat[~np.isnan(sat)]
        if len(sv):
            s['doygunluk_pct'] = float(100.0 * np.mean(sv > 0.5))
    return s


def fmt(v, n=2, suffix=''):
    return '—' if v is None else f"{v:.{n}f}{suffix}"


def eksen_hukmu(ad, s, birim):
    """Tek eksen icin duz hukum.

    Salinim iddiasi YALNIZCA kapi gecildiyse yapilir. Kucuk hata + yuksek
    deadzone doluluk + dusuk doygunluk + kapi kalmasi = SAGLIKLI; belirsiz degil.
    """
    hata = s.get('hata_ort_abs')
    dz = s.get('deadzone_ici_pct')
    sat = s.get('doygunluk_pct')
    per = s.get('periyot_fft') if s.get('periyot_fft') is not None else s.get('periyot_zc')
    sorun = []
    if s.get('salinim_var'):
        if per is not None and per <= 3.0:
            sorun.append('**LIMIT-CEVRIM**: %.2f s periyotlu GERCEK salinim '
                         '(genlik %.2f deg, tepe gucu %%%.1f, dakikada %.1f band disi '
                         'yon degisimi); kotu kol referansi ~%.2f s'
                         % (per, s.get('genlik') or float('nan'),
                            s.get('tepe_guc_pct') or float('nan'),
                            s.get('isaret_degisim_dk') or float('nan'), REF['kotu_periyot']))
        else:
            sorun.append('yavas salinim var: periyot %s s, genlik %.2f deg, tepe gucu %%%.1f '
                         '(limit-cevrim bandinda degil)'
                         % (fmt(per), s.get('genlik') or float('nan'),
                            s.get('tepe_guc_pct') or float('nan')))
    if hata is not None and hata > 2.0 * REF['iyi_hata_abs']:
        sorun.append('ortalama |hata| %.2f deg, iyi kol %.2f deg (kotu kol %.2f)'
                     % (hata, REF['iyi_hata_abs'], REF['kotu_hata_abs']))
    if sat is not None and sat > 10.0:
        sorun.append("komut zamanin %%%.1f'sinde clamp'e dayaniyor" % sat)
    if dz is not None and dz < 50.0:
        sorun.append('deadzone icinde gecen sure yalnizca %%%.1f' % dz)
    if not sorun:
        return ('- **%s: SAGLIKLI.** ortalama |hata| %s deg, zamanin %%%s\'si deadzone icinde, '
                'komut std %s %s, doygunluk %%%s. %s'
                % (ad, fmt(hata), fmt(dz, 1), fmt(s.get('cmd_std')), birim, fmt(sat, 1),
                   s.get('gerekce', '')))
    return '- **%s: SORUNLU.** %s.' % (ad, '; '.join(sorun))


def plot_axis(outdir, tag, title, t, err, terms, cmd_raw, cmd, extra, deadzone):
    n = 3 + (1 if extra else 0)
    fig, ax = plt.subplots(n, 1, figsize=(13, 3.0 * n), sharex=True)
    fig.suptitle(title, fontsize=13, fontweight='bold')

    ax[0].plot(t, err, lw=0.9, color='#c0392b', label='hata')
    ax[0].axhspan(-deadzone, deadzone, color='#95a5a6', alpha=0.25, label=f'deadzone ±{deadzone}')
    ax[0].axhline(0, color='k', lw=0.6)
    ax[0].set_ylabel('hata [deg]'); ax[0].legend(loc='upper right', fontsize=8)
    ax[0].grid(alpha=0.3)

    for name, series, c in terms:
        if series is not None:
            ax[1].plot(t, series, lw=0.9, label=name, color=c)
    ax[1].axhline(0, color='k', lw=0.6)
    ax[1].set_ylabel('PID terimleri'); ax[1].legend(loc='upper right', fontsize=8)
    ax[1].grid(alpha=0.3)

    if cmd_raw is not None:
        ax[2].plot(t, cmd_raw, lw=0.8, color='#7f8c8d', label='ham komut (clamp oncesi)')
    if cmd is not None:
        ax[2].plot(t, cmd, lw=1.1, color='#2980b9', label='uygulanan komut')
    ax[2].axhline(0, color='k', lw=0.6)
    ax[2].set_ylabel('komut'); ax[2].legend(loc='upper right', fontsize=8)
    ax[2].grid(alpha=0.3)

    if extra:
        for name, series, c in extra:
            if series is not None:
                ax[3].plot(t, series, lw=0.9, label=name, color=c)
        ax[3].set_ylabel('ek'); ax[3].legend(loc='upper right', fontsize=8)
        ax[3].grid(alpha=0.3)

    ax[-1].set_xlabel('gecen sure [s]')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p = os.path.join(outdir, f'{tag}.png')
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description='Gudum CSV -> PID grafikleri + durust hukum')
    ap.add_argument('csv', help='goat_cam_offset.py flight_log_guided_*.csv')
    ap.add_argument('--gercek', default=None, help='tools/gercek_konum_logger.py CSV (yer gercegi)')
    ap.add_argument('--outdir', default=None, help='cikti klasoru')
    ap.add_argument('--deadzone', type=float, default=0.4)
    args = ap.parse_args()

    d = read_csv(args.csv)
    outdir = args.outdir or os.path.join('reports', 'pid_' + os.path.basename(args.csv).replace('.csv', ''))
    os.makedirs(outdir, exist_ok=True)

    t = col(d, 'elapsed_s')
    if t is None:
        sys.exit('HATA: elapsed_s sutunu yok.')
    state = [r.get('system_state', '') for r in d['_raw']]
    tracking = np.array([s == 'TRACKING' for s in state])
    ex = col(d, 'stab_error_x_deg'); ey = col(d, 'stab_error_y_deg')
    if ex is None:
        sys.exit('HATA: stab_error_x_deg yok — bu bir gudum CSV\'si mi?')

    figs = []
    # --- YATAY EKSEN (heading) ---
    figs.append(plot_axis(
        outdir, 'eksen_yatay', 'YATAY EKSEN (ex -> heading)', t, ex,
        [('P', col(d, 'p_term_heading'), '#27ae60'),
         ('I', col(d, 'i_term_heading'), '#f39c12'),
         ('D', col(d, 'd_term_heading'), '#8e44ad')],
        col(d, 'cmd_head_raw_deg'), col(d, 'cmd_heading_deg'),
        [('donus hizi [deg/s]', col(d, 'heading_rate_dps'), '#16a085'),
         ('roll [deg]', col(d, 'roll_deg'), '#c0392b')], args.deadzone))

    # --- DIKEY EKSEN (irtifa) ---
    figs.append(plot_axis(
        outdir, 'eksen_dikey', 'DIKEY EKSEN (ey -> irtifa)', t, ey,
        [('P', col(d, 'p_term_alt'), '#27ae60'),
         ('I', col(d, 'i_term_alt'), '#f39c12'),
         ('D', col(d, 'd_term_alt'), '#8e44ad')],
        col(d, 'cmd_alt_raw_m'), col(d, 'cmd_alt_m'),
        [('mevcut irtifa [m]', col(d, 'current_alt_m'), '#2c3e50'),
         ('komut irtifa [m]', col(d, 'target_alt_m'), '#2980b9')], args.deadzone))

    # --- HEDEF GEOMETRISI + HIZ ---
    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle('HEDEF GEOMETRISI ve HIZ', fontsize=13, fontweight='bold')
    for name, key, c in (('yatay kaplama %', 'coverage_w_pct', '#2980b9'),
                         ('dikey kaplama %', 'coverage_h_pct', '#8e44ad')):
        v = col(d, key)
        if v is not None:
            ax[0].plot(t, v, lw=0.9, label=name, color=c)
    ax[0].set_ylabel('kaplama [%]'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    sq = col(d, 'sqrt_area_px')
    if sq is None:
        ar = col(d, 'target_area_px')
        sq = np.sqrt(np.clip(ar, 0, None)) if ar is not None else None
    if sq is not None:
        ax[1].plot(t, sq, lw=0.9, color='#d35400', label='sqrt(alan) [px]')
    ax[1].set_ylabel('sqrt(alan) [px]'); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    for name, key, c in (('airspeed', 'airspeed_ms', '#c0392b'),
                         ('groundspeed', 'groundspeed_ms', '#27ae60')):
        v = col(d, key)
        if v is not None:
            ax[2].plot(t, v, lw=0.9, label=name, color=c)
    thr = col(d, 'throttle_pct')
    if thr is not None:
        a2 = ax[2].twinx(); a2.plot(t, thr, lw=0.7, color='#7f8c8d', alpha=0.7, label='gaz %')
        a2.set_ylabel('gaz [%]')
    ax[2].set_ylabel('hiz [m/s]'); ax[2].set_xlabel('gecen sure [s]')
    ax[2].legend(fontsize=8, loc='upper left'); ax[2].grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p = os.path.join(outdir, 'hedef_ve_hiz.png'); fig.savefig(p, dpi=110); plt.close(fig)
    figs.append(p)

    # --- ISTATISTIK ---
    sx = axis_stats(t, ex, col(d, 'cmd_head_raw_deg'), col(d, 'cmd_heading_deg'),
                    col(d, 'sat_head'), args.deadzone)
    sy = axis_stats(t, ey, col(d, 'cmd_alt_raw_m'), col(d, 'cmd_alt_m'),
                    col(d, 'sat_alt'), args.deadzone)

    # --- YER GERCEGI CAPRAZ KONTROLU ---
    gt_lines = []
    verdict_gt = None
    if args.gercek and os.path.exists(args.gercek):
        g = read_csv(args.gercek)
        tyr = col(g, 'target_yaw_rate_dps')
        man = col(g, 'target_maneuvering')
        rng = col(g, 'range_m')
        be = col(g, 'bearing_error_deg')
        if tyr is not None:
            duz_pct = float(100.0 * np.mean(np.abs(tyr[~np.isnan(tyr)]) < 2.0))
            gt_lines.append(f"- Hedefin donus hizi medyani: **{np.nanmedian(np.abs(tyr)):.2f} deg/s**; "
                            f"zamanin **%{duz_pct:.1f}**'inde hedef DUZ ucuyor (|yaw rate| < 2 deg/s)")
            if rng is not None:
                gt_lines.append(f"- Menzil: {np.nanmin(rng):.0f}–{np.nanmax(rng):.0f} m "
                                f"(medyan {np.nanmedian(rng):.0f} m)")
            if be is not None:
                gt_lines.append(f"- Yer gerceginden yon hatasi: ort |{np.nanmean(np.abs(be)):.2f}| deg, "
                                f"std {np.nanstd(be):.2f} deg")
            osc = (sx.get('hata_ort_abs', 0) > 2.0) or (sx.get('cmd_std', 0) > 5.0)
            if duz_pct > 70.0 and osc:
                verdict_gt = ("**SORUN BIZDE.** Hedef zamanin %%%.1f'inde dumduz ucuyor ama bizim yatay "
                              "hatamiz ort |%.2f| deg ve komut std %.2f deg. Hedef manevrasi bunu "
                              "aciklamiyor — kontrolcu kaynakli." %
                              (duz_pct, sx.get('hata_ort_abs', float('nan')), sx.get('cmd_std', float('nan'))))
            elif duz_pct > 70.0:
                verdict_gt = ("Hedef cogunlukla duz ucuyor ve bizim hatamiz da kucuk — bu kolda "
                              "kontrolcu saglikli gorunuyor.")
            else:
                verdict_gt = ("Hedef zamanin buyuk kisminda manevra yapiyor; hatanin ne kadari "
                              "kontrolcuden ne kadari takip zorlugundan ayirt edilemez.")
        else:
            gt_lines.append("- UYARI: yer gercegi CSV'sinde target_yaw_rate_dps yok.")

    # --- HUKUM ---
    def sal(s, deger, n=2, birim=' s'):
        """Kapi gecilmediyse ciplak sayi yerine acik ifade yaz (ham deger kalir)."""
        if s.get('salinim_var'):
            return '—' if deger is None else f"**{deger:.{n}f}{birim}**"
        gk = s.get('kisa_gerekce', 'salinim yok')
        if deger is None:
            return gk
        return f"{gk} — ham {deger:.{n}f}{birim}, salinim gostergesi DEGIL"

    def axis_block(ad, s, birim):
        L = [f"### {ad}", "",
             f"| olcut | deger | referans |", "|---|---|---|",
             f"| ortalama \\|hata\\| | **{fmt(s.get('hata_ort_abs'))} deg** | iyi kol 0.44 / kotu kol 5.74 |",
             f"| hata std | {fmt(s.get('hata_std'))} deg | — |",
             f"| hata RMS | {fmt(s.get('hata_rms'))} deg | — |",
             f"| maks \\|hata\\| | {fmt(s.get('hata_maks_abs'))} deg | — |",
             f"| deadzone icinde gecen sure | %{fmt(s.get('deadzone_ici_pct'), 1)} | yuksek = sakin |",
             f"| salinim genligi (kirpilmis std) | {fmt(s.get('genlik'))} deg | esik {fmt(s.get('genlik_esigi'))} deg (= deadzone); altinda = olcum gurultusu |",
             f"| **SALINIM KAPISI** | {'**GECTI**' if s.get('salinim_var') else '**KALDI**'} | {s.get('gerekce', '—')} |",
             f"| baskin periyot (band disi yon degisimi) | {sal(s, s.get('periyot_zc'), 2, ' s')} | gercek ucus 4.5–6.0 s, limit-cevrim ~2.0 s |",
             f"| baskin periyot (FFT) | {sal(s, s.get('periyot_fft'), 2, ' s')} | tepe gucun payi %{fmt(s.get('tepe_guc_pct'), 1)}, esik %{BASKIN_ESIK_PCT:.0f} |",
             f"| isaret degisimi (histerezis, band ±{fmt(s.get('band'))} deg) | {sal(s, s.get('isaret_degisim_dk'), 1, ' /dk')} | yalnizca bandi TERK EDEN yon degisimleri sayilir ({s.get('gecis_sayisi', 0)} adet) |",
             f"| ham (kapisiz) sifir gecisi | {fmt(s.get('periyot_zc_ham'))} s / {fmt(s.get('isaret_degisim_dk_ham'), 1)} /dk | gurultuyu de sayar — kayit icin, TESHIS DEGERI YOK |",
             f"| komut std | {fmt(s.get('cmd_std'))} {birim} | gercek ucus 12.3–12.8 (heading) |",
             f"| komut doygunlugu | %{fmt(s.get('doygunluk_pct'), 1)} | yuksek = clamp'e dayaniyor |",
             f"| ornek / sure | {s.get('n', 0)} / {fmt(s.get('sure_s'), 1)} s | — |", ""]
        return L

    md = [f"# PID hukum raporu", "",
          f"- Gudum CSV: `{args.csv}`",
          f"- Yer gercegi: `{args.gercek}`" if args.gercek else "- Yer gercegi: **YOK** (tools/gercek_konum_logger.py ile toplanmali)",
          f"- TRACKING oranı: %{100.0 * np.mean(tracking):.1f} ({int(np.sum(tracking))}/{len(tracking)} kare)",
          ""]
    md += axis_block('YATAY EKSEN (ex -> heading)', sx, 'deg')
    md += axis_block('DIKEY EKSEN (ey -> irtifa)', sy, 'm')
    if gt_lines:
        md += ["### Yer gercegi caprazlamasi", ""] + gt_lines + [""]
    md += ["### HUKUM", ""]
    md += [eksen_hukmu('YATAY EKSEN (ex -> heading)', sx, 'deg'),
           eksen_hukmu('DIKEY EKSEN (ey -> irtifa)', sy, 'm'), ""]
    md += [f"Salinim kapisi: genlik (kirpilmis std) >= deadzone ({args.deadzone:.2f} deg) VE "
           f"FFT tepe gucu >= %{BASKIN_ESIK_PCT:.0f} VE en az {MIN_GECIS} band disi yon degisimi. "
           "Uc kosuldan biri saglanmazsa periyot ve isaret degisimi sayilari olcum "
           "gurultusudur, salinim gostergesi degildir.", ""]
    if verdict_gt:
        md += [verdict_gt, ""]
    else:
        md += ["Yer gercegi verilmedi; 'hedef duz giderken biz saliniyor muyuz?' sorusu "
               "TAM CEVAPLANAMAZ (yukaridaki eksen hukmu yalnizca kendi olcumumuze dayanir). "
               "`tools/gercek_konum_logger.py` ile es zamanli kayit alin.", ""]
    md += ["### Grafikler", ""] + [f"- `{os.path.basename(f)}`" for f in figs]

    rp = os.path.join(outdir, 'HUKUM.md')
    with open(rp, 'w') as fh:
        fh.write("\n".join(md) + "\n")

    print("\n".join(md))
    print(f"\n-> {rp}")
    for f in figs:
        print(f"-> {f}")


if __name__ == '__main__':
    main()
