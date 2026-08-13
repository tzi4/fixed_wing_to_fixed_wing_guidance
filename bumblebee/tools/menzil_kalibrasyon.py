#!/usr/bin/env python3
r"""
MENZIL KALIBRASYONU — gorunur boyuttan menzil kestirimi (CEVRIMDISI).

NEDEN VAR: gudum kodunun hedefin konumunu okumasi YASAK (test butunlugu).
Ama dikey nisan ofsetini menzile duyarli yapmak icin bir menzil kestirimine
ihtiyac var. Cozum: kalibrasyonu BURADA, cevrimdisi yapmak. Sabit boyutlu bir
hedefin goruntudeki boyu s (px) menzille ters orantilidir:

    s ~ k / R   ->   R ~ k / s

Bu arac k'yi yer gercegi (tools/gercek_konum_logger.py) ile eslesmis gudum
kayitlarindan uydurur ve JSON'a yazar. Calisma aninda gudum SADECE kamera
verisini kullanarak menzili kestirebilir; hedefin konumunu hic okumaz.

Arac yorum yapmaz, OLCER: uc aday model uydurulur, hepsi ZAMANA gore ayrilmis
tutulan test kumesinde sinanir (rastgele ayirma sizdirir: ardisik kareler
birbirinin ayni). Model genellemiyorsa bunu acikca soyler.

Kullanim:
    python3 tools/menzil_kalibrasyon.py --pair gudum.csv:gercek.csv
    python3 tools/menzil_kalibrasyon.py --pair a_gudum.csv:a_gercek.csv \
                                        --pair b_gudum.csv:b_gercek.csv
    python3 tools/menzil_kalibrasyon.py --pair g.csv:r.csv --azami-fark 0.10 \
                                        --json reports/menzil_kalibrasyon.json
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Menzil kutulari: yakin / orta / uzak (m). Hata bunlarin her birinde ayri
# raporlanir, cunku 1/s modeli uzak menzilde dogal olarak daha gurultuludur.
KUTULAR = (('yakin (<100 m)', 0.0, 100.0),
           ('orta (100-300 m)', 100.0, 300.0),
           ('uzak (>300 m)', 300.0, float('inf')))

ASGARI_ORNEK = 10      # bunun altinda uydurma yapmak anlamsiz
ASGARI_TEST = 5        # tutulan test kumesi bundan kucukse hukum verilmez


# --------------------------------------------------------------------------
# CSV okuma (pid_grafik.py ile ayni yaklasim)
# --------------------------------------------------------------------------
def csv_oku(path):
    if not os.path.exists(path):
        sys.exit(f"HATA: dosya bulunamadi: {path}")
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"HATA: {path} bos (yalnizca baslik satiri olabilir).")
    cols = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            v = (r.get(k) or '').strip()
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(np.nan)
        cols[k] = np.array(vals, dtype=float)
    cols['_raw'] = rows
    return cols


def sut(d, name, default=None):
    """Sutunu dondur; yoksa ya da tamami NaN ise default."""
    v = d.get(name)
    if v is None or len(v) == 0 or np.all(np.isnan(v)):
        return default
    return v


def zaman_araligi(t):
    g = t[np.isfinite(t)]
    if len(g) == 0:
        return None, None
    return float(np.min(g)), float(np.max(g))


# --------------------------------------------------------------------------
# Bir gudum + yer gercegi ciftini EN YAKIN zaman damgasina gore birlestir
# --------------------------------------------------------------------------
def cift_isle(gudum_path, gercek_path, azami_fark):
    g = csv_oku(gudum_path)
    r = csv_oku(gercek_path)

    tg = sut(g, 'timestamp')
    if tg is None:
        sys.exit(f"HATA: {gudum_path} icinde 'timestamp' (unix) sutunu yok — "
                 "bu bir gudum CSV'si mi? elapsed_s ile birlestirme yapilamaz.")
    tr = sut(r, 'timestamp')
    if tr is None:
        sys.exit(f"HATA: {gercek_path} icinde 'timestamp' (unix) sutunu yok — "
                 "bu bir gercek_konum_logger.py CSV'si mi?")

    # --- gorunur boyut: sqrt_area_px yoksa target_area_px'ten turet ---
    s = sut(g, 'sqrt_area_px')
    boyut_kaynagi = 'sqrt_area_px'
    if s is None:
        alan = sut(g, 'target_area_px')
        if alan is None:
            sys.exit(f"HATA: {gudum_path} icinde ne 'sqrt_area_px' ne de "
                     "'target_area_px' var; gorunur boyut hesaplanamaz.")
        s = np.sqrt(np.clip(alan, 0.0, None))
        boyut_kaynagi = 'sqrt(target_area_px)'
    bw = sut(g, 'bbox_w')
    bh = sut(g, 'bbox_h')
    if bw is None or bh is None:
        sys.exit(f"HATA: {gudum_path} icinde bbox_w/bbox_h sutunu yok.")

    durum = np.array([(row.get('system_state') or '').strip() for row in g['_raw']])
    izleme = (durum == 'TRACKING')

    gecerli = (izleme & np.isfinite(tg) & np.isfinite(s) & np.isfinite(bw) &
               np.isfinite(bh) & (s > 0) & (bw > 0) & (bh > 0))
    n_ham = len(tg)
    n_gecerli = int(np.sum(gecerli))

    # --- yer gercegi menzili: range_m yoksa range_xy_m + alt_diff_m'den ---
    R = sut(r, 'range_m')
    menzil_kaynagi = 'range_m'
    if R is None:
        rxy = sut(r, 'range_xy_m')
        dz = sut(r, 'alt_diff_m')
        if rxy is None or dz is None:
            sys.exit(f"HATA: {gercek_path} icinde 'range_m' yok ve "
                     "range_xy_m/alt_diff_m ile de hesaplanamiyor.")
        R = np.hypot(rxy, dz)
        menzil_kaynagi = 'hypot(range_xy_m, alt_diff_m)'

    r_gecerli = np.isfinite(tr) & np.isfinite(R) & (R > 0)
    tr_g, R_g = tr[r_gecerli], R[r_gecerli]
    duzen = np.argsort(tr_g)
    tr_g, R_g = tr_g[duzen], R_g[duzen]

    ozet = {
        'gudum_csv': os.path.abspath(gudum_path),
        'gercek_csv': os.path.abspath(gercek_path),
        'gudum_satir': n_ham,
        'tracking_gecerli_satir': n_gecerli,
        'gercek_satir': int(len(tr_g)),
        'boyut_kaynagi': boyut_kaynagi,
        'menzil_kaynagi': menzil_kaynagi,
        'gudum_zaman': zaman_araligi(tg),
        'gercek_zaman': zaman_araligi(tr_g),
        'eslesen': 0,
        'atilan_zaman_farki': 0,
    }

    bos = np.array([])
    if n_gecerli == 0 or len(tr_g) == 0:
        return bos, bos, bos, bos, ozet

    tg_s, s_s, bw_s, bh_s = tg[gecerli], s[gecerli], bw[gecerli], bh[gecerli]

    # --- EN YAKIN zaman damgasi eslemesi (searchsorted; ileri/geri karsilastir) ---
    idx = np.searchsorted(tr_g, tg_s)
    sol = np.clip(idx - 1, 0, len(tr_g) - 1)
    sag = np.clip(idx, 0, len(tr_g) - 1)
    d_sol = np.abs(tg_s - tr_g[sol])
    d_sag = np.abs(tg_s - tr_g[sag])
    en_yakin = np.where(d_sol <= d_sag, sol, sag)
    fark = np.minimum(d_sol, d_sag)

    tut = fark <= azami_fark
    ozet['eslesen'] = int(np.sum(tut))
    ozet['atilan_zaman_farki'] = int(np.sum(~tut))
    if ozet['eslesen']:
        ozet['medyan_zaman_farki_s'] = float(np.median(fark[tut]))

    return tg_s[tut], s_s[tut], bh_s[tut], R_g[en_yakin[tut]], ozet


# --------------------------------------------------------------------------
# Modeller — hepsi en kucuk kareler
# --------------------------------------------------------------------------
def fit_ters(x, R):
    """R = k / x  ->  orijinden gecen tek parametreli en kucuk kareler."""
    u = 1.0 / x
    payda = float(np.dot(u, u))
    if not np.isfinite(payda) or payda <= 0:
        return None
    return {'k': float(np.dot(u, R) / payda)}


def fit_ters_ofset(x, R):
    """R = a/x + b  (iki parametreli en kucuk kareler)."""
    A = np.column_stack([1.0 / x, np.ones_like(x)])
    if not np.all(np.isfinite(A)):
        return None
    try:
        c, *_ = np.linalg.lstsq(A, R, rcond=None)
    except np.linalg.LinAlgError:
        return None
    return {'a': float(c[0]), 'b': float(c[1])}


MODELLER = [
    {'ad': 'A', 'formul': 'R = k / sqrt_area_px', 'girdi': 'sqrt_area_px',
     'fit': fit_ters, 'tahmin': lambda p, x: p['k'] / x},
    {'ad': 'B', 'formul': 'R = k / bbox_h', 'girdi': 'bbox_h',
     'fit': fit_ters, 'tahmin': lambda p, x: p['k'] / x},
    {'ad': 'C', 'formul': 'R = a / sqrt_area_px + b', 'girdi': 'sqrt_area_px',
     'fit': fit_ters_ofset, 'tahmin': lambda p, x: p['a'] / x + p['b']},
]


def metrikler(R_ger, R_tah):
    """R^2, RMSE (m), medyan mutlak yuzde hata + menzil kutusu kirilimi."""
    m = np.isfinite(R_ger) & np.isfinite(R_tah)
    R_ger, R_tah = R_ger[m], R_tah[m]
    out = {'n': int(len(R_ger))}
    if len(R_ger) == 0:
        return out
    art = R_tah - R_ger
    ss_res = float(np.sum(art ** 2))
    ss_tot = float(np.sum((R_ger - np.mean(R_ger)) ** 2))
    out['r2'] = (1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None
    out['rmse_m'] = float(np.sqrt(np.mean(art ** 2)))
    out['medyan_mutlak_yuzde'] = float(np.median(np.abs(art) / R_ger * 100.0))
    out['kutular'] = {}
    for ad, lo, hi in KUTULAR:
        k = (R_ger >= lo) & (R_ger < hi)
        if not np.any(k):
            out['kutular'][ad] = {'n': 0}
            continue
        out['kutular'][ad] = {
            'n': int(np.sum(k)),
            'rmse_m': float(np.sqrt(np.mean(art[k] ** 2))),
            'medyan_mutlak_yuzde': float(np.median(np.abs(art[k]) / R_ger[k] * 100.0)),
        }
    return out


def bicim(v, n=2, ek=''):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return '—'
    return f"{v:.{n}f}{ek}"


def param_metni(p):
    if not p:
        return '—'
    return ', '.join(f"{k} = {v:.4f}" for k, v in p.items())


def metrik_satiri(etiket, m):
    if not m or m.get('n', 0) == 0:
        return f"    {etiket:<13} ornek yok"
    return (f"    {etiket:<13} n={m['n']:<6d} R2={bicim(m.get('r2'), 4):<9} "
            f"RMSE={bicim(m.get('rmse_m'), 1)} m   "
            f"medyan |hata|=%{bicim(m.get('medyan_mutlak_yuzde'), 1)}")


def kutu_satirlari(m):
    L = []
    for ad, _, _ in KUTULAR:
        k = (m.get('kutular') or {}).get(ad, {})
        if k.get('n', 0) == 0:
            L.append(f"      {ad:<18} ornek yok")
        else:
            L.append(f"      {ad:<18} n={k['n']:<6d} RMSE={bicim(k['rmse_m'], 1)} m   "
                     f"medyan |hata|=%{bicim(k['medyan_mutlak_yuzde'], 1)}")
    return L


# --------------------------------------------------------------------------
# Grafik
# --------------------------------------------------------------------------
def grafik_ciz(png_yolu, s, bh, R, sonuclar, secilen):
    fig, ax = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle('MENZIL KALIBRASYONU — gorunur boyut vs yer gercegi menzili',
                 fontsize=13, fontweight='bold')
    renk = {'A': '#c0392b', 'B': '#27ae60', 'C': '#8e44ad'}

    # --- 1: menzil vs sqrt(alan) + A ve C egrileri ---
    ax[0].scatter(s, R, s=6, alpha=0.35, color='#2980b9', label='olcum (yer gercegi)')
    xs = np.linspace(max(float(np.min(s)), 1e-6), float(np.max(s)), 400)
    for so in sonuclar:
        if so['girdi'] != 'sqrt_area_px' or so['tum_param'] is None:
            continue
        ax[0].plot(xs, so['tahmin'](so['tum_param'], xs), lw=1.6, color=renk[so['ad']],
                   label=f"{so['ad']}: {so['formul']}  ({param_metni(so['tum_param'])})")
    ax[0].set_xlabel('sqrt(alan) [px]'); ax[0].set_ylabel('menzil [m]')
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    # --- 2: menzil vs bbox_h + B egrisi ---
    ax[1].scatter(bh, R, s=6, alpha=0.35, color='#16a085', label='olcum (yer gercegi)')
    xh = np.linspace(max(float(np.min(bh)), 1e-6), float(np.max(bh)), 400)
    for so in sonuclar:
        if so['girdi'] != 'bbox_h' or so['tum_param'] is None:
            continue
        ax[1].plot(xh, so['tahmin'](so['tum_param'], xh), lw=1.6, color=renk[so['ad']],
                   label=f"{so['ad']}: {so['formul']}  ({param_metni(so['tum_param'])})")
    ax[1].set_xlabel('bbox_h [px]'); ax[1].set_ylabel('menzil [m]')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    # --- 3: artik (tahmin - gercek) vs gercek menzil ---
    for so in sonuclar:
        if so['tum_param'] is None:
            continue
        x = s if so['girdi'] == 'sqrt_area_px' else bh
        art = so['tahmin'](so['tum_param'], x) - R
        vurgu = (so['ad'] == secilen)
        ax[2].scatter(R, art, s=9 if vurgu else 4, alpha=0.55 if vurgu else 0.20,
                      color=renk[so['ad']], label=f"{so['ad']}{' (SECILEN)' if vurgu else ''}")
    ax[2].axhline(0, color='k', lw=0.8)
    for _, lo, hi in KUTULAR:
        if np.isfinite(hi) and hi < float(np.max(R)):
            ax[2].axvline(hi, color='#7f8c8d', lw=0.7, ls='--')
    ax[2].set_xlabel('yer gercegi menzili [m]')
    ax[2].set_ylabel('artik (tahmin - gercek) [m]')
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(png_yolu, dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description='Gorunur boyuttan menzil kestirimi icin CEVRIMDISI kalibrasyon')
    ap.add_argument('--pair', action='append', default=[], metavar='GUDUM.csv:GERCEK.csv',
                    help='gudum CSV ile yer gercegi CSV cifti (tekrarlanabilir)')
    ap.add_argument('--azami-fark', type=float, default=0.15,
                    help='eslesme icin azami zaman damgasi farki [s] (varsayilan 0.15)')
    ap.add_argument('--test-orani', type=float, default=0.30,
                    help='ZAMANIN sonundan tutulan test payi (varsayilan 0.30)')
    ap.add_argument('--json', default=os.path.join('reports', 'menzil_kalibrasyon.json'),
                    help='cikti JSON yolu (PNG ayni ada yazilir)')
    args = ap.parse_args()

    if not args.pair:
        sys.exit("HATA: en az bir --pair GUDUM.csv:GERCEK.csv gerekli.\n"
                 "      Ornek: --pair ucus_loglari/flight_log_guided_X.csv:"
                 "ucus_loglari/gercek_konum_X.csv")
    if not (0.05 <= args.test_orani <= 0.9):
        sys.exit("HATA: --test-orani 0.05 ile 0.90 arasinda olmali.")
    if args.azami_fark <= 0:
        sys.exit("HATA: --azami-fark pozitif olmali.")

    print("=" * 78)
    print("MENZIL KALIBRASYONU")
    print("=" * 78)

    # --- ciftleri oku ve zaman damgasina gore birlestir ---
    S, BH, RR, GRUP = [], [], [], []
    ozetler = []
    for i, p in enumerate(args.pair):
        if ':' not in p:
            sys.exit(f"HATA: --pair bicimi 'gudum.csv:gercek.csv' olmali, alinan: {p}")
        gudum_path, gercek_path = p.rsplit(':', 1)
        _tg, s, bh, R, ozet = cift_isle(gudum_path, gercek_path, args.azami_fark)
        ozetler.append(ozet)
        print(f"\n[cift {i + 1}] {os.path.basename(gudum_path)} + {os.path.basename(gercek_path)}")
        print(f"    gudum satir             : {ozet['gudum_satir']}")
        print(f"    TRACKING + gecerli bbox : {ozet['tracking_gecerli_satir']}")
        print(f"    yer gercegi satir       : {ozet['gercek_satir']}")
        print(f"    eslesen (<= {args.azami_fark:.3f} s)  : {ozet['eslesen']}")
        print(f"    atilan (zaman farki)    : {ozet['atilan_zaman_farki']}")
        if ozet.get('medyan_zaman_farki_s') is not None:
            print(f"    medyan zaman farki      : {ozet['medyan_zaman_farki_s'] * 1000:.1f} ms")
        print(f"    boyut / menzil kaynagi  : {ozet['boyut_kaynagi']} / {ozet['menzil_kaynagi']}")
        if ozet['eslesen'] == 0:
            g0, g1 = ozet['gudum_zaman']
            r0, r1 = ozet['gercek_zaman']
            print("    UYARI: bu ciftten hic satir eslesmedi.")
            if g0 is not None and r0 is not None:
                ortak = min(g1, r1) - max(g0, r0)
                print(f"           gudum penceresi : {g0:.1f} .. {g1:.1f}")
                print(f"           gercek penceresi: {r0:.1f} .. {r1:.1f}")
                if ortak <= 0:
                    print(f"           iki kayit ORTUSMUYOR (aralarinda {-ortak:.1f} s var) — "
                          "bu iki dosya ayni ucustan degil.")
                else:
                    print(f"           {ortak:.1f} s ortusme var; --azami-fark buyutulebilir.")
            continue
        S.append(s); BH.append(bh); RR.append(R)
        GRUP.append(np.full(len(s), i, dtype=int))

    if not S:
        print("\nSONUC: eslesen tek bir satir bile yok; kalibrasyon YAPILAMADI.")
        print("       Gudum ve yer gercegi kayitlarinin AYNI ucustan ve es zamanli "
              "olmasi gerekir.")
        sys.exit(2)

    s_all = np.concatenate(S); bh_all = np.concatenate(BH)
    R_all = np.concatenate(RR); grup = np.concatenate(GRUP)
    n = len(s_all)
    print(f"\nToplam eslesen ornek : {n}")
    print(f"Menzil araligi       : {np.min(R_all):.1f} .. {np.max(R_all):.1f} m "
          f"(medyan {np.median(R_all):.1f} m)")
    print(f"sqrt(alan) araligi   : {np.min(s_all):.2f} .. {np.max(s_all):.2f} px")
    print(f"bbox_h araligi       : {np.min(bh_all):.2f} .. {np.max(bh_all):.2f} px")

    if n < ASGARI_ORNEK:
        print(f"\nSONUC: {n} ornek ile uydurma yapilmaz (asgari {ASGARI_ORNEK}). "
              "Daha uzun es zamanli kayit gerekli.")
        sys.exit(3)

    # --- ZAMANA gore ayirma: her ciftin ilk %70'i fit, son %30'u test ---
    # Rastgele ayirma SIZDIRIR: ardisik kareler neredeyse ayni veridir.
    fit_maske = np.zeros(n, dtype=bool)
    for i in np.unique(grup):
        k = np.where(grup == i)[0]
        if len(k) == 1:
            fit_maske[k] = True
            continue
        kes = int(round(len(k) * (1.0 - args.test_orani)))
        kes = max(1, min(len(k) - 1, kes))
        fit_maske[k[:kes]] = True
    test_maske = ~fit_maske
    n_fit, n_test = int(np.sum(fit_maske)), int(np.sum(test_maske))
    print(f"\nZamana gore ayirma   : fit {n_fit} ornek | tutulan test {n_test} ornek "
          f"(her ciftin son %{args.test_orani * 100:.0f}'i)")
    test_gecerli = n_test >= ASGARI_TEST
    if not test_gecerli:
        print(f"    UYARI: tutulan test kumesi {ASGARI_TEST} ornekten kucuk; "
              "genelleme hukmu VERILEMEZ.")

    girdiler = {'sqrt_area_px': s_all, 'bbox_h': bh_all}

    # --- modelleri uydur ---
    sonuclar = []
    print("\n" + "-" * 78)
    print("MODELLER")
    print("-" * 78)
    for m in MODELLER:
        x = girdiler[m['girdi']]
        tum_p = m['fit'](x, R_all)
        fit_p = m['fit'](x[fit_maske], R_all[fit_maske]) if n_fit else None
        so = {
            'ad': m['ad'], 'formul': m['formul'], 'girdi': m['girdi'],
            'tahmin': m['tahmin'], 'tum_param': tum_p, 'fit_param': fit_p,
            'tum': metrikler(R_all, m['tahmin'](tum_p, x)) if tum_p else {},
            'ic': metrikler(R_all[fit_maske], m['tahmin'](fit_p, x[fit_maske])) if fit_p else {},
            'test': (metrikler(R_all[test_maske], m['tahmin'](fit_p, x[test_maske]))
                     if (fit_p and n_test > 0) else {}),
        }
        sonuclar.append(so)

        print(f"\n  Model {so['ad']}: {so['formul']}")
        if tum_p is None:
            print("    uydurulamadi (tekil matris ya da gecersiz girdi).")
            continue
        print(f"    tum veri parametreleri  : {param_metni(tum_p)}")
        print(f"    fit kumesi parametreleri: {param_metni(fit_p)}")
        print(metrik_satiri('TUM VERI', so['tum']))
        print("      menzil kutulari (tum veri):")
        for satir in kutu_satirlari(so['tum']):
            print(satir)
        print(metrik_satiri('IC ORNEKLEM', so['ic']))
        print(metrik_satiri('TUTULAN TEST', so['test']))
        if so['test'].get('n', 0):
            print("      menzil kutulari (tutulan test):")
            for satir in kutu_satirlari(so['test']):
                print(satir)

    # --- en iyi modeli TUTULAN TEST RMSE'sine gore sec ---
    aday = [so for so in sonuclar if so['tum_param'] is not None]
    if not aday:
        print("\nSONUC: hicbir model uydurulamadi.")
        sys.exit(4)

    olculebilir = []
    if test_gecerli:
        olculebilir = [so for so in aday if so['test'].get('rmse_m') is not None]
        secim_olcutu = 'tutulan test RMSE'
    if not olculebilir:
        olculebilir = [so for so in aday if so['tum'].get('rmse_m') is not None]
        secim_olcutu = 'tum veri RMSE (tutulan test guvenilir degil)'
        test_gecerli = False
    if not olculebilir:
        print("\nSONUC: modeller karsilastirilamadi.")
        sys.exit(4)
    en_iyi = min(olculebilir,
                 key=lambda so: (so['test'] if test_gecerli else so['tum'])['rmse_m'])

    print("\n" + "=" * 78)
    print(f"SECILEN MODEL: {en_iyi['ad']} — {en_iyi['formul']}")
    print(f"Secim olcutu : {secim_olcutu}")
    print(f"Parametreler (TUM veriye yeniden uydurulmus): {param_metni(en_iyi['tum_param'])}")
    print(f"Gecerli menzil araligi: {np.min(R_all):.1f} .. {np.max(R_all):.1f} m "
          "(disina ekstrapolasyon guvenilmez)")
    print("=" * 78)

    # --- genelleme hukmu: acikca soyle ---
    ic_rmse = en_iyi['ic'].get('rmse_m')
    test_rmse = en_iyi['test'].get('rmse_m')
    test_r2 = en_iyi['test'].get('r2')
    if not test_gecerli:
        genelleme = ("Tutulan test kumesi yetersiz — modelin GENELLEYIP genellemedigi "
                     "OLCULEMEDI. Bu kalibrasyona uydurma hatasi kadar guvenilebilir, "
                     "daha fazlasina degil.")
    elif ic_rmse is None or test_rmse is None:
        genelleme = "Genelleme olculemedi (metrik hesaplanamadi)."
    elif test_r2 is not None and test_r2 < 0:
        genelleme = (f"MODEL GENELLEMIYOR: tutulan test R2 = {test_r2:.3f} < 0, yani tahmin "
                     "sabit ortalama tahmininden daha kotu. Bu kalibrasyon calisma aninda "
                     "KULLANILMAMALI.")
    elif ic_rmse > 1e-9 and test_rmse > 2.0 * ic_rmse:
        genelleme = (f"UYARI: tutulan test RMSE ({test_rmse:.1f} m) ic orneklem RMSE'sinin "
                     f"({ic_rmse:.1f} m) 2 katindan buyuk. Model kayit boyunca kayiyor; "
                     "tek bir sabit tum ucusa yetmiyor olabilir.")
    else:
        genelleme = (f"Model genelliyor: tutulan test RMSE {test_rmse:.1f} m, ic orneklem "
                     f"{ic_rmse:.1f} m; tutulan testte medyan mutlak hata "
                     f"%{bicim(en_iyi['test'].get('medyan_mutlak_yuzde'), 1)}.")
    print("\nGENELLEME: " + genelleme)

    # --- JSON + PNG ---
    json_yolu = os.path.abspath(args.json)
    os.makedirs(os.path.dirname(json_yolu) or '.', exist_ok=True)
    png_yolu = os.path.splitext(json_yolu)[0] + '.png'

    def temiz(so):
        return {'ad': so['ad'], 'formul': so['formul'], 'girdi': so['girdi'],
                'tum_veri_parametreleri': so['tum_param'],
                'fit_kumesi_parametreleri': so['fit_param'],
                'tum_veri': so['tum'], 'ic_orneklem': so['ic'],
                'tutulan_test': so['test']}

    cikti = {
        'olusturma': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'aciklama': ('Gorunur boyuttan menzil kestirimi. Gudum kodu hedefin konumunu '
                     'OKUMAZ; bu katsayilar cevrimdisi kalibre edilmistir.'),
        'model': en_iyi['ad'],
        'formul': en_iyi['formul'],
        'girdi_sutunu': en_iyi['girdi'],
        'parametreler': en_iyi['tum_param'],
        'secim_olcutu': secim_olcutu,
        'genelleme_hukmu': genelleme,
        'ornek_sayisi': int(n),
        'fit_ornek': n_fit,
        'test_ornek': n_test,
        'test_orani': args.test_orani,
        'azami_zaman_farki_s': args.azami_fark,
        'fit_istatistikleri': {
            'tum_veri': en_iyi['tum'],
            'ic_orneklem': en_iyi['ic'],
            'tutulan_test': en_iyi['test'],
        },
        'gecerli_menzil_araligi': {
            'min_m': float(np.min(R_all)),
            'max_m': float(np.max(R_all)),
            'medyan_m': float(np.median(R_all)),
            'not': 'Fit yalnizca bu aralikta gecerlidir; disina EKSTRAPOLASYON guvenilmez.',
        },
        'gecerli_boyut_araligi': {
            'sqrt_area_px_min': float(np.min(s_all)),
            'sqrt_area_px_max': float(np.max(s_all)),
            'bbox_h_min': float(np.min(bh_all)),
            'bbox_h_max': float(np.max(bh_all)),
        },
        'kaynak_csvler': ozetler,
        'tum_modeller': [temiz(so) for so in sonuclar],
    }
    with open(json_yolu, 'w') as fh:
        json.dump(cikti, fh, indent=2, ensure_ascii=False)

    grafik_ciz(png_yolu, s_all, bh_all, R_all, sonuclar, en_iyi['ad'])

    print(f"\n-> {json_yolu}")
    print(f"-> {png_yolu}")


if __name__ == '__main__':
    main()
