#!/usr/bin/env python3
"""DUMAN TESTI — simulasyona girmeden ONCE calistirilir.

Neden var: 2026-07-29'da sozdizimi gecerli ama CALISMAYAN kod commit'lendi
(bir "class ..." satiri silinmisti). ast.parse ve --help bunu yakalayamadi;
hata ancak sim ayaktayken, dakikalar harcandiktan sonra ortaya cikti.
Bu script o sinifi hatalari saniyeler icinde yakalar.

Kullanim:  python3 tools/duman_testi.py     (0 = temiz, 1 = hata)
"""
import importlib.util
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
hatalar = []


def kontrol(ad, kosul, detay=""):
    print(f"  {'OK  ' if kosul else 'HATA'} {ad}" + (f" — {detay}" if detay and not kosul else ""))
    if not kosul:
        hatalar.append(ad)


def modul_yukle(yol, ad):
    spec = importlib.util.spec_from_file_location(ad, yol)
    m = importlib.util.module_from_spec(spec)
    sys.modules[ad] = m
    spec.loader.exec_module(m)
    return m


def _sema(yol, ad):
    import ast as _a
    t = _a.parse(open(yol).read())
    h = r = pl = None
    for n in _a.walk(t):
        if isinstance(n, _a.Assign) and isinstance(n.value, _a.List):
            x = n.targets[0]
            if isinstance(x, _a.Name) and x.id == 'headers':
                h = len(n.value.elts)
            if isinstance(x, _a.Name) and x.id == 'row':
                r = len(n.value.elts)
        if isinstance(n, _a.AugAssign) and isinstance(n.target, _a.Name) and n.target.id == 'row':
            pl = len(n.value.elts)
    kontrol(f"{ad}: baslik={h} satir={r}+{pl}", h is not None and h == (r or 0) + (pl or 0),
            "uyusmazlik log sutunlarini kaydirir")


print("1) Gudum modulu gercekten yukleniyor mu (sozdizimi DEGIL, calisma anı)")
try:
    g = modul_yukle(os.path.join(ROOT, 'goat_cam_offset.py'), 'gco_smoke')
    kontrol("modul yuklendi", True)
    for cls, meths in (('FlightLogger', []), ('RedisListener', ['get_gorev']),
                       ('MavlinkManager', ['send_heading_target', 'send_altitude_target', 'run']),
                       ('AutopilotController', ['stabilize_pixel', '_log_state', 'run', 'clamp'])):
        var = hasattr(g, cls)
        kontrol(f"sinif {cls}", var)
        if var:
            for me in meths:
                kontrol(f"  {cls}.{me}", hasattr(getattr(g, cls), me))
    kontrol("hedef verisi koda sizmamis (get_rakip_irtifa yok)",
            not hasattr(getattr(g, 'RedisListener', object), 'get_rakip_irtifa'))
except Exception as e:
    kontrol("modul yuklendi", False, repr(e))
    g = None

print("\n1b) teva.py (yarisma surumu: 1 Hz hedef 3B konumu) yukleniyor mu")
try:
    tv = modul_yukle(os.path.join(ROOT, 'teva.py'), 'teva_smoke')
    kontrol("teva modulu yuklendi", True)
    for cls in ('HedefTelemetri', 'MavlinkManager', 'AutopilotController'):
        kontrol(f"teva.{cls}", hasattr(tv, cls))
    if hasattr(tv, 'HedefTelemetri'):
        for me in ('menzil', 'run'):
            kontrol(f"  HedefTelemetri.{me}", hasattr(tv.HedefTelemetri, me))
except Exception as e:
    kontrol("teva modulu yuklendi", False, repr(e))
    tv = None

def _alan_denetimi(yol, ad, sinif='AutopilotController'):
    """Sinifta OKUNAN her self.X, __init__'te ATANMIS mi?

    2026-07-30: menzil-farkinda aim eklerken self.aim_tam_menzil_m atanmadan
    okundu; sozdizimi gecerliydi, --help gecti, duman testi gecti ve hata
    ancak sim ayaktayken AttributeError olarak ciktı. Bu denetim onu yakalar.
    """
    import ast as _a
    t = _a.parse(open(yol).read())
    for n in _a.walk(t):
        if isinstance(n, _a.ClassDef) and n.name == sinif:
            atanan, okunan = set(), {}
            for x in _a.walk(n):
                if isinstance(x, _a.Attribute) and isinstance(x.value, _a.Name) and x.value.id == 'self':
                    if isinstance(x.ctx, _a.Store):
                        atanan.add(x.attr)
                    else:
                        okunan.setdefault(x.attr, x.lineno)
            metotlar = {f.name for f in n.body if isinstance(f, (_a.FunctionDef, _a.AsyncFunctionDef))}
            eksik = {k: v for k, v in okunan.items() if k not in atanan and k not in metotlar}
            kontrol(f"{ad}: {sinif} alanlari atanmis", not eksik,
                    "atanmadan okunan: " + ", ".join(f"{k} (satir {v})" for k, v in sorted(eksik.items())))
            return
    kontrol(f"{ad}: {sinif} bulundu", False)


print("\n1c) Sinif alanlari: okunan her self.X atanmis mi?")
for _d in ('goat_cam_offset.py', 'teva.py'):
    try:
        _alan_denetimi(os.path.join(ROOT, _d), _d)
    except Exception as e:
        kontrol(f"alan denetimi ({_d})", False, repr(e))

print("\n2) CSV sema tutarliligi (baslik sayisi == satir sayisi)")
for _dosya in ('goat_cam_offset.py', 'teva.py'):
    try:
        _sema(os.path.join(ROOT, _dosya), _dosya)
    except Exception as e:
        kontrol(f"sema kontrolu ({_dosya})", False, repr(e))

print("\n3) Sanal gimbal matematigi (regresyon: isaret ve merkezleme)")
if g is not None:
    try:
        K = np.array([[4543, 0, 1025], [0, 4539, 569], [0, 0, 1]], float)
        Kinv = np.linalg.inv(K)

        def ry(d):
            r = math.radians(d)
            return np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])

        def ham_piksel(eps, th, roll=0.0):
            """Ufka gore eps derece yukaridaki hedefin HAM (x, y) pikseli.
            roll != 0 iken hedef yatayda da kayar; x'i sabit varsaymak yanlistir."""
            e = math.radians(eps)
            dvec = np.array([math.cos(e), 0.0, -math.sin(e)])
            R = g.compute_R_b_e(math.radians(roll), math.radians(th), 0.0)
            h = K @ (g.R_c_b_T @ (R.T @ dvec))
            return h[0] / h[2], h[1] / h[2]

        def exey(eps, th, aim, roll=0.0):
            x, y = ham_piksel(eps, th, roll)
            R = g.compute_R_b_e(math.radians(roll), math.radians(th), 0.0)
            rb = g.R_c_b @ (Kinv @ np.array([x, y, 1.0]))
            hh = K @ (g.R_c_b_T @ (ry(aim) @ (R @ rb)))
            sx, sy = hh[0] / hh[2], hh[1] / hh[2]
            return (math.degrees(math.atan((sx - 1025) / 4543)),
                    math.degrees(math.atan((sy - 569) / 4539)))

        def ey(eps, th, aim, roll=0.0):
            return exey(eps, th, aim, roll)[1]

        # (a) de-rotasyon: es irtifali hedef, aim=0'da her roll/pitch'te merkezde
        kotu = [(rl, th) for rl in (0, 10, 20, 30) for th in (-2.77, 0.0, 4.19)
                if max(abs(v) for v in exey(0.0, th, 0.0, rl)) > 1e-3]
        kontrol("de-rotasyon roll/pitch'i tam cikariyor", not kotu, f"sapan: {kotu[:3]}")
        # (b) denge bagintisi: ey=0 <=> eps = -aim
        kontrol("denge eps = -aim", all(abs(ey(-a, -2.77, a)) < 1e-3 for a in (-6, -2.8, 0, 2.8, 6)))
        # (c) aim = -theta hedefi HAM kadraj merkezine oturtur
        for th in (-2.77, 4.19):
            _, yv = ham_piksel(th, th)   # denge eps = theta
            kontrol(f"aim=-theta (theta={th:+.2f}) -> ham piksel merkezde",
                    abs(yv - 569) < 2.0, f"y={yv:.1f}")
    except Exception as e:
        kontrol("gimbal matematigi", False, repr(e))

print("\n4) Yardimci araclar --help ile aciliyor mu")
for t_ in ('pid_grafik.py', 'gercek_konum_logger.py', 'test_kurulum.py'):
    p = os.path.join(ROOT, 'tools', t_)
    if not os.path.exists(p):
        kontrol(t_, False, "dosya yok"); continue
    rc = subprocess.run([sys.executable, p, '--help'], capture_output=True, timeout=60).returncode
    kontrol(t_, rc == 0, f"cikis kodu {rc}")

print("\n" + ("DUMAN TESTI TEMIZ — sim'e girilebilir" if not hatalar
              else f"DUMAN TESTI BASARISIZ ({len(hatalar)}): " + ", ".join(hatalar)))
sys.exit(1 if hatalar else 0)
