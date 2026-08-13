#!/usr/bin/env python3
"""YARISMA SUNUCUSU TAKLIDI — hedefin 3B konumunu 1 Hz yayinlar.

KAYNAK: MAVLink GLOBAL_POSITION_INT (Gazebo DEGIL).
Ilk surum Gazebo x/y'yi elle enlem/boylama ceviriyordu; olcum gosterdi ki o
cerceve ArduPilot'unkiyle UYUSMUYOR (doguda sabit ~188 m, kuzeyde buyuyen
fark) ve menzile ~6 m sistematik sapma giriyordu. Yarismada hem sunucu hem
ucak WGS84 veriyor, yani ayni cercevede — burada da iki tarafi ArduPilot'un
kendi cozumunden almak gercege daha yakin.

    rakip_telemetri : hedef (SysID 2, port 14561) — teva.py MENZIL icin okur
    avci_telemetri  : avci  (SysID 1, port 14551) — yalniz video overlay'i

Bicim: {"konumBilgileri": [{"iha_enlem":.., "iha_boylam":.., "iha_irtifa":..}]}

1 Hz BILEREK: daha sik yayinlamak teva.py'nin ileri sarma mantigini test
etmeden birakir.
"""
import argparse
import json
import time

import redis
from pymavlink import mavutil


def paket(msg, gurultu_m=0.0):
    lat, lon = msg.lat / 1e7, msg.lon / 1e7
    alt = msg.relative_alt / 1000.0
    if gurultu_m > 0:
        import random
        lat += random.gauss(0, gurultu_m) / 111320.0
        lon += random.gauss(0, gurultu_m) / 111320.0
        alt += random.gauss(0, gurultu_m)
    return json.dumps({"konumBilgileri": [{
        "iha_enlem": round(lat, 7), "iha_boylam": round(lon, 7),
        "iha_irtifa": round(alt, 2)}]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=1.0, help='yayin hizi (yarisma: 1)')
    ap.add_argument('--hedef-port', type=int, default=14561)
    ap.add_argument('--avci-port', type=int, default=14551)
    ap.add_argument('--gurultu-m', type=float, default=0.0,
                    help='konuma eklenecek gauss gurultusu (saglamlik testi)')
    a = ap.parse_args()

    r = redis.Redis(host='localhost', port=6379, db=0); r.ping()
    hedef = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.hedef_port}')
    print(f'[sunucu] hedef {a.hedef_port} heartbeat bekleniyor...'); hedef.wait_heartbeat(timeout=30)
    avci = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.avci_port}')
    print(f'[sunucu] avci {a.avci_port} heartbeat bekleniyor...'); avci.wait_heartbeat(timeout=30)
    print(f'Sunucu taklidi: {a.hz} Hz, gurultu {a.gurultu_m} m')

    period = 1.0 / a.hz
    son = 0.0; n = 0
    sh = sa = None
    try:
        while True:
            m = hedef.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if m: sh = m
            m = avci.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if m: sa = m
            now = time.time()
            if now - son >= period:
                son = now
                if sh is not None:
                    r.set('rakip_telemetri', paket(sh, a.gurultu_m))
                if sa is not None:
                    r.set('avci_telemetri', paket(sa, a.gurultu_m))
                n += 1
                if sh is not None and n % 20 == 1:
                    print(f"  [{n:4d}] hedef {sh.lat/1e7:.6f} {sh.lon/1e7:.6f} "
                          f"{sh.relative_alt/1000.0:.1f} m")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n{n} paket yayinlandi.")


if __name__ == '__main__':
    main()
