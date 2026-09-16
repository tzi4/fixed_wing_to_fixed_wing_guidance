#!/usr/bin/env python3
"""Emulate the competition server by publishing target position at 1 Hz.

Use MAVLink GLOBAL_POSITION_INT from ArduPilot. A previous conversion
from Gazebo x/y to latitude/longitude did not align with ArduPilot's
frame: measured differences were about 188 m eastward and increased
northward, causing about 6 m systematic range bias. In the competition,
both server and aircraft use WGS84. Reading both positions from ArduPilot
provides consistent coordinates in the simulation.

  rakip_telemetri: target, SysID 2 on port 14561, read by teva.py for range.
  hunter_telemetry: hunter, SysID 1 on port 14551, used only by the overlay.

Schema:
    {"konumBilgileri": [{"iha_enlem": .., "iha_boylam": .., "iha_irtifa": ..}]}
The deliberate 1 Hz rate exercises teva.py's dead-reckoning logic.
"""
import argparse
import json
import time

import redis
from pymavlink import mavutil


def packet(msg, noise_m=0.0):
    lat, lon = msg.lat / 1e7, msg.lon / 1e7
    alt = msg.relative_alt / 1000.0
    if noise_m > 0:
        import random
        lat += random.gauss(0, noise_m) / 111320.0
        lon += random.gauss(0, noise_m) / 111320.0
        alt += random.gauss(0, noise_m)
    return json.dumps({"konumBilgileri": [{
        "iha_enlem": round(lat, 7), "iha_boylam": round(lon, 7),
        "iha_irtifa": round(alt, 2)}]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=1.0, help='broadcast speed (competition: 1)')
    ap.add_argument('--target-port', type=int, default=14561)
    ap.add_argument('--hunter-port', type=int, default=14551)
    ap.add_argument('--noise-m', type=float, default=0.0,
                    help='gaussian noise to be added to the position (robustness test)')
    a = ap.parse_args()

    r = redis.Redis(host='localhost', port=6379, db=0); r.ping()
    target = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.target_port}')
    print(f'[server] target {a.target_port} waiting for heartbeat...'); target.wait_heartbeat(timeout=30)
    hunter = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.hunter_port}')
    print(f'[server] hunter {a.hunter_port} heartbeat expected...'); hunter.wait_heartbeat(timeout=30)
    print(f'Server simulation: {a.hz} Hz, noise {a.noise_m} m')

    period = 1.0 / a.hz
    last_value = 0.0; n = 0
    sh = sa = None
    try:
        while True:
            m = target.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if m: sh = m
            m = hunter.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if m: sa = m
            now = time.time()
            if now - last_value >= period:
                last_value = now
                if sh is not None:
                    r.set('rakip_telemetri', packet(sh, a.noise_m))
                if sa is not None:
                    r.set('hunter_telemetry', packet(sa, a.noise_m))
                n += 1
                if sh is not None and n % 20 == 1:
                    print(f"  [{n:4d}] target {sh.lat/1e7:.6f} {sh.lon/1e7:.6f} "
                          f"{sh.relative_alt/1000.0:.1f} m")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n{n} package has been released.")


if __name__ == '__main__':
    main()
