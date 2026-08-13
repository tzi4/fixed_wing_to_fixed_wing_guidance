#!/usr/bin/env python3
"""
GERCEK KONUM LOGGER — gudum kodundan BAGIMSIZ dogruluk cipasi.

NEDEN VAR: gudum CSV'si dunyayi KAMERADAN gorur. Kamera hatasi salinirken
"iyi gidiyor" demek mumkun. Bu script hedefi ve avciyi Gazebo'nun kendi
model_states'inden okur — EKF'ten, bbox'tan, gudum kodundan bagimsiz yer
gercegi (ground truth). Boylece su ayrim yapilabilir:

    ex hatasi salaniyor + hedef DUMDUZ gidiyor      -> sorun BIZDE (kontrolcu)
    ex hatasi salaniyor + hedef de manevra yapiyor  -> takip zaten zor

Gudum ile AYNI ANDA calisir; hicbir MAVLink portuna baglanmaz, dolayisiyla
14551/14553/14561 cakismasi YOKTUR.

Kullanim:
    python3 tools/gercek_konum_logger.py                  # ucus_loglari/ altina yazar
    python3 tools/gercek_konum_logger.py --hz 20
    python3 tools/gercek_konum_logger.py --out /tmp/x.csv
"""
import argparse
import csv
import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelStates


def quat_to_euler(q):
    """(roll, pitch, yaw) radyan — ZYX."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


COLUMNS = [
    'timestamp', 'elapsed_s',
    'hunter_x', 'hunter_y', 'hunter_z', 'hunter_roll_deg', 'hunter_pitch_deg', 'hunter_yaw_deg',
    'hunter_vx', 'hunter_vy', 'hunter_vz', 'hunter_gs_ms', 'hunter_yaw_rate_dps',
    'target_x', 'target_y', 'target_z', 'target_roll_deg', 'target_pitch_deg', 'target_yaw_deg',
    'target_vx', 'target_vy', 'target_vz', 'target_gs_ms', 'target_yaw_rate_dps',
    # --- turetilmis: gudumun "olmasi gereken" degerleri ---
    'range_m', 'range_xy_m', 'alt_diff_m',
    'los_bearing_deg',      # avcidan hedefe GERCEK pusula acisi
    'los_elev_deg',         # avcidan hedefe GERCEK yukselis acisi
    'bearing_error_deg',    # los_bearing - hunter_yaw  (ex'in yer gercegi karsiligi)
    'elev_error_deg',       # los_elev  - hunter_pitch  (ey'in yer gercegi karsiligi)
    'closure_rate_ms',      # menzil degisim hizi (negatif = yaklasiyor)
    'target_maneuvering',   # 1 = hedef donuyor (|yaw_rate| > 2 deg/s)
]


class Logger:
    def __init__(self, path, hunter_key, target_key, hz):
        self.path = path
        self.hunter_key = hunter_key
        self.target_key = target_key
        self.period = 1.0 / hz
        self.start = time.time()
        self.last_write = 0.0
        self.prev = {}
        self.rows = 0
        self.warned = False
        self.fh = open(path, 'w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(COLUMNS)
        self.fh.flush()
        print(f"Yer gercegi logu: {path}")

    @staticmethod
    def _pick(names, key):
        if key in names:
            return names.index(key)
        for i, n in enumerate(names):
            if key.lower() in n.lower():
                return i
        return None

    def cb(self, msg):
        now = time.time()
        if now - self.last_write < self.period:
            return
        hi = self._pick(msg.name, self.hunter_key)
        ti = self._pick(msg.name, self.target_key)
        if hi is None or ti is None:
            if not self.warned:
                print(f"UYARI: model bulunamadi. Gazebo'daki modeller: {list(msg.name)}")
                print("       --hunter / --target ile ad verin.")
                self.warned = True
            return
        dt = now - self.last_write if self.last_write else 0.0
        self.last_write = now

        out = [f"{now:.4f}", f"{now - self.start:.3f}"]
        vals = {}
        for tag, idx in (('hunter', hi), ('target', ti)):
            p = msg.pose[idx].position
            r, pit, yaw = quat_to_euler(msg.pose[idx].orientation)
            t = msg.twist[idx].linear
            gs = math.hypot(t.x, t.y)
            yaw_deg = math.degrees(yaw)
            rate = 0.0
            if dt > 1e-6 and tag in self.prev:
                rate = wrap180(yaw_deg - self.prev[tag]) / dt
            self.prev[tag] = yaw_deg
            vals[tag] = dict(x=p.x, y=p.y, z=p.z, yaw=yaw_deg, pitch=math.degrees(pit), gs=gs)
            out += [f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}",
                    f"{math.degrees(r):.3f}", f"{math.degrees(pit):.3f}", f"{yaw_deg:.3f}",
                    f"{t.x:.3f}", f"{t.y:.3f}", f"{t.z:.3f}", f"{gs:.3f}", f"{rate:.3f}"]

        h, g = vals['hunter'], vals['target']
        dx, dy, dz = g['x'] - h['x'], g['y'] - h['y'], g['z'] - h['z']
        rng_xy = math.hypot(dx, dy)
        rng = math.sqrt(rng_xy ** 2 + dz ** 2)
        # Gazebo ENU: yaw 0 = +X. Pusula yerine ayni referansta kaliyoruz ki
        # bearing_error dogrudan hunter_yaw ile karsilastirilabilsin.
        los_b = math.degrees(math.atan2(dy, dx))
        los_e = math.degrees(math.atan2(dz, rng_xy)) if rng_xy > 1e-6 else 0.0
        closure = 0.0
        if dt > 1e-6 and 'range' in self.prev:
            closure = (rng - self.prev['range']) / dt
        self.prev['range'] = rng
        tgt_rate = float(out[COLUMNS.index('target_yaw_rate_dps')])
        out += [f"{rng:.3f}", f"{rng_xy:.3f}", f"{dz:.3f}",
                f"{los_b:.3f}", f"{los_e:.3f}",
                f"{wrap180(los_b - h['yaw']):.3f}",
                f"{los_e - h['pitch']:.3f}",
                f"{closure:.3f}",
                1 if abs(tgt_rate) > 2.0 else 0]
        self.w.writerow(out)
        self.rows += 1
        if self.rows % 100 == 0:
            self.fh.flush()

    def close(self):
        try:
            self.fh.flush(); self.fh.close()
        except Exception:
            pass
        print(f"\n{self.rows} satir yazildi -> {self.path}")


def main():
    ap = argparse.ArgumentParser(description='Gazebo yer gercegi (ground truth) kaydedici')
    ap.add_argument('--out', default=None, help='CSV yolu (varsayilan ucus_loglari/gercek_konum_*.csv)')
    ap.add_argument('--hz', type=float, default=10.0, help='kayit hizi (varsayilan 10)')
    ap.add_argument('--hunter', default='hunter', help='avci model adi (varsayilan hunter)')
    ap.add_argument('--target', default='target', help='hedef model adi (varsayilan target)')
    args = ap.parse_args()

    out = args.out
    if out is None:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        d = os.path.join(root, 'ucus_loglari')
        os.makedirs(d, exist_ok=True)
        out = os.path.join(d, time.strftime('gercek_konum_%Y%m%d_%H%M%S.csv'))

    rospy.init_node('gercek_konum_logger', anonymous=True, disable_signals=True)
    lg = Logger(out, args.hunter, args.target, args.hz)
    rospy.Subscriber('/gazebo/model_states', ModelStates, lg.cb, queue_size=1)
    print("Kayit basladi (Ctrl-C ile bitir).")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lg.close()


if __name__ == '__main__':
    main()
