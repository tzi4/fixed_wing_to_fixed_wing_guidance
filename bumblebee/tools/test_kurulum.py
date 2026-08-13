#!/usr/bin/env python3
"""TEST KURULUM ARACI — avciyi istenen baslangic geometrisine oturtur.

ONEMLI AYRIM: bu TEST ALTYAPISIDIR, gudum degildir. Hedefin gercek konumunu
Gazebo'dan okur; bu bilgi gudum surecine (goat_cam_offset.py) ASLA girmez.
Kurulum bitince script cikar, gudum yalnizca kameradan gelen bbox ile calisir.

DERS (2026-07-29): ilk surum komutlari command_sender alt-sureci ile 8 saniyede
bir gonderiyordu. GUIDED'da hedefsiz kalan ucak loiter'a girdigi icin menzil
213 m -> 12.6 km acildi. Artik MAVLink'e DOGRUDAN baglanip komutu 4 Hz
yeniliyoruz; ayrica "bekle" modunda ucak hic GUIDED'a alinmaz.

Modlar:
  bekle        : ucak AUTO'da kalir, sadece istenen menzile gelmesi beklenir.
                 (T1 "hedef tam karside, es irtifa" icin dogru olan budur:
                  hicbir komut verilmez, geometri gorevden dogal olarak olusur)
  irtifa_ofset : avci GUIDED'a alinir, hedefe dogru heading + istenen irtifa
                 4 Hz gonderilir; |dz - istenen| toleransa girince cikar.

Kullanim:
  python3 tools/test_kurulum.py --mod bekle --menzil-min 150 --menzil-max 260
  python3 tools/test_kurulum.py --mod irtifa_ofset --dz -25   # hedef 25 m ASAGIDA
"""
import argparse
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from pymavlink import mavutil

HUNTER_PORT = 14551


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class State:
    """Gazebo yer gercegi — YALNIZCA kurulum icin."""

    def __init__(self):
        self.d = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._cb, queue_size=1)

    def _cb(self, msg):
        try:
            hi, ti = msg.name.index('hunter'), msg.name.index('target')
        except ValueError:
            return
        hp, tp = msg.pose[hi].position, msg.pose[ti].position
        self.d = dict(hx=hp.x, hy=hp.y, hz=hp.z, tx=tp.x, ty=tp.y, tz=tp.z,
                      hyaw=math.degrees(quat_yaw(msg.pose[hi].orientation)),
                      hgs=math.hypot(msg.twist[hi].linear.x, msg.twist[hi].linear.y),
                      tgs=math.hypot(msg.twist[ti].linear.x, msg.twist[ti].linear.y))

    def wait(self, t=15.0):
        t0 = time.time()
        while self.d is None and time.time() - t0 < t:
            time.sleep(0.1)
        if self.d is None:
            sys.exit('HATA: /gazebo/model_states okunamadi (sim ayakta mi?).')
        return self.d


def geom(d):
    dx, dy, dz = d['tx'] - d['hx'], d['ty'] - d['hy'], d['tz'] - d['hz']
    rxy = math.hypot(dx, dy)
    brg = math.degrees(math.atan2(dy, dx))
    return dict(rng=math.sqrt(rxy ** 2 + dz ** 2), rxy=rxy, dz=dz, bearing=brg,
                dyaw=(brg - d['hyaw'] + 180) % 360 - 180,
                elev=math.degrees(math.atan2(dz, rxy)) if rxy > 1e-6 else 0.0)


class Link:
    def __init__(self, port, sysid):
        self.m = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}')
        print(f'[link] 14{port % 1000:03d} heartbeat bekleniyor...')
        self.m.wait_heartbeat(timeout=30)
        self.sysid = sysid

    def set_mode(self, name='GUIDED'):
        self.m.set_mode(self.m.mode_mapping()[name])
        t0 = time.time()
        while time.time() - t0 < 10:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg and self.m.flightmode == name:
                print(f'[link] mod {name} onaylandi')
                return True
        print(f'[link] UYARI: mod {name} dogrulanamadi')
        return False

    def ack_topla(self, sure=0.0):
        """COMMAND_ACK'leri oku. Komut REDDEDILIYORSA tahmin yurutmek yerine
        ArduPilot'un kendi cevabini gormek icin."""
        sonuc = {}
        t0 = time.time()
        while True:
            m = self.m.recv_match(type='COMMAND_ACK', blocking=False)
            if m is None:
                if time.time() - t0 >= sure:
                    break
                time.sleep(0.02); continue
            sonuc.setdefault(m.command, []).append(m.result)
        return sonuc

    def heading(self, deg, rate=15.0):
        # 43002: param1=1 (raw heading), param2=heading, param3=rate.
        # rate=0 verilirse ArduPlane heading'i HIC degistirmez.
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            0, 1, deg % 360, rate, 0, 0, 0, 0)

    def speed(self, ms):
        # 43000: param1=0 (airspeed), param2=hiz, param3=ivme
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            0, 0, ms, 2.0, 0, 0, 0, 0)

    def altitude(self, m_):
        # 43001: param3=0 (rampa yok), param7=irtifa (HOME-goreli)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            0, 0, 0, 0, 0, 0, 0, max(20.0, m_))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mod', required=True, choices=('bekle', 'irtifa_ofset'))
    ap.add_argument('--menzil-min', type=float, default=150.0)
    ap.add_argument('--menzil-max', type=float, default=280.0)
    ap.add_argument('--dz', type=float, default=0.0,
                    help='hedef avcinin kac m USTUNDE olsun (negatif = asagida)')
    ap.add_argument('--tol-dz', type=float, default=4.0)
    ap.add_argument('--speed', type=float, default=None,
                    help='avci IAS komutu (menzil kapatmak icin; zarf <=22)')
    ap.add_argument('--menzil-hedef', type=float, default=None,
                    help='irtifa_ofset modunda ayrica bu menzilin altina inilsin')
    ap.add_argument('--timeout', type=float, default=420.0)
    ap.add_argument('--tani', action='store_true',
                    help='TANI MODU: sadece heading komutu gonderip ACK sonucunu basar')
    args = ap.parse_args()

    rospy.init_node('test_kurulum', anonymous=True, disable_signals=True)
    st = State(); st.wait()

    if args.tani:
        ADLAR = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                 3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS'}
        link = Link(HUNTER_PORT, 1)
        print(f'[tani] baglanti target_system={link.m.target_system} '
              f'target_component={link.m.target_component}')
        link.set_mode('GUIDED')
        d = st.d; g = geom(d)
        hedef_hdg = g['bearing'] % 360
        print(f"[tani] hunter_yaw={d['hyaw']:+.1f}  hedefe bearing={hedef_hdg:.1f}  "
              f"dyaw={g['dyaw']:+.1f}")
        for etiket, fn in (('heading', lambda: link.heading(hedef_hdg)),
                           ('altitude', lambda: link.altitude(d['tz'])),
                           ('speed', lambda: link.speed(18.0))):
            link.ack_topla(0.2)          # kuyrugu bosalt
            fn()
            acks = link.ack_topla(1.5)
            print(f"  {etiket:9s} -> " + (", ".join(
                f"cmd{c}: {[ADLAR.get(r, r) for r in rs]}" for c, rs in acks.items())
                or "ACK YOK"))
        print('\n[tani] 20 s boyunca heading komutu 4 Hz gonderilecek, yaw izlenecek')
        t0 = time.time(); yaw0 = st.d['hyaw']
        while time.time() - t0 < 20:
            link.heading(hedef_hdg); time.sleep(0.25)
        print(f"  yaw {yaw0:+.1f} -> {st.d['hyaw']:+.1f}  (istenen {hedef_hdg:.1f})")
        print(f"  degisim {((st.d['hyaw']-yaw0+180)%360)-180:+.1f} deg")
        return 0
    link = None
    if args.mod == 'irtifa_ofset':
        link = Link(HUNTER_PORT, 1)
        link.set_mode('GUIDED')

    t0 = time.time(); son_rapor = 0.0
    while time.time() - t0 < args.timeout:
        d = st.d
        if d is None:
            time.sleep(0.1); continue
        g = geom(d)

        if args.mod == 'irtifa_ofset':
            link.heading(g['bearing'])          # hedefe dogru
            link.altitude(d['tz'] - args.dz)    # hedef bizden args.dz kadar yukarida olsun
            if args.speed is not None:
                link.speed(args.speed)
            hazir = abs(g['dz'] - args.dz) < args.tol_dz and abs(g['dyaw']) < 8.0
            if args.menzil_hedef is not None:
                hazir = hazir and g['rng'] <= args.menzil_hedef
        else:
            hazir = args.menzil_min <= g['rng'] <= args.menzil_max and abs(g['dyaw']) < 8.0

        if time.time() - son_rapor > 5.0:
            son_rapor = time.time()
            print(f"  R={g['rng']:7.0f} m  dz={g['dz']:+6.1f} m  dyaw={g['dyaw']:+6.1f} deg  "
                  f"elev={g['elev']:+5.2f} deg  hgs={d['hgs']:.1f} tgs={d['tgs']:.1f}")
        if hazir:
            print(f"\n[HAZIR] menzil={g['rng']:.0f} m  dz={g['dz']:+.1f} m  "
                  f"dyaw={g['dyaw']:+.1f} deg  LOS elev={g['elev']:+.2f} deg")
            print("GUDUMU HEMEN BASLAT — GUIDED'da komutsuz kalan ucak loiter'a girer.")
            return 0
        time.sleep(0.25)

    print('\n[ZAMAN ASIMI] istenen geometriye ulasilamadi.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
