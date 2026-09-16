#!/usr/bin/env python3
"""Set the hunter's initial geometry for a test.

This is test infrastructure. It reads target ground truth from Gazebo,
and never sends that information to goat_cam_offset.py. The script exits
when setup is complete, leaving guidance to use camera bounding boxes.

Commands are sent directly through MAVLink at 4 Hz. An earlier version
sent commands through a command_sender subprocess every 8 s, allowing
the aircraft to loiter between updates and increasing range from 213 m
to 12.6 km. The wait mode leaves the aircraft in AUTO throughout.

Modes:
  wait: stay in AUTO and wait for the desired range. For T1, with the
         target directly ahead at equal altitude, mission geometry
         establishes the conditions without issuing commands.
  altitude_offset: enter GUIDED and send heading toward the target and the
                requested altitude at 4 Hz. Exit when vertical separation
                reaches the requested tolerance.

Usage:
    python3 tools/test_setup.py --setup-mode wait --range-min 150 --range-max 260
    python3 tools/test_setup.py --setup-mode altitude_offset --dz -25
The second example places the target 25 m below the hunter.
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
    """Gazebo ground truth — for installation ONLY."""

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
            sys.exit('ERROR: /gazebo/model_states could not be read (sim standing?).')
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
        print(f'[link] 14{port % 1000:03d} heartbeat expected...')
        self.m.wait_heartbeat(timeout=30)
        self.sysid = sysid

    def set_mode(self, name='GUIDED'):
        self.m.set_mode(self.m.mode_mapping()[name])
        t0 = time.time()
        while time.time() - t0 < 10:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg and self.m.flightmode == name:
                print(f'[link] setup_mode {name} confirmed')
                return True
        print(f'[link] WARNING: mode {name} could not be verified')
        return False

    def collect_acks(self, duration_s=0.0):
        """Read COMMAND_ACKs. To see ArduPilot's own response instead of guessing if the command is REJECTED."""
        result_value = {}
        t0 = time.time()
        while True:
            m = self.m.recv_match(type='COMMAND_ACK', blocking=False)
            if m is None:
                if time.time() - t0 >= duration_s:
                    break
                time.sleep(0.02); continue
            result_value.setdefault(m.command, []).append(m.result)
        return result_value

    def heading(self, deg, rate=15.0):
        # 43002: param1=1 (raw heading), param2=heading, param3=rate.  If rate=0 is given, ArduPlane does not change the heading AT ALL.
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            0, 1, deg % 360, rate, 0, 0, 0, 0)

    def speed(self, ms):
        # 43000: param1=0 (airspeed), param2=speed, param3=acceleration
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            0, 0, ms, 2.0, 0, 0, 0, 0)

    def altitude(self, m_):
        # 43001: param3=0 (no ramp), param7=altitude (HOME-relative)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            0, 0, 0, 0, 0, 0, 0, max(20.0, m_))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--setup-mode', required=True, choices=('wait', 'altitude_offset'))
    ap.add_argument('--range-min', type=float, default=150.0)
    ap.add_argument('--range-max', type=float, default=280.0)
    ap.add_argument('--dz', type=float, default=0.0,
                    help='How many meters above the target hunter (negative = below)')
    ap.add_argument('--tol-dz', type=float, default=4.0)
    ap.add_argument('--speed', type=float, default=None,
                    help='hunter IAS command (to close range; envelope <=22)')
    ap.add_argument('--target-range', type=float, default=None,
                    help='In altitude_offset mode, also go below this range')
    ap.add_argument('--timeout', type=float, default=420.0)
    ap.add_argument('--diagnostic', action='store_true',
                    help='DIAGNOSTIC MODE: just send heading command and get result ACK')
    args = ap.parse_args()

    rospy.init_node('test_setup', anonymous=True, disable_signals=True)
    st = State(); st.wait()

    if args.diagnostic:
        NAMES = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                 3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS'}
        link = Link(HUNTER_PORT, 1)
        print(f'[diagnostic] connection target_system={link.m.target_system} '
              f'target_component={link.m.target_component}')
        link.set_mode('GUIDED')
        d = st.d; g = geom(d)
        target_hdg = g['bearing'] % 360
        print(f"[diagnostic] hunter_yaw={d['hyaw']:+.1f} target bearing={target_hdg:.1f}  "
              f"dyaw={g['dyaw']:+.1f}")
        for label, fn in (('heading', lambda: link.heading(target_hdg)),
                           ('altitude', lambda: link.altitude(d['tz'])),
                           ('speed', lambda: link.speed(18.0))):
            link.collect_acks(0.2)          # drain the queue
            fn()
            acks = link.collect_acks(1.5)
            print(f"  {label:9s} -> " + (", ".join(
                f"cmd{c}: {[NAMES.get(r, r) for r in rs]}" for c, rs in acks.items())
                or "ACK NO"))
        print('\n[diagnosis] Heading command 4 Hz will be sent along 20 s, yaw will be followed')
        t0 = time.time(); yaw0 = st.d['hyaw']
        while time.time() - t0 < 20:
            link.heading(target_hdg); time.sleep(0.25)
        print(f"  yaw {yaw0:+.1f} -> {st.d['hyaw']:+.1f}  (requested {target_hdg:.1f})")
        print(f"  change value {((st.d['hyaw']-yaw0+180)%360)-180:+.1f}")
        return 0
    link = None
    if args.setup_mode == 'altitude_offset':
        link = Link(HUNTER_PORT, 1)
        link.set_mode('GUIDED')

    t0 = time.time(); last_report = 0.0
    while time.time() - t0 < args.timeout:
        d = st.d
        if d is None:
            time.sleep(0.1); continue
        g = geom(d)

        if args.setup_mode == 'altitude_offset':
            link.heading(g['bearing'])          # towards the target
            link.altitude(d['tz'] - args.dz)    # target should be args.dz above us
            if args.speed is not None:
                link.speed(args.speed)
            ready = abs(g['dz'] - args.dz) < args.tol_dz and abs(g['dyaw']) < 8.0
            if args.target_range is not None:
                ready = ready and g['rng'] <= args.target_range
        else:
            ready = args.range_min <= g['rng'] <= args.range_max and abs(g['dyaw']) < 8.0

        if time.time() - last_report > 5.0:
            last_report = time.time()
            print(f"  R={g['rng']:7.0f} m  dz={g['dz']:+6.1f} m  dyaw={g['dyaw']:+6.1f} deg  "
                  f"elev={g['elev']:+5.2f} deg  hgs={d['hgs']:.1f} tgs={d['tgs']:.1f}")
        if ready:
            print(f"\n[READY] range={g['rng']:.0f} m dz={g['dz']:+.1f} m  "
                  f"dyaw={g['dyaw']:+.1f} deg  LOS elev={g['elev']:+.2f} deg")
            print("START GUIDANCE IMMEDIATELY — Left without command at GUIDED, the aircraft enters the loiter.")
            return 0
        time.sleep(0.25)

    print('\n[TIMEOUT] requested geometry could not be reached.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
