#!/usr/bin/env python3
"""Change the target's mission altitude while keeping SysID 2 in AUTO.

MAV_CMD_DO_CHANGE_ALTITUDE changes the altitude while the target
continues along its mission. Switching it to GUIDED without continuous
commands would instead initiate loiter.

This is ramp-test infrastructure. It passes no data to guidance.
"""
import argparse, time
from pymavlink import mavutil

NAMES = {0: 'ACCEPTED', 1: 'TEMP_REJ', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', type=float, required=True, help='new mission altitude (HOME-relative, m)')
    ap.add_argument('--port', type=int, default=14561)
    ap.add_argument('--duration-s', type=float, default=0.0, help='How many seconds to send again?')
    a = ap.parse_args()
    m = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.port}')
    m.wait_heartbeat(timeout=30)
    print(f'[target] sys={m.target_system} setup_mode={m.flightmode} -> altitude {a.alt:.0f} m')
    t0 = time.time()
    while True:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
                                0, a.alt, 3, 0, 0, 0, 0, 0)  # param1=altitude, param2=frame(3=REL)
        ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE:
            print(f'  ACK: {NAMES.get(ack.result, ack.result)}')
        if time.time() - t0 >= a.duration_s:
            break
        time.sleep(1.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
