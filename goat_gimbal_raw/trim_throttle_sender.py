#!/usr/bin/env python3
"""
TRIM_THROTTLE Sender ----------------------- Mode 1: It receives a value between 50-127 from the user and continuously sends this value. Mode 2: It starts from 100 and decreases by 1 per second until 70, then increases by 1 per second to 100, decreases again to 70... (loop)

NOTE: Only works in GUIDED mode. If not GUIDED, it waits.
"""

import time
import sys
import math
from pymavlink import mavutil

UAV_PORT = '14553'

def connect():
    """Connect MAVLink."""
    print(f"[*] Establishing connection to MAVLink: 127.0.0.1:{UAV_PORT}")
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{UAV_PORT}')
    print("[*] Heartbeat expected...")
    master.wait_heartbeat()
    print(f"[+] Connection established! (system={master.target_system}, component={master.target_component})")
    return master


def get_flight_mode(master):
    """Return current flight mode. It does recv_match to update the cache."""
    # Refresh cache with non-blocking recv_match
    master.recv_match(blocking=False)
    hb = master.messages.get('HEARTBEAT', None)
    if hb:
        return mavutil.mode_string_v10(hb)
    return None


def is_guided(master):
    """Are we in GUIDED mode?"""
    return get_flight_mode(master) == 'GUIDED'


def get_current_heading(master):
    """Return the current heading in degrees (0-360)."""
    att = master.messages.get('ATTITUDE', None)
    if att:
        hdg = math.degrees(att.yaw)
        if hdg < 0:
            hdg += 360.0
        return hdg
    return None


def get_current_altitude(master):
    """Return the current altitude in meters (AMSL)."""
    loc = master.messages.get('GLOBAL_POSITION_INT', None)
    if loc:
        return loc.alt / 1000.0
    return None


def send_trim_throttle(master, value):
    """Send parameter TRIM_THROTTLE."""
    value = max(0, min(127, value))
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'TRIM_THROTTLE',
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )


def send_heading(master, heading_deg):
    """Send heading with MAV_CMD_GUIDED_CHANGE_HEADING."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        43002,  # MAV_CMD_GUIDED_CHANGE_HEADING
        0,
        1,            # param1: raw magnetic heading
        heading_deg,  # param2: target heading
        40,           # param3: heading rate (deg/s)
        0, 0, 0, 0
    )


def send_altitude(master, alt_m):
    """Send altitude with MAV_CMD_GUIDED_CHANGE_ALTITUDE."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        43001,  # MAV_CMD_GUIDED_CHANGE_ALTITUDE
        0,
        0, 0, 0,
        0, 0, 0,
        alt_m   # param7: desired altitude (AMSL)
    )


def mode1(master):
    """Mode 1: Constant value sending."""
    while True:
        try:
            val = int(input("\n[Mode 1] Enter a value between 50-127: "))
            if 50 <= val <= 127:
                break
            print("  ⚠ The value must be between 50-127!")
        except ValueError:
            print("  ⚠ Enter a valid number!")

    print(f"\n[Mode 1] TRIM_THROTTLE = {val} constantly sending... (Exit with Ctrl+C)")
    print("[*] Transmission is made only in GUIDED mode. Fixed at Heading + Altitud.")
    print("-" * 50)

    hold_hdg = None
    hold_alt = None
    try:
        while True:
            mode = get_flight_mode(master)
            if mode == 'GUIDED':
                # Catch heading/altitude on first GUIDED entry
                if hold_hdg is None:
                    hold_hdg = get_current_heading(master)
                    hold_alt = get_current_altitude(master)
                    print(f"  ✈ Heading={hold_hdg:.1f}° Fixed Altitud={hold_alt:.1f}m")
                
                send_trim_throttle(master, val)
                if hold_hdg is not None:
                    send_heading(master, hold_hdg)
                if hold_alt is not None:
                    send_altitude(master, hold_alt)
                print(f"  ✓ [{mode}] THR={val}  HDG={hold_hdg:.1f}°  ALT={hold_alt:.1f}m")
                time.sleep(0.2)  # Shipping 5 Hz
            else:
                hold_hdg = None
                hold_alt = None
                print(f"  ⏳ Expecting GUIDED... (current: {mode or '?'})")
                time.sleep(1.0)
    except KeyboardInterrupt:
        print(f"\n[!] Mode 1 terminated. Final value: {val}")


def mode2(master):
    """Mode 2: 100 → 70 → 100 → 70 ... (1 step per second)"""
    current = 100
    direction = -1  # -1: decreasing, +1: increasing
    
    print(f"\n[Mode 2] Start: {current}")
    print("  100 → 70 (↓1/sn) → 100 (↑1/sn) → 70 (↓1/sn) ... loop")
    print("[*] Transmission is made only in GUIDED mode. Fixed at Heading + Altitud.")
    print("  Exit with Ctrl+C")
    print("-" * 50)

    hold_hdg = None
    hold_alt = None
    try:
        while True:
            mode = get_flight_mode(master)
            if mode == 'GUIDED':
                # Catch heading/altitude on first GUIDED entry
                if hold_hdg is None:
                    hold_hdg = get_current_heading(master)
                    hold_alt = get_current_altitude(master)
                    print(f"  ✈ Heading={hold_hdg:.1f}° Fixed Altitud={hold_alt:.1f}m")
                
                send_trim_throttle(master, current)
                if hold_hdg is not None:
                    send_heading(master, hold_hdg)
                if hold_alt is not None:
                    send_altitude(master, hold_alt)
                
                if direction == -1:
                    arrow = "↓"
                else:
                    arrow = "↑"
                print(f"  {arrow} [{mode}] THR={current}  HDG={hold_hdg:.1f}°  ALT={hold_alt:.1f}m")

                time.sleep(1.0)

                current += direction

                # Reached lower limit → change direction (up)
                if current <= 70:
                    current = 70
                    direction = 1
                # Reached upper limit → change direction (down)
                elif current >= 100:
                    current = 100
                    direction = -1
            else:
                hold_hdg = None
                hold_alt = None
                print(f"  ⏳ Expecting GUIDED... (current: {mode or '?'})")
                time.sleep(1.0)

    except KeyboardInterrupt:
        print(f"\n[!] Mode 2 terminated. Final value: {current}")


def main():
    master = connect()

    print("\n" + "=" * 50)
    print("  TRIM_THROTTLE Shipper")
    print("=" * 50)
    print("  Mode 1: Fixed value (between 50-127)")
    print("  Mode 2: Auto oscillation (100 ↔ 70)")
    print("  ⚠ Only works in GUIDED mode!")
    print("=" * 50)

    while True:
        try:
            mode = input("\nSelect mode (1 or 2): ").strip()
            if mode in ('1', '2'):
                break
            print("  ⚠ Please enter 1 or 2!")
        except (ValueError, EOFError):
            print("  ⚠ Enter a valid value!")

    if mode == '1':
        mode1(master)
    else:
        mode2(master)


if __name__ == '__main__':
    main()
