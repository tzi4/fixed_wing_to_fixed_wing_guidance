#!/usr/bin/env python3
"""Arm the target first, then put the target and Bumblebee in AUTO.

When run without arguments from a terminal, ask for:
  * Target (SysID 2) ground speed [m/s, Enter=20].
  * Bumblebee (SysID 1) ground speed [m/s, Enter=20].
  * Delay between takeoffs [s, Enter=2].
An empty response selects the displayed default. Invalid values and values
outside the permitted envelope prompt another question.

For automation, an argument supplied on the command line skips that question.
With --yes or nonterminal stdin, no questions are asked. The delay defaults
to 2 s. If no speed is supplied, no speed command is sent.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil


AIRCRAFT = ((14561, 2, 'target'), (14551, 1, 'bumblebee'))

# Acceptable GROUND speed range per aircraft (ten checks in interactive question; equalise_speed also trims with AIRSPEED_MIN/MAX).
SPEED_ENVELOPE = {'target': (9.0, 22.0), 'bumblebee': (15.0, 24.0)}
DEFAULT_SPEED = 20.0
DEFAULT_DELAY = 2.0
DELAY_RANGE = (0.0, 600.0)

MAV_CMD_DO_CHANGE_SPEED = 178
ACK_NAME = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
            3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}


def connect(port, expected_sysid, timeout):
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=254, source_component=191)
    heartbeat = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        candidate = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if candidate is not None and candidate.get_srcSystem() == expected_sysid:
            heartbeat = candidate
            break
    received_sysid = 0 if heartbeat is None else heartbeat.get_srcSystem()
    if heartbeat is None or received_sysid != expected_sysid:
        master.close()
        raise RuntimeError(f"port {port}: SysID {expected_sysid} expected, {received_sysid} received")
    master.target_system = received_sysid
    master.target_component = heartbeat.get_srcComponent()
    return master


def armed(master):
    return bool(master.motors_armed())


def arm(master, timeout, force_after):
    started = time.monotonic()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if time.monotonic() - started >= force_after:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 2989, 0, 0, 0, 0, 0,
            )
        else:
            master.arducopter_arm()
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and armed(master):
            return
    raise TimeoutError(f"SysID {master.target_system} was not armed")


def set_auto(master, timeout):
    mapping = master.mode_mapping() or {}
    mode_id = mapping.get('AUTO')
    if mode_id is None:
        raise RuntimeError('AUTO mode not found')
    master.set_mode(mode_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and getattr(msg, 'custom_mode', None) == mode_id:
            return
    raise TimeoutError(f"SysID {master.target_system} AUTO did not happen")


def read_param(master, name, timeout=6.0):
    """Read one parameter, returning None when unavailable."""
    master.mav.param_request_read_send(master.target_system,
                                       master.target_component,
                                       name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            return float(msg.param_value)
    return None


def sample_speeds(master, seconds):
    """(average IAS, average GPS ground speed, average altitude) for example."""
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                   mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 200000, 0, 0, 0, 0, 0)
    ias, gps, alt = [], [], []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = master.recv_match(type=['VFR_HUD', 'GLOBAL_POSITION_INT'],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'VFR_HUD':
            ias.append(msg.airspeed)
        else:
            gps.append(math.hypot(msg.vx, msg.vy) / 100.0)
            alt.append(msg.relative_alt / 1000.0)
    if not ias or not gps:
        raise RuntimeError('telemetry sample unavailable (VFR_HUD/GLOBAL_POSITION_INT)')
    return (sum(ias) / len(ias), sum(gps) / len(gps),
            sum(alt) / len(alt) if alt else float('nan'))


def wait_cruise(masters, target_alt, timeout):
    """Wait until both planes reach the target altitude (takeoff + elevation completion)."""
    deadline = time.monotonic() + timeout
    reached = {name: False for _, name in masters}
    while time.monotonic() < deadline:
        for master, name in masters:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            while msg is not None:
                if msg.relative_alt / 1000.0 >= target_alt:
                    reached[name] = True
                msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if all(reached.values()):
            return True
        time.sleep(0.2)
    missing = [name for name, ok in reached.items() if not ok]
    print(f"WARNING: {', '.join(missing)} {target_alt} did not reach m; continue anyway",
          file=sys.stderr)
    return False


def prompt_float(label, unit, default, low, high, reader=input, out=sys.stderr):
    """Interactive number ask.

    Empty Enter -> default. Input that is not a number or a value other than [low, high] will cause a warning and repeat the question. If the input stream runs out (EOF) the default is used.
    """
    question = f"{label} [{unit}, Enter={default:g}]: "
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print(f"  input finished, default {default:g} used", file=out)
            return default
        raw = raw.strip().replace(',', '.')
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("  invalid input (number expected), try again", file=out)
            continue
        if not low <= value <= high:
            print(f"  {value:g} is outside the accepted range [{low:g}, {high:g}], "
                  f"try again", file=out)
            continue
        return value


def resolve_settings(args, interactive, reader=input, out=sys.stderr):
    """Resolve final speed and delay values from arguments and optional prompts.

Skip questions for values supplied on the command line. When interactive
is False, use DEFAULT_DELAY and leave unspecified speeds as None,
meaning that no speed command is sent to those aircraft.
    """
    speeds = {}
    for _, sysid, name in AIRCRAFT:
        value = args.speed_by_name.get(name)
        if value is None and interactive:
            low, high = SPEED_ENVELOPE[name]
            value = prompt_float(f"{name} (SysID {sysid}) ground speed", 'm/s',
                                 DEFAULT_SPEED, low, high, reader=reader, out=out)
        speeds[name] = value
    delay = args.delay
    if delay is None:
        if interactive:
            delay = prompt_float('duration between departures', 's', DEFAULT_DELAY,
                                 DELAY_RANGE[0], DELAY_RANGE[1],
                                 reader=reader, out=out)
        else:
            delay = DEFAULT_DELAY
    return speeds, delay


def equalise_speed(masters, desired_gps, settle, sample):
    """Bring each aircraft's ground speed to its desired_gps target.

Without an airspeed sensor (ARSPD_TYPE 0), TECS uses synthetic airspeed:
IAS = |V_gps - W_EKF3|. Even with zero actual wind, each aircraft's EKF3
wind estimate can drift differently. Measured offsets were +0.29 m/s for
the hunter and -2.19 m/s for the target. Equal airspeed commands therefore
do not produce equal ground speeds.

Measure IAS - GPS for each aircraft, then use DO_CHANGE_SPEED to command
requested ground speed + measured offset. Command 178 is accepted in AUTO.
Command 43000 is restricted to GUIDED.

The desired_gps dictionary maps aircraft names to ground speeds in m/s.
A value of None leaves that aircraft's speed unchanged.
    """
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                   mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        for master, _ in masters:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 200000, 0, 0, 0, 0, 0)
    print(f"[speed] both aircraft >=45 m expected, then {settle:.0f} s seating")
    wait_cruise(masters, 45.0, timeout=max(60.0, settle))
    time.sleep(settle)
    for master, name in masters:
        wish = desired_gps.get(name)
        if wish is None:
            print(f"[{name}] no speed request, no command sent")
            continue
        ias, gps, alt = sample_speeds(master, sample)
        offset = ias - gps
        wanted = wish + offset
        low = read_param(master, 'AIRSPEED_MIN')
        high = read_param(master, 'AIRSPEED_MAX')
        print(f"[{name}] measurement: IAS={ias:.2f} GPS={gps:.2f} lower={alt:.0f} "
              f"difference={offset:+.2f} -> airspeed target {wanted:.2f}")
        if low is not None and high is not None:
            clamped = min(max(wanted, low), high)
            if abs(clamped - wanted) > 1e-6:
                # The outside the envelope command is NOT TRIMMED, it is REJECTED: ArduPlane Plane::do_change_speed() does not execute the request at all (COMMAND_ACK=FAILED) and the plane fails at its previous speed.
                print(f"[{name}] WARNING: {wanted:.2f} outside the flight envelope "
                      f"[{low:.1f}, {high:.1f}]; Sending {clamped:.2f}",
                      file=sys.stderr)
            wanted = clamped
        master.mav.command_long_send(
            master.target_system, master.target_component,
            MAV_CMD_DO_CHANGE_SPEED, 0,
            0.0, float(wanted), -1.0, 0, 0, 0, 0)
        result = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if ack is not None and ack.command == MAV_CMD_DO_CHANGE_SPEED:
                result = ack.result
                break
        label = ACK_NAME.get(result, 'ACK NO')
        print(f"[{name}] DO_CHANGE_SPEED {wanted:.2f} m/s -> {label}")
        if result != 0:
            print(f"[{name}] WARNING: speed command not accepted ({label})",
                  file=sys.stderr)


def nudge(master, seconds):
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 2200, 65535, 65535, 65535, 65535, 65535)
    time.sleep(seconds)
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--delay', type=float, default=None,
                        help=f'inter-departure duration (s); interactive if not given '
                             f'prompted, default {DEFAULT_DELAY:g}')
    parser.add_argument('--yes', action='store_true',
                        help='run automatically without prompts or confirmation')
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--force-arm-after', type=float, default=30,
                        help='SITL force-arm time after pre-arm wait')
    parser.add_argument('--nudge', type=float, default=1.0)
    parser.add_argument('--speed', type=float, default=None,
                        help='same GROUND speed for both aircraft (m/s) '
                             '(backwards compatibility shortcut)')
    parser.add_argument('--target-speed', type=float, default=None,
                        help='Desired GROUND speed (m/s) for target (SysID 2)')
    parser.add_argument('--bumblebee-speed', type=float, default=None,
                        help='Desired GROUND speed for bumblebee (SysID 1) (m/s)')
    parser.add_argument('--speed-settle', type=float, default=25.0,
                        help='speed: settling time (s) after reaching altitude')
    parser.add_argument('--speed-sample', type=float, default=8.0,
                        help='speed: IAS/GPS averaging window (s)')
    args = parser.parse_args()
    if args.delay is not None and args.delay < 0:
        parser.error('--delay cannot be negative')
    if args.nudge < 0 or args.force_arm_after < 0:
        parser.error('times cannot be negative')
    for flag, value in (('--speed', args.speed),
                        ('--target-speed', args.target_speed),
                        ('--bumblebee-speed', args.bumblebee_speed)):
        if value is not None and value <= 0:
            parser.error(f'{flag} must be positive')
    if args.speed_settle < 0 or args.speed_sample <= 0:
        parser.error('--speed-settle >= 0 and --speed-sample > 0')

    # Desired speed per plane: custom flag > --speed shortcut > (prompt/None).
    args.speed_by_name = {
        'target': args.target_speed if args.target_speed is not None else args.speed,
        'bumblebee': args.bumblebee_speed if args.bumblebee_speed is not None else args.speed,
    }
    for name, value in args.speed_by_name.items():
        low, high = SPEED_ENVELOPE[name]
        if value is not None and not low <= value <= high:
            print(f"WARNING: For {name}, {value:g} m/s is out of envelope [{low:g}, {high:g}]; "
                  f"Will be trimmed with AIRSPEED_MIN/MAX", file=sys.stderr)

    interactive = sys.stdin.isatty() and not args.yes
    speeds, delay = resolve_settings(args, interactive)
    want_speed = any(value is not None for value in speeds.values())

    masters = []
    try:
        for port, sysid, name in AIRCRAFT:
            print(f"[{name}] Establishing connection to {port}")
            master = connect(port, sysid, args.timeout)
            arm(master, args.timeout, args.force_arm_after)
            masters.append((master, name))
            print(f"[{name}] ARMED")
        if interactive:
            plan = ', '.join(
                f"{name}={speeds[name]:g} m/s" if speeds[name] is not None
                else f"{name}=no speed command" for _, _, name in AIRCRAFT)
            try:
                input(f"The target will depart {delay:.2f} s then Bumblebee "
                      f"({plan}). ENTER: ")
            except EOFError:
                print()
        for index, (master, name) in enumerate(masters):
            set_auto(master, args.timeout)
            nudge(master, args.nudge)
            print(f"[{name}] AUTO and takeoff impulse sent")
            if index == 0:
                time.sleep(delay)
        if want_speed:
            equalise_speed(masters, speeds, args.speed_settle,
                           args.speed_sample)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        for master, _ in masters:
            master.close()


if __name__ == '__main__':
    main()
