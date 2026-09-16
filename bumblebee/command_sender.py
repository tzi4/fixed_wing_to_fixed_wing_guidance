#!/usr/bin/env python3
"""Test vehicle sending heading + altitude + speed command in GUIDED mode.

It uses the exact sending pattern of the old codes (tzi2.py / final_competition/speeds.py):

  * heading -> MAV_CMD_GUIDED_CHANGE_HEADING (43002) param1=1 (magnetic nose)
  * altitude -> MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) param7=altitude
  * speed -> MAV_CMD_GUIDED_CHANGE_SPEED (43000) param1=0 (airspeed) if failed MAV_CMD_DO_CHANGE_SPEED (178) fallback

THERE ARE TWO WORKING WAYS:

Argument-driven mode repeatedly sends commands at 5 Hz using the
tzi2.py TestCommander pattern. Display COMMAND_ACK and STATUSTEXT
messages and optionally record telemetry in a CSV file.

     ./command_sender.py --connect udpin:127.0.0.1:14553 --sysid 1 \
         --heading 0 --alt 100 --speed 20 --hold 30 --csv reports/x.csv

2) INTERACTIVE (if run without arguments, or with --interactive) To manually position the target in front of the camera: connect to a vehicle and ask for heading / altitude / ground speed repeatedly.

     ./command_sender.py

   Interactive sequence: collect inputs, change mode, then send commands.
  * Select [1] Bumblebee on port 14553 or [2] target on port 14561.
    Enter selects vehicle 2.
  * If the aircraft is not in GUIDED, ask once whether to enter GUIDED
    after collecting inputs. Save the answer without changing mode yet.
  * Each round displays the current state and asks for heading in degrees,
    relative altitude in metres, and ground speed in m/s. An empty reply
    holds the current value. Invalid or out-of-envelope values are retried.
  * Collect a short sample window to estimate IAS - GPS. On the first
    round this still occurs in the original mode during stable flight.
  * Change to GUIDED. Once the mode is confirmed, send the three commands
    consecutively and print their acknowledgments.
  * Enter q at any prompt, or use Ctrl-D, to quit. On the first round the
    original mode is retained because GUIDED has not yet been entered.
    Subsequent rounds leave the aircraft in GUIDED and continue the
    requested trajectory.

   WHY THIS ORDER: When ArduPlane enters GUIDED without a target, it starts LOITER around the current position. If the mode had been changed first, the plane would circle while the user was answering the questions, and the "current" defaults would be thrown away.    Once the input is received first, (a) the defaults are read from the stable flight (alleviating the stale heading problem), (b) there is no human delay between switching to GUIDED and sending the command -- the aircraft turns directly into the desired direction.

   SPEED = GROUND SPEED. Since there is no sensor (ARSPD_TYPE 0), the airspeed is synthetic (IAS = |V_gps - W_EKF3|) and shifts per plane; The equalise_speed pattern in the formation.py is applied here as well: live (IAS - GPS) offset is measured in a short window before submission, calculated as command = desired ground speed + offset, and clipped to the live AIRSPEED_MIN/MAX envelope.

   NOTE: 43000/43001/43002 is ONLY accepted in GUIDED (FAILED in other mode), altitude 43001 is RELATIVE to HOME.
"""

import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

MAV_CMD_GUIDED_CHANGE_SPEED = 43000
MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
MAV_CMD_GUIDED_CHANGE_HEADING = 43002
MAV_CMD_DO_CHANGE_SPEED = 178

SPEED_TYPE = {'airspeed': 0, 'groundspeed': 1}

ACK_NAME = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
    4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

# --- interactive mode constants ------------------------------------------- (name, link, expected SysID, accepted GROUND speed range) Ground speed ranges formation.py Same as SPEED_ENVELOPE; If the live AIRSPEED_MIN/MAX can be read, it will also be cropped with it before sending.
VEHICLES = {
    '1': ('bumblebee', 'udpin:127.0.0.1:14553', 1, (15.0, 24.0)),
    '2': ('target', 'udpin:127.0.0.1:14561', 2, (9.0, 22.0)),
}
DEFAULT_VEHICLE = '2'          # usage scenario: positioning the target manually
ALT_RANGE = (0.0, 1000.0)      # m, relative to HOME
HEADING_RANGE = (0.0, 360.0)   # degrees
QUIT_WORDS = ('q', 'Q', 'exit', 'quit')

QUIT = object()                # prompt_value(): user wanted to quit


def ack_text(result):
    return ACK_NAME.get(result, f'RESULT_{result}')


class CommandSender:
    """Thin wrapper sending command heading, altitude and speed to single vehicle."""

    def __init__(self, connect, sysid, timeout=30.0):
        self.master = mavutil.mavlink_connection(
            connect, source_system=254, source_component=190)
        deadline = time.monotonic() + timeout
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = self.master.recv_match(
                type='HEARTBEAT', blocking=True, timeout=1)
            if candidate is not None and (sysid is None
                                          or candidate.get_srcSystem() == sysid):
                heartbeat = candidate
                break
        if heartbeat is None:
            raise RuntimeError(f'{connect}: SysID {sysid} heartbeat not received')
        self.master.target_system = heartbeat.get_srcSystem()
        self.master.target_component = heartbeat.get_srcComponent()
        self.acks = []          # (t, command, result)
        self.statustexts = []   # (t, text)
        self.t0 = time.monotonic()

    # ---------------- send ----------------
    def _long(self, command, *params):
        padded = list(params) + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            command, 0, *padded)

    def send_heading(self, heading_deg, heading_rate=40.0):
        """param1=1 for magnetic heading, param2=degrees, param3=deg/s (must be nonzero)."""
        self._long(MAV_CMD_GUIDED_CHANGE_HEADING,
                   1, float(heading_deg % 360.0), float(heading_rate))

    def send_altitude(self, alt_m, climb_rate=0.0):
        """param3=climb speed (0=maximum), param7=target altitude.

        Because the conversion COMMAND_LONG->COMMAND_INT assumes MAV_FRAME_GLOBAL_RELATIVE_ALT, the altitude is in meters RELATED to HOME (not AMSL).
        """
        self._long(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                   0, 0, float(climb_rate), 0, 0, 0, float(alt_m))

    def send_speed_43000(self, speed_ms, speed_type=0, accel=1.0):
        """Sam Hyams's method: accepted only in GUIDED, with FAILED returned
for a command outside the permitted envelope."""
        self._long(MAV_CMD_GUIDED_CHANGE_SPEED,
                   float(speed_type), float(speed_ms), float(accel))

    def send_speed_178(self, speed_ms, speed_type=0, throttle=-1.0):
        """final_competition/speeds.py pattern: DO_CHANGE_SPEED, param3=-1 (throttle does not change)."""
        self._long(MAV_CMD_DO_CHANGE_SPEED,
                   float(speed_type), float(speed_ms), float(throttle))

    # ---------------- setup_mode ----------------
    def mode(self):
        hb = self.master.messages.get('HEARTBEAT')
        return mavutil.mode_string_v10(hb) if hb else None

    def set_mode(self, name, timeout=15.0):
        mapping = self.master.mode_mapping() or {}
        mode_id = mapping.get(name)
        if mode_id is None:
            raise RuntimeError(f'{name} mode not found')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.master.set_mode(mode_id)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg is not None and getattr(msg, 'custom_mode', None) == mode_id:
                return True
        return False

    # ---------------- read messages ----------------
    def pump(self, verbose=True):
        """Drain incoming messages and record or print ACK and STATUSTEXT messages."""
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == 'COMMAND_ACK':
                row = (time.monotonic() - self.t0, msg.command, msg.result)
                self.acks.append(row)
                if verbose:
                    print(f'  ACK cmd={msg.command} -> {ack_text(msg.result)}')
            elif kind == 'STATUSTEXT':
                text = msg.text.strip()
                self.statustexts.append((time.monotonic() - self.t0, text))
                if verbose:
                    print(f'  TEXT: {text}')

    def telemetry(self):
        """(indicated_airspeed, groundspeed, gps_speed, alt_rel, hdg, throttle)."""
        vfr = self.master.messages.get('VFR_HUD')
        gpi = self.master.messages.get('GLOBAL_POSITION_INT')
        gps_speed = None
        alt_rel = None
        if gpi is not None:
            gps_speed = math.hypot(gpi.vx, gpi.vy) / 100.0
            alt_rel = gpi.relative_alt / 1000.0
        return (
            getattr(vfr, 'airspeed', None),
            getattr(vfr, 'groundspeed', None),
            gps_speed,
            alt_rel,
            getattr(vfr, 'heading', None),
            getattr(vfr, 'throttle', None),
        )

    def request_streams(self, rate_hz=10):
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self._long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       msg_id, int(1e6 / rate_hz))

    # ---------------- interactive mode helpers ----------------
    def drain(self, verbose=False):
        """Drain stale UDP telemetry accumulated while prompting the user."""
        self.pump(verbose=verbose)

    def sample(self, seconds, verbose=False):
        """Average telemetry over a short window, following formation.sample_speeds.

Discard queued messages before collecting fresh VFR_HUD and
GLOBAL_POSITION_INT samples for the specified number of seconds.
Return ias, gps, alt and hdg. Unavailable fields have value None.
        """
        self.drain(verbose=verbose)
        ias, gps, alt, hdg = [], [], [], []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type=['VFR_HUD', 'GLOBAL_POSITION_INT', 'COMMAND_ACK',
                      'STATUSTEXT'],
                blocking=True, timeout=1)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == 'VFR_HUD':
                ias.append(msg.airspeed)
                hdg.append(msg.heading)
            elif kind == 'GLOBAL_POSITION_INT':
                gps.append(math.hypot(msg.vx, msg.vy) / 100.0)
                alt.append(msg.relative_alt / 1000.0)
            elif kind == 'COMMAND_ACK':
                self.acks.append(
                    (time.monotonic() - self.t0, msg.command, msg.result))
            elif kind == 'STATUSTEXT':
                text = msg.text.strip()
                self.statustexts.append((time.monotonic() - self.t0, text))
                if verbose:
                    print(f'  TEXT: {text}')

        def mean(values):
            return sum(values) / len(values) if values else None

        return {'ias': mean(ias), 'gps': mean(gps), 'alt': mean(alt),
                'hdg': hdg[-1] if hdg else None}

    def read_param(self, name, timeout=6.0):
        """Read one parameter as in formation.read_param, returning None when unavailable."""
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode(), -1)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True,
                                         timeout=1)
            if msg is not None and msg.param_id == name:
                return float(msg.param_value)
        return None

    def wait_ack(self, command, timeout=3.0):
        """Wait for ACK of specific command; Return None if it does not arrive."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                self.statustexts.append(
                    (time.monotonic() - self.t0, msg.text.strip()))
                continue
            self.acks.append((time.monotonic() - self.t0, msg.command,
                              msg.result))
            if msg.command == command:
                return msg.result
        return None

    def close(self):
        self.master.close()


# ============================ INTERACTIVE MODE ============================
# The prompting functions accept injected reader and output streams for unit testing.

def prompt_value(label, unit, default, low, high, reader=input, out=sys.stderr):
    """Prompt for a number. Enter accepts the current default, and q quits.

Reject nonnumeric input and values outside [low, high], then prompt
again. When telemetry is unavailable and default is None, an empty
response is rejected. EOF, including Ctrl-D, returns QUIT.
    """
    if default is None:
        question = f'{label} [{unit}, current could not be read, enter value]: '
    else:
        question = f'{label} [{unit}, Enter={default:g} (current)]: '
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print('  End of input. Exiting.', file=out)
            return QUIT
        raw = raw.strip()
        if raw in QUIT_WORDS:
            return QUIT
        raw = raw.replace(',', '.')
        if not raw:
            if default is None:
                print('  current value could not be read, enter a number', file=out)
                continue
            return default
        try:
            value = float(raw)
        except ValueError:
            print('  invalid input (number expected), try again',
                  file=out)
            continue
        if not low <= value <= high:
            print(f'  {value:g} is outside the accepted range [{low:g}, {high:g}], '
                  f'try again', file=out)
            continue
        return value


def prompt_vehicle(reader=input, out=sys.stderr, default=DEFAULT_VEHICLE):
    """Select a vehicle: 1 for Bumblebee, 2 for target. Enter accepts the default, q quits."""
    options = '  '.join(
        f'[{key}] {VEHICLES[key][0]} ({VEHICLES[key][1].rsplit(":", 1)[1]})'
        for key in sorted(VEHICLES))
    question = f'Which vehicle? {options}, Enter={default}: '
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print('  End of input. Exiting.', file=out)
            return QUIT
        raw = raw.strip()
        if raw in QUIT_WORDS:
            return QUIT
        if not raw:
            return default
        if raw in VEHICLES:
            return raw
        by_name = [key for key, spec in VEHICLES.items()
                   if spec[0].lower() == raw.lower()]
        if by_name:
            return by_name[0]
        print(f'  invalid selection "{raw}", expected {sorted(VEHICLES)}',
              file=out)


def prompt_yes_no(question, default=True, reader=input, out=sys.stderr):
    """Ask yes/no. Enter or EOF accepts the default."""
    suffix = '[Y/n, Enter=Y]' if default else '[y/N, Enter=N]'
    while True:
        try:
            raw = reader(f'{question} {suffix}: ')
        except EOFError:
            print(f'  End of input. Using default {"Y" if default else "N"}.', file=out)
            return default
        raw = raw.strip().lower()
        if not raw:
            return default
        if raw in ('y', 'yes'):
            return True
        if raw in ('n', 'no'):
            return False
        print('  Invalid input. Enter y or n.', file=out)


def status_line(snap, mode):
    """One-line status summary."""
    def fmt(value, digits=1, suffix=''):
        return '?' if value is None else f'{value:.{digits}f}{suffix}'
    return (f'currently: setup_mode={mode} hdg={"?" if snap["hdg"] is None else snap["hdg"]}'
            f' alt={fmt(snap["alt"])}m (relative)'
            f' IAS={fmt(snap["ias"])} GPS={fmt(snap["gps"])}')


def interactive(args):
    """Argumentless (or --interactive) operation: question-answer command loop."""
    choice = prompt_vehicle()
    if choice is QUIT:
        print('Exited')
        return 0
    name, connect, sysid, envelope = VEHICLES[choice]

    print(f'[connecting] {name} {connect} (SysID {sysid}) ...')
    try:
        sender = CommandSender(connect, sysid, timeout=args.connect_timeout)
    except OSError as exc:
        print(f'ERROR: {connect} connection failed ({exc}). Port is another '
              f'may have been bound by the process '
              f'(goat_gimbal_aircraft.py uses 14553).', file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    try:
        sender.request_streams()
        mode = sender.mode()
        print(f'[connected] {name} sys={sender.master.target_system} '
              f'comp={sender.master.target_component} setup_mode={mode}')

        # --- GUIDED permission: ASKED but the mode CANNOT be changed NOW --- Targetless GUIDED = LOITER around current position; To prevent the plane from circling while answering the questions, the mode change is postponed until the next sending step after the inputs are received.
        allow_guided = True
        if mode != 'GUIDED':
            print(f'WARNING: mode {mode}; 43000/43001/43002 only in GUIDED\' '
                  f'accepted.', file=sys.stderr)
            print('The mode will not be changed IMMEDIATELY: questions first, then GUIDED, '
                  'commands immediately afterward (GUIDED without a target enters LOITER).')
            allow_guided = prompt_yes_no(
                "Switch to GUIDED after receiving inputs?", default=True)
            if not allow_guided:
                print('[mode] will not be changed; commands may return FAILED',
                      file=sys.stderr)

        # --- live flight envelope (read once) ---
        low = sender.read_param('AIRSPEED_MIN')
        high = sender.read_param('AIRSPEED_MAX')
        if low is None or high is None:
            low, high = envelope
            print(f'[envelope] AIRSPEED_MIN/MAX could not be read, table value '
                  f'[{low:g}, {high:g}] will be used', file=sys.stderr)
        else:
            print(f'[envelope] live AIRSPEED_MIN={low:g} AIRSPEED_MAX={high:g} '
                  f'(The command IAS is clipped to this)')
        gs_low, gs_high = envelope

        print("\nEmpty Enter = CURRENT value. 'Q' to any questions to exit.")
        print('Sequence: questions -> (IAS-GPS) offset -> GUIDED -> commands '
              '(mode is not changed without inputs).\n')

        # --- command loop ---
        while True:
            snap = sender.sample(args.status_sample)
            mode = sender.mode()
            print(status_line(snap, mode))

            hdg_now = None if snap['hdg'] is None else float(snap['hdg'])
            heading = prompt_value('heading', 'degrees', hdg_now,
                                   HEADING_RANGE[0], HEADING_RANGE[1])
            if heading is QUIT:
                break
            altitude = prompt_value('altitude', 'relative m', snap['alt'],
                                    ALT_RANGE[0], ALT_RANGE[1])
            if altitude is QUIT:
                break
            wish_gps = prompt_value('ground speed', 'm/s', snap['gps'],
                                    gs_low, gs_high)
            if wish_gps is QUIT:
                break

            # Speed: formation.equalise_speed pattern -> offset live (IAS-GPS)
            print(f'  ({args.offset_sample:g} s offset measurement...)')
            measured = sender.sample(args.offset_sample)
            if measured['ias'] is None or measured['gps'] is None:
                offset = 0.0
                print('  WARNING: IAS/GPS sample unavailable, assuming zero offset',
                      file=sys.stderr)
            else:
                offset = measured['ias'] - measured['gps']
            wanted = wish_gps + offset
            clamped = min(max(wanted, low), high)
            if abs(clamped - wanted) > 1e-6:
                # Non-envelope command WILL NOT TRIM, IT WILL BE REJECTED (Plane::do_change_speed)
                print(f'  WARNING: {wanted:.2f} outside the flight envelope [{low:g}, {high:g}]; '
                      f'Sending {clamped:.2f}', file=sys.stderr)
            wanted = clamped

            # --- mode: inputs received, ONLY NOW passed to GUIDED --- So there is no human delay between input to GUIDED and command sending; the aircraft takes direction without settling on targetless LOITER.
            mode = sender.mode()
            if mode != 'GUIDED':
                if allow_guided:
                    print("  (Switching to GUIDED ...)")
                    if sender.set_mode('GUIDED'):
                        print('[setup_mode] GUIDED confirmed')
                    else:
                        print('[mode] GUIDED COULD NOT BE MODIFIED; commands FAILED '
                              'will be returned', file=sys.stderr)
                else:
                    print(f'[mode] {mode} protected (permission denied); commands '
                          f'FAILED can return', file=sys.stderr)

            sender.drain()
            sender.send_heading(heading, args.heading_rate)
            ack_hdg = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_HEADING,
                                      args.ack_timeout)
            sender.send_altitude(altitude, args.climb_rate)
            ack_alt = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                                      args.ack_timeout)
            sender.send_speed_43000(wanted, SPEED_TYPE['airspeed'], args.accel)
            ack_spd = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_SPEED,
                                      args.ack_timeout)

            def label(result):
                return 'ACK NO' if result is None else ack_text(result)

            print(f'-> heading {heading:g} deg      : {label(ack_hdg)}')
            print(f'-> altitude {altitude:g} m (view): {label(ack_alt)}')
            print(f'-> IAS {wanted:.1f} commanded (desired ground speed '
                  f'{wish_gps:g}, offset {offset:+.2f}): {label(ack_spd)}')
            print()
    except KeyboardInterrupt:
        print('\n(Ctrl-C) exiting')
    finally:
        sender.close()
    print('Exited')
    return 0


def run(args):
    sender = CommandSender(args.connect, args.sysid)
    print(f'[connected] {args.connect} sys={sender.master.target_system} '
          f'comp={sender.master.target_component} setup_mode={sender.mode()}')
    sender.request_streams()

    if args.mode:
        if sender.set_mode(args.mode):
            print(f'[setup_mode] {args.mode} confirmed')
        else:
            print(f'[mode] {args.mode} COULD NOT BE CHANGED', file=sys.stderr)

    speed_type = SPEED_TYPE[args.speed_type]
    method = args.method
    writer = None
    handle = None
    if args.csv:
        handle = open(args.csv, 'w', newline='')
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'cmd_speed', 'indicated_as', 'groundspeed',
                         'gps_speed', 'alt_rel', 'heading', 'throttle', 'mode'])

    period = 1.0 / args.rate
    end = time.monotonic() + args.hold
    next_print = 0.0
    fallback_done = False
    try:
        while time.monotonic() < end:
            loop_t = time.monotonic() - sender.t0
            if args.heading is not None:
                sender.send_heading(args.heading, args.heading_rate)
            if args.alt is not None:
                sender.send_altitude(args.alt, args.climb_rate)
            if args.speed is not None:
                if method in ('auto', '43000'):
                    sender.send_speed_43000(args.speed, speed_type, args.accel)
                if method == '178':
                    sender.send_speed_178(args.speed, speed_type)
            sender.pump(verbose=not args.quiet)

            # auto: fall back to 178 if 43000 is rejected
            if method == 'auto' and not fallback_done:
                bad = [r for t, c, r in sender.acks
                       if c == MAV_CMD_GUIDED_CHANGE_SPEED and r != 0]
                if bad:
                    print(f'[fallback] 43000 -> {ack_text(bad[-1])}, '
                          f'trying 178 DO_CHANGE_SPEED')
                    method = '178'
                    fallback_done = True

            ias, gs, gps, alt, hdg, thr = sender.telemetry()
            if writer:
                writer.writerow([f'{loop_t:.2f}', args.speed, ias, gs, gps,
                                 alt, hdg, thr, sender.mode()])
            if loop_t >= next_print:
                next_print = loop_t + 1.0
                print(f'  t={loop_t:6.1f} setup_mode={sender.mode()} '
                      f'IAS={ias if ias is None else round(ias, 2)} '
                      f'GS={gps if gps is None else round(gps, 2)} '
                      f'alt={alt if alt is None else round(alt, 1)} '
                      f'hdg={hdg} thr={thr}')
            time.sleep(period)
    finally:
        if handle:
            handle.close()
        sender.close()

    print('\n---ACK summary---')
    seen = {}
    for _, command, result in sender.acks:
        seen.setdefault((command, result), 0)
        seen[(command, result)] += 1
    for (command, result), count in sorted(seen.items()):
        print(f'  cmd {command}: {ack_text(result)} x{count}')
    if not sender.acks:
        print('  (no ACK arrived)')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--connect', default='udpin:127.0.0.1:14553')
    parser.add_argument('--sysid', type=int, default=None)
    parser.add_argument('--heading', type=float, default=None, help='degrees 0-360')
    parser.add_argument('--alt', type=float, default=None, help='metre (home-relative)')
    parser.add_argument('--speed', type=float, default=None, help='m/s')
    parser.add_argument('--speed-type', choices=sorted(SPEED_TYPE), default='airspeed')
    parser.add_argument('--method', choices=('auto', '43000', '178'), default='auto')
    parser.add_argument('--accel', type=float, default=1.0, help='43000 param3 (m/s2)')
    parser.add_argument('--heading-rate', type=float, default=40.0, help='deg/s')
    parser.add_argument('--climb-rate', type=float, default=0.0, help='m/s, 0=max')
    parser.add_argument('--mode', default='GUIDED', help='If "" is given, the mode will not be changed')
    parser.add_argument('--hold', type=float, default=30.0, help='seconds')
    parser.add_argument('--rate', type=float, default=5.0, help='sending Hz.')
    parser.add_argument('--csv', default=None)
    parser.add_argument('--quiet', action='store_true')
    # --- interactive mode ---
    parser.add_argument('--interactive', action='store_true',
                        help='question-answer mode (same as running without arguments): '
                             'questions first, then GUIDED, then command')
    parser.add_argument('--status-sample', type=float, default=1.5,
                        help='interactive: measurement window(s) for status line')
    parser.add_argument('--offset-sample', type=float, default=4.0,
                        help='interactive: (IAS-GPS) offset measurement window (s)')
    parser.add_argument('--ack-timeout', type=float, default=3.0,
                        help='interactive: ACK wait time (s) per command')
    parser.add_argument('--connect-timeout', type=float, default=30.0,
                        help='interactive: heartbeat cooldown (s)')
    args = parser.parse_args()
    if args.rate <= 0 or args.hold < 0:
        parser.error('rate > 0 and hold >= 0')
    if args.status_sample <= 0 or args.offset_sample <= 0:
        parser.error('--status-sample and --offset-sample must be > 0')
    if args.ack_timeout < 0 or args.connect_timeout <= 0:
        parser.error('--ack-timeout >= 0 and --connect-timeout > 0')
    # Call without arguments (or open --interactive) -> question-answer mode.
    if args.interactive or len(sys.argv) == 1:
        return interactive(args)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
