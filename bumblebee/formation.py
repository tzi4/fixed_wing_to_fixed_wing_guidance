#!/usr/bin/env python3
"""Once hedefi arm et, sonra hedef ve Bumblebee'yi AUTO'ya al.

Argumansiz calistirilirsa (ve stdin bir terminalse) su uc soruyu sorar:
  * target (SysID 2) yer hizi   [m/s, Enter=20]
  * bumblebee (SysID 1) yer hizi [m/s, Enter=20]
  * kalkislar arasi sure         [s,   Enter=2]
Bos Enter = koseli parantezdeki varsayilan; gecersiz veya zarf disi girdide
soru tekrarlanir.

Otomasyon icin: komut satirindan verilen deger icin o soru SORULMAZ; `--yes`
verilirse ya da stdin bir terminal degilse hicbir soru sorulmaz (delay 2 s,
hiz verilmemisse hiz komutu gonderilmez).
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil


AIRCRAFT = ((14561, 2, 'target'), (14551, 1, 'bumblebee'))

# Ucak basina kabul edilen YER hizi araligi (etkilesimli soruda on kontrol;
# equalise_speed ayrica AIRSPEED_MIN/MAX ile kirpar).
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
        raise RuntimeError(f"port {port}: SysID {expected_sysid} bekleniyordu, {received_sysid} alındı")
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
    raise TimeoutError(f"SysID {master.target_system} arm olmadı")


def set_auto(master, timeout):
    mapping = master.mode_mapping() or {}
    mode_id = mapping.get('AUTO')
    if mode_id is None:
        raise RuntimeError('AUTO modu bulunamadı')
    master.set_mode(mode_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and getattr(msg, 'custom_mode', None) == mode_id:
            return
    raise TimeoutError(f"SysID {master.target_system} AUTO olmadı")


def read_param(master, name, timeout=6.0):
    """Tek parametreyi oku; alinamazsa None."""
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
    """(ortalama IAS, ortalama GPS yer hizi, ortalama irtifa) ornekle."""
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
        raise RuntimeError('telemetri ornegi alinamadi (VFR_HUD/GLOBAL_POSITION_INT)')
    return (sum(ias) / len(ias), sum(gps) / len(gps),
            sum(alt) / len(alt) if alt else float('nan'))


def wait_cruise(masters, target_alt, timeout):
    """Iki ucak da hedef irtifaya ulasana kadar bekle (kalkis + kot alma bitsin)."""
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
    print(f"UYARI: {', '.join(missing)} {target_alt} m'ye ulasmadi; yine de devam",
          file=sys.stderr)
    return False


def prompt_float(label, unit, default, low, high, reader=input, out=sys.stderr):
    """Etkilesimli sayi sor.

    Bos Enter -> default. Sayi olmayan girdi ya da [low, high] disi deger
    uyari basip soruyu tekrarlatir. Girdi akisi biterse (EOF) varsayilan
    kullanilir.
    """
    question = f"{label} [{unit}, Enter={default:g}]: "
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print(f"  girdi bitti, varsayilan {default:g} kullanildi", file=out)
            return default
        raw = raw.strip().replace(',', '.')
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("  gecersiz girdi (sayi bekleniyordu), tekrar deneyin", file=out)
            continue
        if not low <= value <= high:
            print(f"  {value:g} kabul araligi disinda [{low:g}, {high:g}], "
                  f"tekrar deneyin", file=out)
            continue
        return value


def resolve_settings(args, interactive, reader=input, out=sys.stderr):
    """Komut satiri + (gerekiyorsa) sorulardan nihai hiz/delay degerlerini uret.

    Komut satirindan verilen deger icin soru sorulmaz. interactive False ise
    hic soru sorulmaz: delay varsayilani DEFAULT_DELAY, verilmeyen hiz None
    kalir (= o ucaga hic hiz komutu gonderilmez, eski davranis).
    """
    speeds = {}
    for _, sysid, name in AIRCRAFT:
        value = args.speed_by_name.get(name)
        if value is None and interactive:
            low, high = SPEED_ENVELOPE[name]
            value = prompt_float(f"{name} (SysID {sysid}) yer hizi", 'm/s',
                                 DEFAULT_SPEED, low, high, reader=reader, out=out)
        speeds[name] = value
    delay = args.delay
    if delay is None:
        if interactive:
            delay = prompt_float('kalkislar arasi sure', 's', DEFAULT_DELAY,
                                 DELAY_RANGE[0], DELAY_RANGE[1],
                                 reader=reader, out=out)
        else:
            delay = DEFAULT_DELAY
    return speeds, delay


def equalise_speed(masters, desired_gps, settle, sample):
    """Her ucagin YER hizini kendi hedefine (desired_gps[isim]) getir.

    Sensor yok (ARSPD_TYPE 0) oldugu icin TECS'in kullandigi hava hizi
    sentetiktir:  IAS = |V_gps - W_EKF3|.  Gercek ruzgar sifir olsa bile EKF3
    ruzgar tahmini ucak basina farkli kayar (olcum: avci +0.29, hedef -2.19
    m/s), yani IKI UCAGA AYNI hava hizini vermek yer hizlarini EsITLEMEZ.
    Bu yuzden her ucakta once kendi (IAS - GPS) farki OLCULUR, sonra
    DO_CHANGE_SPEED ile hava hizi hedefi = istenen yer hizi + fark gonderilir.
    Boylece yer hizi istenen degere oturur. AUTO modda gecerli komut 178'dir
    (43000 yalniz GUIDED icin).

    desired_gps: {ucak adi: istenen yer hizi (m/s) veya None}. None olan ucaga
    hic komut gonderilmez.
    """
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                   mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        for master, _ in masters:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 200000, 0, 0, 0, 0, 0)
    print(f"[hiz] iki ucak da >=45 m bekleniyor, sonra {settle:.0f} s oturma")
    wait_cruise(masters, 45.0, timeout=max(60.0, settle))
    time.sleep(settle)
    for master, name in masters:
        wish = desired_gps.get(name)
        if wish is None:
            print(f"[{name}] hiz istegi yok, komut gonderilmiyor")
            continue
        ias, gps, alt = sample_speeds(master, sample)
        offset = ias - gps
        wanted = wish + offset
        low = read_param(master, 'AIRSPEED_MIN')
        high = read_param(master, 'AIRSPEED_MAX')
        print(f"[{name}] olcum: IAS={ias:.2f} GPS={gps:.2f} alt={alt:.0f} "
              f"fark={offset:+.2f} -> hava hizi hedefi {wanted:.2f}")
        if low is not None and high is not None:
            clamped = min(max(wanted, low), high)
            if abs(clamped - wanted) > 1e-6:
                # Zarf disi komut KIRPILMAZ, REDDEDILIR: ArduPlane
                # Plane::do_change_speed() araligi tutmayan istegi hic
                # uygulamaz (COMMAND_ACK=FAILED) ve ucak eski hizinda kalir.
                print(f"[{name}] UYARI: {wanted:.2f} zarf disi "
                      f"[{low:.1f}, {high:.1f}]; {clamped:.2f} gonderiliyor",
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
        label = ACK_NAME.get(result, 'ACK YOK')
        print(f"[{name}] DO_CHANGE_SPEED {wanted:.2f} m/s -> {label}")
        if result != 0:
            print(f"[{name}] UYARI: hiz komutu kabul edilmedi ({label})",
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
                        help=f'kalkislar arasi sure (s); verilmezse etkilesimli '
                             f'sorulur, varsayilan {DEFAULT_DELAY:g}')
    parser.add_argument('--yes', action='store_true',
                        help='hicbir soru sorma, onay isteme (tam otomatik)')
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--force-arm-after', type=float, default=30,
                        help='SITL pre-arm beklemesinden sonra force-arm süresi')
    parser.add_argument('--nudge', type=float, default=1.0)
    parser.add_argument('--speed', type=float, default=None,
                        help='iki ucak icin de ayni YER hizi (m/s) '
                             '(geriye uyumluluk kisayolu)')
    parser.add_argument('--target-speed', type=float, default=None,
                        help='target (SysID 2) icin istenen YER hizi (m/s)')
    parser.add_argument('--bumblebee-speed', type=float, default=None,
                        help='bumblebee (SysID 1) icin istenen YER hizi (m/s)')
    parser.add_argument('--speed-settle', type=float, default=25.0,
                        help='hiz: irtifaya ulastiktan sonra oturma suresi (s)')
    parser.add_argument('--speed-sample', type=float, default=8.0,
                        help='hiz: IAS/GPS ortalama alma penceresi (s)')
    args = parser.parse_args()
    if args.delay is not None and args.delay < 0:
        parser.error('--delay negatif olamaz')
    if args.nudge < 0 or args.force_arm_after < 0:
        parser.error('süreler negatif olamaz')
    for flag, value in (('--speed', args.speed),
                        ('--target-speed', args.target_speed),
                        ('--bumblebee-speed', args.bumblebee_speed)):
        if value is not None and value <= 0:
            parser.error(f'{flag} pozitif olmali')
    if args.speed_settle < 0 or args.speed_sample <= 0:
        parser.error('--speed-settle >= 0 ve --speed-sample > 0 olmali')

    # Ucak basina istenen hiz: ozel bayrak > --speed kisayolu > (soru/None).
    args.speed_by_name = {
        'target': args.target_speed if args.target_speed is not None else args.speed,
        'bumblebee': args.bumblebee_speed if args.bumblebee_speed is not None else args.speed,
    }
    for name, value in args.speed_by_name.items():
        low, high = SPEED_ENVELOPE[name]
        if value is not None and not low <= value <= high:
            print(f"UYARI: {name} icin {value:g} m/s zarf disi [{low:g}, {high:g}]; "
                  f"AIRSPEED_MIN/MAX ile kirpilacak", file=sys.stderr)

    interactive = sys.stdin.isatty() and not args.yes
    speeds, delay = resolve_settings(args, interactive)
    want_speed = any(value is not None for value in speeds.values())

    masters = []
    try:
        for port, sysid, name in AIRCRAFT:
            print(f"[{name}] {port} bağlantısı kuruluyor")
            master = connect(port, sysid, args.timeout)
            arm(master, args.timeout, args.force_arm_after)
            masters.append((master, name))
            print(f"[{name}] ARMED")
        if interactive:
            plan = ', '.join(
                f"{name}={speeds[name]:g} m/s" if speeds[name] is not None
                else f"{name}=hiz komutu yok" for _, _, name in AIRCRAFT)
            try:
                input(f"Hedef, {delay:.2f} s sonra Bumblebee kalkacak "
                      f"({plan}). ENTER: ")
            except EOFError:
                print()
        for index, (master, name) in enumerate(masters):
            set_auto(master, args.timeout)
            nudge(master, args.nudge)
            print(f"[{name}] AUTO ve kalkış dürtüsü gönderildi")
            if index == 0:
                time.sleep(delay)
        if want_speed:
            equalise_speed(masters, speeds, args.speed_settle,
                           args.speed_sample)
    except Exception as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        for master, _ in masters:
            master.close()


if __name__ == '__main__':
    main()
