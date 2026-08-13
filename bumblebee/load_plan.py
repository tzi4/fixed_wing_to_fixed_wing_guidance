#!/usr/bin/env python3
"""Upload a QGroundControl plan and verify it by reading it back."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil


GLOBAL_FRAMES = {
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
}

# ArduPilot yalnız bu komutları bir Location olarak saklar
# (AP_Mission::stored_in_location, libraries/AP_Mission/AP_Mission.cpp).
# Listede OLMAYAN öğeler (ör. DO_CHANGE_SPEED 178) geri okunurken frame=0 ve
# x=y=z=0 döner: AP_Mission::mission_cmd_to_mavlink_int paketi sıfırlar ve
# konum alanlarını yalnız stored_in_location() true ise doldurur. Bu yüzden
# bu öğelerde koordinat/frame değil param1..param4 doğrulanır.
LOCATION_COMMANDS = {
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
    mavutil.mavlink.MAV_CMD_DO_LAND_START,
    mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
    mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
    mavutil.mavlink.MAV_CMD_DO_SET_ROI,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
    mavutil.mavlink.MAV_CMD_NAV_PAYLOAD_PLACE,
    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
    mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
    mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
}


def parse_ports(value):
    result = []
    for index, token in enumerate(value.split(','), start=1):
        bits = token.strip().split(':')
        if not bits[0].isdigit():
            raise argparse.ArgumentTypeError(f"geçersiz port: {token}")
        result.append((int(bits[0]), int(bits[1]) if len(bits) == 2 else index))
    return result


def read_plan(path):
    with path.open(encoding='utf-8') as handle:
        data = json.load(handle)
    raw_items = data.get('mission', {}).get('items', [])
    if not raw_items:
        raise ValueError('planda görev öğesi yok')
    home = data.get('mission', {}).get('plannedHomePosition', [])
    if len(home) != 3:
        raise ValueError('plannedHomePosition eksik')
    items = [{
        'seq': 0,
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL,
        'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        'current': 1,
        'autocontinue': 1,
        'params': [0.0, 0.0, 0.0, 0.0, float(home[0]), float(home[1]), float(home[2])],
    }]
    for index, item in enumerate(raw_items, start=1):
        if item.get('type', 'SimpleItem') != 'SimpleItem':
            raise ValueError(f"karmaşık görev öğesi desteklenmiyor: {index - 1}")
        params = [(0.0 if value is None else float(value)) for value in item.get('params', [])]
        if len(params) != 7:
            raise ValueError(f"öğe {index - 1}: yedi parametre gerekli")
        frame = int(item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        items.append({
            'seq': index,
            'frame': frame,
            'command': int(item['command']),
            'current': 0,
            'autocontinue': 1 if item.get('autoContinue', True) else 0,
            'params': params,
        })
    return items


def mission_message(item, system, component, use_int=True):
    params = item['params']
    if not use_int:
        return mavutil.mavlink.MAVLink_mission_item_message(
            system, component, item['seq'], item['frame'], item['command'],
            item['current'], item['autocontinue'], *params,
        )
    if item['frame'] in GLOBAL_FRAMES:
        x, y = int(round(params[4] * 1e7)), int(round(params[5] * 1e7))
    else:
        x, y = int(round(params[4])), int(round(params[5]))
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        system, component, item['seq'], item['frame'], item['command'],
        item['current'], item['autocontinue'], *params[:4], x, y, params[6],
    )


def receive_for(master, types, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type=types, blocking=True, timeout=min(0.5, deadline - time.monotonic()))
        if msg is not None and msg.get_srcSystem() == master.target_system:
            return msg
    return None


def upload_once(master, items):
    system, component = master.target_system, master.target_component
    master.mav.mission_clear_all_send(system, component)
    receive_for(master, ['MISSION_ACK'], 2)
    master.mav.mission_count_send(system, component, len(items))
    sent = set()
    deadline = time.monotonic() + max(20, len(items) * 4)
    while time.monotonic() < deadline:
        msg = receive_for(master, ['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], 3)
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_ACK':
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"görev reddedildi: ACK={msg.type}")
            if len(sent) == len(items):
                return
            continue
        seq = int(msg.seq)
        if not 0 <= seq < len(items):
            raise RuntimeError(f"araç geçersiz sıra istedi: {seq}")
        # Her zaman MISSION_ITEM_INT gönder (araç MISSION_REQUEST ile istese bile).
        # ArduPilot int32 1e7 lat/lon'u tam saklar; float MISSION_ITEM ise float32'ye
        # düşürüp home'dan uzak (ör. ~100+ km) waypoint'lerde ~3e-6° hata üretir ve
        # katı geri-okuma doğrulamasını (abs_tol 2e-6) düşürür. ArduPilot da zaten
        # "GCS should send MISSION_ITEM_INT" diyerek int ister.
        master.mav.send(mission_message(items[seq], system, component, True))
        sent.add(seq)
    raise TimeoutError(f"yükleme zaman aşımı ({len(sent)}/{len(items)} öğe)")


def upload(master, items):
    last_error = None
    max_attempts = 8
    for attempt in range(1, max_attempts + 1):
        try:
            return upload_once(master, items)
        except RuntimeError as exc:
            last_error = exc
            if 'ACK=4' not in str(exc) or attempt == max_attempts:
                raise
            print(f"geçici sıra reddi; görev protokolü yeniden başlatılıyor ({attempt}/{max_attempts})")
            time.sleep(1)
    raise last_error


def download(master):
    system, component = master.target_system, master.target_component
    master.mav.mission_request_list_send(system, component)
    count_msg = receive_for(master, ['MISSION_COUNT'], 8)
    if count_msg is None:
        raise TimeoutError('MISSION_COUNT alınamadı')
    result = []
    for seq in range(int(count_msg.count)):
        master.mav.mission_request_int_send(system, component, seq)
        msg = receive_for(master, ['MISSION_ITEM_INT', 'MISSION_ITEM'], 5)
        if msg is None:
            raise TimeoutError(f"görev öğesi geri okunamadı: {seq}")
        result.append(msg)
    master.mav.mission_ack_send(system, component, mavutil.mavlink.MAV_MISSION_ACCEPTED)
    return result


def verify(expected, actual):
    if len(expected) != len(actual):
        raise RuntimeError(f"geri okuma sayısı farklı: {len(actual)} != {len(expected)}")
    for want, got in zip(expected, actual):
        frame_alias = {5: 0, 6: 3, 11: 10}
        got_frame = frame_alias.get(int(got.frame), int(got.frame))
        if int(got.seq) != want['seq'] or int(got.command) != want['command']:
            raise RuntimeError(f"öğe {want['seq']} kimliği farklı")
        # Konum saklamayan öğeler (ör. DO_CHANGE_SPEED): frame/koordinat geri
        # okumada her zaman 0 döner, onun yerine param1..param4 doğrulanır.
        if want['command'] not in LOCATION_COMMANDS:
            for index in range(4):
                if not math.isclose(float(getattr(got, f'param{index + 1}')),
                                    want['params'][index], abs_tol=1e-4):
                    raise RuntimeError(
                        f"öğe {want['seq']} (komut {want['command']}) "
                        f"param{index + 1} farklı: alınan="
                        f"{getattr(got, f'param{index + 1}')} "
                        f"beklenen={want['params'][index]}")
            continue
        if got_frame != want['frame']:
            raise RuntimeError(f"öğe {want['seq']} kimliği farklı")
        # ArduPlane sıra 0 home koordinatını aktif EKF home değeriyle geri
        # yazabilir; görev öğeleri 1..N bire bir doğrulanır.
        if want['seq'] == 0:
            continue
        params = want['params']
        got_x = float(got.x) / 1e7 if got.get_type() == 'MISSION_ITEM_INT' and want['frame'] in GLOBAL_FRAMES else float(got.x)
        got_y = float(got.y) / 1e7 if got.get_type() == 'MISSION_ITEM_INT' and want['frame'] in GLOBAL_FRAMES else float(got.y)
        if not (math.isclose(got_x, params[4], abs_tol=2e-6) and math.isclose(got_y, params[5], abs_tol=2e-6) and math.isclose(float(got.z), params[6], abs_tol=0.1)):
            raise RuntimeError(f"öğe {want['seq']} koordinatı farklı: alınan=({got_x},{got_y},{float(got.z)}) beklenen=({params[4]},{params[5]},{params[6]})")


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=root / 'missions' / 'dumduz.plan')
    parser.add_argument('--ports', type=parse_ports, default=parse_ports('14551:1,14561:2'), help='virgülle ayrılmış PORT[:SYSID]')
    parser.add_argument('--timeout', type=float, default=60)
    args = parser.parse_args()
    items = read_plan(args.plan.resolve())
    failures = []
    for port, expected_sysid in args.ports:
        master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=255, source_component=190)
        try:
            print(f"[{port}] SysID {expected_sysid} heartbeat bekleniyor...", flush=True)
            heartbeat = None
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                candidate = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if candidate is not None and candidate.get_srcSystem() == expected_sysid:
                    heartbeat = candidate
                    break
            received_sysid = 0 if heartbeat is None else heartbeat.get_srcSystem()
            if heartbeat is None or received_sysid != expected_sysid:
                raise RuntimeError(f"beklenen SysID {expected_sysid}, alınan {received_sysid}")
            master.target_system = received_sysid
            master.target_component = heartbeat.get_srcComponent()
            print(f"[{port}] {len(items)} öğe yükleniyor")
            upload(master, items)
            verify(items, download(master))
            print(f"[{port}] görev geri okunarak doğrulandı")
        except Exception as exc:
            failures.append(f"{port}: {exc}")
            print(f"[{port}] HATA: {exc}", file=sys.stderr)
        finally:
            master.close()
        time.sleep(0.5)
    if failures:
        raise SystemExit('Görev yükleme başarısız: ' + '; '.join(failures))


if __name__ == '__main__':
    main()
