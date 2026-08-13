#!/usr/bin/env python3
"""
TRIM_THROTTLE Gönderici
-----------------------
Mod 1: Kullanıcıdan 50-127 arası bir değer alır, sürekli bu değeri gönderir.
Mod 2: 100'den başlayıp 70'e kadar saniyede 1 azaltarak iner,
       sonra saniyede 1 artırarak 100'e çıkar, tekrar 70'e iner... (döngü)

NOT: Sadece GUIDED modunda çalışır. GUIDED değilse bekler.
"""

import time
import sys
import math
from pymavlink import mavutil

UAV_PORT = '14553'

def connect():
    """MAVLink bağlantısı kur."""
    print(f"[*] MAVLink bağlantısı kuruluyor: 127.0.0.1:{UAV_PORT}")
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{UAV_PORT}')
    print("[*] Heartbeat bekleniyor...")
    master.wait_heartbeat()
    print(f"[+] Bağlantı kuruldu! (system={master.target_system}, component={master.target_component})")
    return master


def get_flight_mode(master):
    """Güncel uçuş modunu döndür. Cache'i güncellemek için recv_match yapar."""
    # Non-blocking recv_match ile cache'i tazele
    master.recv_match(blocking=False)
    hb = master.messages.get('HEARTBEAT', None)
    if hb:
        return mavutil.mode_string_v10(hb)
    return None


def is_guided(master):
    """GUIDED modunda mıyız?"""
    return get_flight_mode(master) == 'GUIDED'


def get_current_heading(master):
    """Güncel heading'i derece olarak döndür (0-360)."""
    att = master.messages.get('ATTITUDE', None)
    if att:
        hdg = math.degrees(att.yaw)
        if hdg < 0:
            hdg += 360.0
        return hdg
    return None


def get_current_altitude(master):
    """Güncel irtifayı metre olarak döndür (AMSL)."""
    loc = master.messages.get('GLOBAL_POSITION_INT', None)
    if loc:
        return loc.alt / 1000.0
    return None


def send_trim_throttle(master, value):
    """TRIM_THROTTLE parametresini gönder."""
    value = max(0, min(127, value))
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'TRIM_THROTTLE',
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )


def send_heading(master, heading_deg):
    """MAV_CMD_GUIDED_CHANGE_HEADING ile heading gönder."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        43002,  # MAV_CMD_GUIDED_CHANGE_HEADING
        0,
        1,            # param1: raw magnetic heading
        heading_deg,  # param2: hedef heading
        40,           # param3: heading rate (deg/s)
        0, 0, 0, 0
    )


def send_altitude(master, alt_m):
    """MAV_CMD_GUIDED_CHANGE_ALTITUDE ile irtifa gönder."""
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
    """Mod 1: Sabit değer gönderimi."""
    while True:
        try:
            val = int(input("\n[Mod 1] 50-127 arası bir değer girin: "))
            if 50 <= val <= 127:
                break
            print("  ⚠  Değer 50-127 arasında olmalı!")
        except ValueError:
            print("  ⚠  Geçerli bir sayı girin!")

    print(f"\n[Mod 1] TRIM_THROTTLE = {val} sürekli gönderiliyor... (Ctrl+C ile çıkış)")
    print("[*] Sadece GUIDED modunda gönderim yapılır. Heading + Altitude sabitlenir.")
    print("-" * 50)

    hold_hdg = None
    hold_alt = None
    try:
        while True:
            mode = get_flight_mode(master)
            if mode == 'GUIDED':
                # İlk GUIDED girişinde heading/altitude yakala
                if hold_hdg is None:
                    hold_hdg = get_current_heading(master)
                    hold_alt = get_current_altitude(master)
                    print(f"  ✈ Heading={hold_hdg:.1f}° Altitude={hold_alt:.1f}m sabitlendi")
                
                send_trim_throttle(master, val)
                if hold_hdg is not None:
                    send_heading(master, hold_hdg)
                if hold_alt is not None:
                    send_altitude(master, hold_alt)
                print(f"  ✓ [{mode}] THR={val}  HDG={hold_hdg:.1f}°  ALT={hold_alt:.1f}m")
                time.sleep(0.2)  # 5 Hz gönderim
            else:
                hold_hdg = None
                hold_alt = None
                print(f"  ⏳ GUIDED bekleniyor... (şu an: {mode or '?'})")
                time.sleep(1.0)
    except KeyboardInterrupt:
        print(f"\n[!] Mod 1 sonlandırıldı. Son değer: {val}")


def mode2(master):
    """Mod 2: 100 → 70 → 100 → 70 ... (saniyede 1 adım)"""
    current = 100
    direction = -1  # -1: azalıyor, +1: artıyor
    
    print(f"\n[Mod 2] Başlangıç: {current}")
    print("  100 → 70 (↓1/sn) → 100 (↑1/sn) → 70 (↓1/sn) ... döngü")
    print("[*] Sadece GUIDED modunda gönderim yapılır. Heading + Altitude sabitlenir.")
    print("  Ctrl+C ile çıkış")
    print("-" * 50)

    hold_hdg = None
    hold_alt = None
    try:
        while True:
            mode = get_flight_mode(master)
            if mode == 'GUIDED':
                # İlk GUIDED girişinde heading/altitude yakala
                if hold_hdg is None:
                    hold_hdg = get_current_heading(master)
                    hold_alt = get_current_altitude(master)
                    print(f"  ✈ Heading={hold_hdg:.1f}° Altitude={hold_alt:.1f}m sabitlendi")
                
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

                # Alt limite ulaştı → yön değiştir (yukarı)
                if current <= 70:
                    current = 70
                    direction = 1
                # Üst limite ulaştı → yön değiştir (aşağı)
                elif current >= 100:
                    current = 100
                    direction = -1
            else:
                hold_hdg = None
                hold_alt = None
                print(f"  ⏳ GUIDED bekleniyor... (şu an: {mode or '?'})")
                time.sleep(1.0)

    except KeyboardInterrupt:
        print(f"\n[!] Mod 2 sonlandırıldı. Son değer: {current}")


def main():
    master = connect()

    print("\n" + "=" * 50)
    print("  TRIM_THROTTLE Gönderici")
    print("=" * 50)
    print("  Mod 1: Sabit değer (50-127 arası)")
    print("  Mod 2: Otomatik salınım (100 ↔ 70)")
    print("  ⚠  Sadece GUIDED modunda çalışır!")
    print("=" * 50)

    while True:
        try:
            mode = input("\nMod seçin (1 veya 2): ").strip()
            if mode in ('1', '2'):
                break
            print("  ⚠  Lütfen 1 veya 2 girin!")
        except (ValueError, EOFError):
            print("  ⚠  Geçerli bir değer girin!")

    if mode == '1':
        mode1(master)
    else:
        mode2(master)


if __name__ == '__main__':
    main()
