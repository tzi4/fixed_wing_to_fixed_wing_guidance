import sys
import time
from pymavlink import mavutil

def set_airspeed(master, target_speed_ms):
    print(f"[{master.address}] Hedef hiz gonderiliyor: {target_speed_ms} m/s")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        178,            # MAV_CMD_DO_CHANGE_SPEED
        0,              # confirmation
        0,              # param1: SPEED_TYPE (0 = airspeed, 1 = groundspeed)
        target_speed_ms,# param2: speed
        -1,             # param3: throttle (-1 means no change)
        0, 0, 0, 0      # unused
    )

def main():
    # Argüman olarak hız verilmişse onu al, yoksa 18.0 kullan
    if len(sys.argv) > 1:
        target_speed = float(sys.argv[1])
    else:
        target_speed = 18.0

    print(f"Hedef hiz ayarlaniyor: {target_speed} m/s\n")
    
    # Uçak 1
    print("Ucak 1 (Port 14551) ile baglanti kuruluyor...")
    master1 = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    master1.wait_heartbeat()
    print("Ucak 1 Heartbeat alindi.")
    
    # Uçak 2
    print("Ucak 2 (Port 14561) ile baglanti kuruluyor...")
    master2 = mavutil.mavlink_connection('udp:127.0.0.1:14561')
    master2.wait_heartbeat()
    print("Ucak 2 Heartbeat alindi.")
    
    print("\nHiz komutlari gonderiliyor...")
    # Komutları gönder
    set_airspeed(master1, target_speed)
    set_airspeed(master2, target_speed)
    
    # Mesajların gitmesi için kısa bir bekleme
    time.sleep(1)
    print("\nKomutlar basariyla iletildi.")

if __name__ == '__main__':
    main()
