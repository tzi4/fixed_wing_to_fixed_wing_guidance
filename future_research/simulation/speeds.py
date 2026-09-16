import sys
import time
from pymavlink import mavutil

def set_airspeed(master, target_speed_ms):
    print(f"[{master.address}] Sending target speed: {target_speed_ms} m/s")
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
    # If speed is given as an argument, take it, otherwise use 18.0
    if len(sys.argv) > 1:
        target_speed = float(sys.argv[1])
    else:
        target_speed = 18.0

    print(f"Setting target speed: {target_speed} m/s\n")
    
    # plane 1
    print("Establishing a connection with Aircraft 1 (Port 14551)...")
    master1 = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    master1.wait_heartbeat()
    print("Aircraft 1 Heartbeat received.")
    
    # plane 2
    print("Establishing connection with Aircraft 2 (Port 14561)...")
    master2 = mavutil.mavlink_connection('udp:127.0.0.1:14561')
    master2.wait_heartbeat()
    print("Aircraft 2 Heartbeat was taken.")
    
    print("\nSending speed commands...")
    # Send commands
    set_airspeed(master1, target_speed)
    set_airspeed(master2, target_speed)
    
    # Short wait for messages to go out
    time.sleep(1)
    print("\nCommands were transmitted successfully.")

if __name__ == '__main__':
    main()
