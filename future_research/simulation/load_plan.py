import json
import time
import os
from pymavlink import mavutil
from pymavlink import mavwp

PLAN_FILE_1 = "/home/tzi4/Application/straight.plan"
PLAN_FILE_2 = "/home/tzi4/Applications/straight.plan"

if os.path.exists(PLAN_FILE_1):
    PLAN_FILE = PLAN_FILE_1
elif os.path.exists(PLAN_FILE_2):
    PLAN_FILE = PLAN_FILE_2
else:
    PLAN_FILE = PLAN_FILE_2 # Default

AIRCRAFT_PORTS = [14551, 14561]

def process_plan_file(filepath, target_system, target_component):
    with open(filepath, 'r') as f:
        plan_data = json.load(f)
        
    items = plan_data.get("mission", {}).get("items", [])
    home_pos = plan_data.get("mission", {}).get("plannedHomePosition", [0.0, 0.0, 0.0])
    
    wploader = mavwp.MAVWPLoader(target_system=target_system, target_component=target_component)
    
    wploader.add(mavutil.mavlink.MAVLink_mission_item_int_message(
        target_system, target_component,
        0, 
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 
        0, 0, 0, 0, 
        int(home_pos[0] * 1e7),
        int(home_pos[1] * 1e7),
        home_pos[2]
    ))
    
    seq = 1
    for item in items:
        command = item.get("command")
        frame = item.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        params = item.get("params", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        params = [float(p) if p is not None else 0.0 for p in params]
        autocontinue = 1 if item.get("autoContinue", True) else 0
        
        if frame in [mavutil.mavlink.MAV_FRAME_GLOBAL, 
                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                     mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT]:
            x_int = int(params[4] * 1e7)
            y_int = int(params[5] * 1e7)
        else:
            x_int = int(params[4])
            y_int = int(params[5])
            
        z = params[6]
        
        msg = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system, target_component,
            seq,
            frame, command,
            0, autocontinue,
            params[0], params[1], params[2], params[3],
            x_int, y_int, z
        )
        wploader.add(msg)
        seq += 1
        
    return wploader

def upload_mission(port):
    print(f"\n[{port}] Establishing connection (via pymavlink)...")
    try:
        # Connect directly via Pymavlink (without using DroneKit, for MISSION_ITEM_INT support)
        master = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
        
        print(f"[{port}] Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        
        target_system = master.target_system
        target_component = master.target_component
        print(f"[{port}] Heartbeat received (SysID: {target_system}, CompID: {target_component}).")
        
        try:
            wploader = process_plan_file(PLAN_FILE, target_system, target_component)
        except Exception as e:
            print(f"[{port}] Plan reading error: {e}")
            master.close()
            return
            
        print(f"[{port}] Clearing old task...")
        master.mav.mission_clear_all_send(target_system, target_component)
        ack = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=2)
        
        print(f"[{port}] {wploader.count()} WPs are being sent (with High Precision MISSION_ITEM_INT)...")
        master.mav.mission_count_send(target_system, target_component, wploader.count())
        
        for i in range(wploader.count()):
            msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
            if not msg:
                print(f"[{port}] ERROR: MISSION_REQUEST did not come from the vehicle.")
                master.close()
                return
            
            # pymavlink sends the actual message stored in wploader
            master.mav.send(wploader.wp(msg.seq))
            print(f"[{port}] WP {msg.seq} submitted.")
            
        ack = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[{port}] Task successfully loaded!")
        else:
            print(f"[{port}] WARNING: Task confirmation (ACK) could not be received or was rejected: {ack}")
            
        master.close()
    except Exception as e:
        print(f"[{port}] Error occurred: {e}")

def main():
    print(f"High precision blueprint loader launched.")
    print(f"Task file: '{PLAN_FILE}'")
    
    # Load the mission into both vehicles
    for port in AIRCRAFT_PORTS:
        upload_mission(port)
        time.sleep(1)
        
    print("\nTask uploading to all vehicles has been completed.")

if __name__ == '__main__':
    main()
