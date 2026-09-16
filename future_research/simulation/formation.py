from mavlinkHandler import MAVLinkHandlerDronekit as MAVLinkHandler
import time

aircraft_ports = [14551, 14561]

def prepare_aircraft(port):
    print(f"[{port}] Connecting...")
    try:
        handler = MAVLinkHandler(f'127.0.0.1:{port}', _wait_ready=True)
        
        # Arming only
        print(f"[{port}] Arming...")
        handler.arm_vehicle()
        
        # Wait for arming
        for _ in range(30):
            if handler.master.armed:
                print(f"[{port}] ARMED.")
                return handler
            handler.arm_vehicle()
            time.sleep(1)
            
        print(f"[{port}] Failed to ARM.")
        return None
    except Exception as e:
        print(f"[{port}] Error: {e}")
        return None



def main():
    handlers = {}
    
    # 1. Connect and Arm
    print("\n--- AIRCRAFT PREPARATION ---")
    for port in aircraft_ports:
        h = prepare_aircraft(port)
        if h:
            handlers[port] = h
        time.sleep(0.5)
        
    if len(handlers) < len(aircraft_ports):
        print("Warning: Some aircraft failed to prepare. Proceeding with active ones.")

    # 2. Get User Input for Delay
    try:
        delay_str = input("\nEnter delay between takeoffs (seconds) [default: 0.05]: ")
        delay = float(delay_str) if delay_str.strip() else 0.05
    except ValueError:
        print("Invalid input, using default 0.05 seconds.")
        delay = 0.05

    print(f"\nPlan: Launch Plane 2 (Target) -> Wait {delay}s -> Launch Plane 1 (Tracker)")
    input("Press ENTER to START sequence...")

    # 3. Sequenced Launch
    # Define launch helper
    def do_launch(port):
        if port not in handlers:
            print(f"[{port}] Skipping (Not connected).")
            return
        
        h = handlers[port]
        print(f"\n[{port}] Launching...")
        h.set_mode('AUTO')
        # Throttle Nudge
        h.master.channels.overrides = {'3': 2200}
        time.sleep(1.0) # Hold nudge briefly
        h.master.channels.overrides = {} 
        print(f"[{port}] Launch command sent (Throttle released).")

    # Launch Plane 2 (14561)
    do_launch(14561)

    # Wait
    if 14561 in handlers and 14551 in handlers:
        print(f"\nWaiting {delay} seconds...")
        time.sleep(delay)

    # Launch Plane 1 (14551)
    do_launch(14551)

    print("\nSequence completed.")

if __name__ == "__main__":
    main()
