"""
Load aircraft parameters from .parm files into SITL.

Connections:
  plane1.parm -> instance 0, SYSID 1, tcp:127.0.0.1:5762.
  plane2.parm -> instance 1, SYSID 2, tcp:127.0.0.1:5772.

Usage:
  python3 set_params.py     # interactive prompts
  python3 set_params.py 1   # aircraft 1
  python3 set_params.py 2   # aircraft 2
  python3 set_params.py 3   # both aircraft
"""
import os
import sys
from pymavlink import mavutil

# --- DESCRIPTION OF EACH AIRCRAFT ---
AIRCRAFT = [
    {
        "name": "Plane 1",
        "parm_file": "plane1.parm",
        "port": "tcp:127.0.0.1:5762",   # Instance 0
    },
    {
        "name": "Plane 2",
        "parm_file": "plane2.parm",
        "port": "tcp:127.0.0.1:5772",   # Instance 1
    },
]


def parse_parm_file(filepath, run_mode):
    """
    Reads the .parm file, returns a list of (param_name, value).     Lines starting with # and empty lines are skipped.
    """
    params = []
    if not os.path.isfile(filepath):
        print(f"  ⚠ File not found: {filepath}")
        return params

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                param_name = parts[0]
                if run_mode == "1" and param_name == "AIRSPEED_CRUISE":
                    continue
                if run_mode == "2" and param_name == "TRIM_THROTTLE":
                    continue
                try:
                    param_value = float(parts[1])
                    params.append((param_name, param_value))
                except ValueError:
                    print(f"  ⚠ Invalid value omitted: {line}")
    return params


def apply_params(port, params, timeout=15):
    """
    It connects to the specified port and sends the parameter list.
    """
    try:
        print(f"  Connecting: {port}...")
        master = mavutil.mavlink_connection(port)
        master.wait_heartbeat(timeout=timeout)
        sysid = master.target_system
        print(f"  ✓ Heartbeat received (SYSID={sysid})")

        for param_name, param_value in params:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_name.encode('utf-8'),
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Wait for confirmation
            ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if ack and ack.param_id.strip('\x00') == param_name:
                print(f"  ✓ {param_name} = {ack.param_value}")
            else:
                print(f"  ⚠ {param_name} = {param_value} (confirmation not received but sent)")

        master.close()
        return True
    except Exception as e:
        print(f"  ✗ ERROR: {e}")
        return False


def get_run_mode():
    """Retrieves the mode selection from the terminal."""
    while True:
        print("\nHangi setup_mode:")
        print("  1 - trim_throttle (Current operation, TRIM_THROTTLE is used)")
        print("  2 - airspeed (parameter AIRSPEED_CRUISE is sent)")
        ans = input("\nSelection (1/2): ").strip()
        if ans in ['1', '2']:
            return ans
        print("⚠ Please enter 1 or 2.")


def get_choice():
    """Get selection in terminal or from argument."""
    # If there is a command line argument, use it
    if len(sys.argv) > 1:
        return sys.argv[1]

    # Or ask the user
    print("\nSelect target:")
    print("  1 - Plane 1 only")
    print("  2 - Plane 2 only")
    print("  3 - Both planes")
    return input("\nSelection (1/2/3): ").strip()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 50)
    print("  AIRPLANE PARAMETER SETTER")
    print("=" * 50)

    run_mode = get_run_mode()
    choice = get_choice()

    if choice == "1":
        targets = [AIRCRAFT[0]]
    elif choice == "2":
        targets = [AIRCRAFT[1]]
    elif choice == "3":
        targets = AIRCRAFT
    else:
        print(f"\n✗ Invalid selection: '{choice}' (enter 1, 2 or 3)")
        sys.exit(1)

    for ac in targets:
        parm_path = os.path.join(script_dir, ac["parm_file"])
        print(f"\n--- {ac['name']} ({ac['parm_file']}) ---")

        params = parse_parm_file(parm_path, run_mode)
        if not params:
            print("  No parameters or file not found, skipping.")
            continue

        print(f"  {len(params)} parameters read:")
        for name, val in params:
            print(f"    {name} = {val}")

        apply_params(ac["port"], params)

    print("\n" + "=" * 50)
    print("  COMPLETED")
    print("=" * 50)
