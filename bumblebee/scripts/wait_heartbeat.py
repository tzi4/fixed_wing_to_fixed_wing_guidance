#!/usr/bin/env python3
"""Wait for the two package MAVLink endpoints and validate SysIDs."""

import argparse
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    endpoints = [(14551, 1), (14561, 2)]
    masters = [(mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=252, source_component=193), port, sysid) for port, sysid in endpoints]
    pending = {port for _, port, _ in masters}
    deadline = time.monotonic() + args.timeout
    while pending and time.monotonic() < deadline:
        for master, port, sysid in masters:
            if port not in pending:
                continue
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is None:
                continue
            if msg.get_srcSystem() != sysid:
                raise SystemExit(f'port {port}: expected SysID {sysid}, received {msg.get_srcSystem()}')
            pending.remove(port)
            print(f'port {port}: SysID {sysid} heartbeat ready')
        time.sleep(0.05)
    for master, _, _ in masters:
        master.close()
    if pending:
        raise SystemExit('heartbeat timeout: ' + ','.join(map(str, sorted(pending))))


if __name__ == '__main__':
    main()
