#!/usr/bin/env python
"""Read-only Feetech bus probe: ping IDs 1..12 and report which answer.

PING only - no writes, no torque enable, no position commands. Safe to run on an
arm that is powered or unpowered, and harmless if the port turns out to be a CAN
bridge rather than a servo bus (it simply won't answer).
"""

import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUDS = [1_000_000, 500_000, 115_200]

for baud in BAUDS:
    ph = PortHandler(PORT)
    if not ph.openPort():
        print(f"could not open {PORT}")
        raise SystemExit(1)
    ph.setBaudRate(baud)
    packet = PacketHandler(0)  # protocol 0 = STS/SMS series

    found = []
    for sid in range(1, 13):
        model, comm, err = packet.ping(ph, sid)
        if comm == COMM_SUCCESS and err == 0:
            found.append((sid, model))
    ph.closePort()

    print(f"baud {baud:>9}: " + (
        ", ".join(f"id{sid}=model{model}" for sid, model in found) if found else "no response"
    ))
    if found:
        print(f"\n--> {len(found)} servos on {PORT} at {baud} baud")
        break
else:
    print(f"\n--> nothing answered the Feetech protocol on {PORT}")
    print("    Likely the Damiao CAN bridge (or the arm is unpowered).")
