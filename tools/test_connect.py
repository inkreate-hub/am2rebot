#!/usr/bin/env python
"""Read-only check that the mixed-firmware bus now handshakes.

Connects the Feetech bus exactly as LeRobot does, reads firmware + present
position, then disconnects. Does NOT enable torque and does NOT write any goal
position, so the arm stays limp throughout.
"""

import logging

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

# am-leader-6dof layout, straight from so_leader._ARM_PROFILES
MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_yaw": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(6, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
}

bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=MOTORS)
bus.connect()  # handshake happens here - this is what used to raise
print("\n--> bus.connect() succeeded, handshake passed\n")

print(f"{'joint':<15}{'id':>4}{'firmware':>10}{'raw ticks':>12}")
print("-" * 41)
fw = bus._read_firmware_version(bus.ids)
for name, motor in MOTORS.items():
    raw = bus.read("Present_Position", name, normalize=False)
    print(f"{name:<15}{motor.id:>4}{fw.get(motor.id, '?'):>10}{raw:>12}")

bus.disconnect()
print("\ndisconnected cleanly; torque was never enabled")
