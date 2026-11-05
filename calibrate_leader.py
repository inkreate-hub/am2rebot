#!/usr/bin/env python
"""Calibrate the AM-ARM200 leader with the mixed-firmware guard bypassed.

Why this exists
---------------
`lerobot-calibrate` refuses to connect when the servos report different firmware
versions. This arm's servos came from several donor arms, so they are split:

    ids 1,5,6,7 -> 3.9      ids 2,3,4 -> 3.10

That check (`FeetechMotorsBus._assert_same_firmware`) is a blanket uniformity
guard, not a functional dependency:

  * the firmware version is read in exactly one place -- the assertion itself --
    and never used to pick a control table, register address or encoding;
  * the only firmware-conditional register in `tables.py` is `Hts`, valid for
    >= 2.54, and both 3.9 and 3.10 clear that;
  * verified on this arm: Homing_Offset, Min_Position_Limit, Max_Position_Limit
    and Present_Position all read sane values on both firmware groups, and the
    ID/baudrate writes during assembly worked identically across both.

So we skip the assertion and keep every other handshake step. The honest fix is
to flash all seven to one version with Feetech's tool; this is the pragmatic one.

Everything else mirrors `lerobot.scripts.lerobot_calibrate` exactly.

Usage:
    python calibrate_leader.py [--port /dev/ttyACM0] [--id am_leader]
"""

import argparse
import logging
import sys

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.utils.utils import init_logging


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="am_leader")
    ap.add_argument("--arm-profile", default="am-leader-6dof")
    args = ap.parse_args()

    init_logging()

    original = FeetechMotorsBus._assert_same_firmware

    def _skip_firmware_check(self):
        versions = self._read_firmware_version(self.ids)
        if len(set(versions.values())) != 1:
            logging.warning(
                "BYPASSING mixed-firmware guard. Versions: %s", versions
            )
        else:
            original(self)

    FeetechMotorsBus._assert_same_firmware = _skip_firmware_check

    cfg = SOLeaderTeleopConfig(
        port=args.port, id=args.id, arm_profile=args.arm_profile
    )
    device = SOLeader(cfg)
    device.connect(calibrate=False)
    try:
        device.calibrate()
    finally:
        device.disconnect()
        FeetechMotorsBus._assert_same_firmware = original
    return 0


if __name__ == "__main__":
    sys.exit(main())
