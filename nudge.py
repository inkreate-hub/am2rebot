#!/usr/bin/env python
"""Move one joint a small, monitored amount. Returns it where it started.

A deliberately minimal driven move, for sanity-checking that a joint can be
driven at all and for watching what the servo and the supply do while it
happens. Unlike measure_steps.py this takes no measurements and draws no
conclusions -- it just moves, logs, and comes back.

Logged at every increment: position, Present_Load, Present_Voltage,
Present_Current and the Status error flags. Voltage is the interesting one --
if it sags as the motor pulls, the supply is current-limiting, which looks
exactly like a stiff joint from the servo's point of view.

Aborts immediately on any non-zero Status, and always disables torque and
restores the motion profile on the way out.

THIS WRITES Goal_Position, Torque_Enable, Goal_Velocity and Acceleration.
All four are RAM registers -- no EEPROM, no calibration touched.

    ./lerobot_alohamini/.venv/bin/python nudge.py --joint shoulder_pan
    ./lerobot_alohamini/.venv/bin/python nudge.py --joint shoulder_pan --degrees 20
"""

import argparse
import sys
import time

from measure_steps import MIN_DRIVE_VOLTS, TRAVEL, settle

# Status register bits, Feetech STS. Reported by name so an abort says why.
STATUS_BITS = {
    0x01: "input voltage",
    0x02: "angle limit",
    0x04: "overheating",
    0x08: "range",
    0x10: "checksum",
    0x20: "overload",
    0x40: "instruction",
}


def decode_status(status: int) -> str:
    return ", ".join(v for k, v in STATUS_BITS.items() if status & k) or "ok"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--joint", default="shoulder_pan", choices=list(TRAVEL))
    p.add_argument("--degrees", type=float, default=10.0, help="total move, then back")
    p.add_argument("--increment", type=float, default=2.0)
    p.add_argument("--velocity", type=int, default=300)
    p.add_argument("--accel", type=int, default=20)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--id", default="am_leader")
    args = p.parse_args()

    name = args.joint

    from lerobot.teleoperators.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig

    leader = SOLeader(SOLeaderTeleopConfig(port=args.port, id=args.id,
                                           arm_profile="am-leader-6dof"))
    leader.connect(calibrate=False)
    bus = leader.bus

    def probe() -> tuple[float, int, float, int, int]:
        return (
            leader.get_action()[f"{name}.pos"],
            bus.read("Present_Load", name, normalize=False),
            bus.read("Present_Voltage", name, normalize=False) / 10.0,
            bus.read("Present_Current", name, normalize=False),
            bus.read("Status", name, normalize=False),
        )

    start, _, volts, _, status0 = probe()
    lo, hi = TRAVEL[name]

    if volts < MIN_DRIVE_VOLTS:
        print(f"Refusing: {volts:.1f} V is below {MIN_DRIVE_VOLTS:g} V. "
              f"No external servo supply?", file=sys.stderr)
        leader.disconnect()
        return 1
    if status0:
        print(f"Refusing: {name} already reports '{decode_status(status0)}'. "
              f"Power-cycle the arm to clear it.", file=sys.stderr)
        leader.disconnect()
        return 1
    if not (lo < start + args.degrees < hi):
        print(f"Refusing: {start:.1f} + {args.degrees:g} leaves travel "
              f"({lo:.0f} .. {hi:.0f}). Use a smaller --degrees or the other sign.",
              file=sys.stderr)
        leader.disconnect()
        return 1

    print(f"\n{name}: {start:.2f} -> {start + args.degrees:.2f} deg and back, "
          f"{args.increment:g} deg at a time")
    print(f"supply {volts:.1f} V, travel {lo:.0f} .. {hi:.0f}\n")

    prev_vel = prev_acc = None
    rc = 0
    try:
        prev_vel = bus.read("Goal_Velocity", name, normalize=False)
        prev_acc = bus.read("Acceleration", name, normalize=False)
        bus.write("Goal_Velocity", name, args.velocity, normalize=False)
        bus.write("Acceleration", name, args.accel, normalize=False)
        bus.write("Goal_Position", name, start)  # park before energising
        bus.enable_torque(name)
        time.sleep(0.2)

        print(f"{'target':>9}{'actual':>9}{'load':>7}{'volts':>7}{'curr':>7}  status")
        print("-" * 54)

        # Out, then back. Same waypoints in reverse, so the return leg also
        # exercises the joint in the opposite direction.
        n = max(1, int(abs(args.degrees) / args.increment))
        sign = 1.0 if args.degrees >= 0 else -1.0
        out = [start + sign * args.increment * (i + 1) for i in range(n)]
        for target in out + list(reversed(out[:-1])) + [start]:
            bus.write("Goal_Position", name, target)
            settle(leader, name)
            pos, load, v, curr, st = probe()
            print(f"{target:>9.2f}{pos:>9.2f}{load:>7}{v:>7.1f}{curr:>7}  "
                  f"{decode_status(st)}")
            if st:
                print(f"\nABORT: {name} reports '{decode_status(st)}'. Stopping here.",
                      file=sys.stderr)
                rc = 1
                break

        end, _, v_end, _, _ = probe()
        print(f"\nback at {end:.2f} deg (started {start:.2f}, "
              f"delta {end - start:+.2f}), supply {v_end:.1f} V")

    except KeyboardInterrupt:
        print("\ninterrupted")
        rc = 130
    except Exception as exc:  # noqa: BLE001 - report, then still de-energise
        print(f"\nerror: {exc}", file=sys.stderr)
        rc = 1
    finally:
        try:
            bus.disable_torque(name)
            print("torque disabled")
            if prev_vel is not None:
                bus.write("Goal_Velocity", name, prev_vel, normalize=False)
                bus.write("Acceleration", name, prev_acc, normalize=False)
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: could not de-energise {name}: {exc}", file=sys.stderr)
            print("Cut the arm's power before handling it.", file=sys.stderr)
        leader.disconnect()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
