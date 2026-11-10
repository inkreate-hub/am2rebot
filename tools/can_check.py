#!/usr/bin/env python
"""Stage 1: does anything answer on the CAN bus? Read-only, no torque, no lerobot.

Deliberately built on python-can alone. If `RobstrideMotorsBus` fails to
handshake there are two very different causes -- a wiring/bitrate problem, or
the motors being in the wrong *communication* mode -- and a tool that goes
through the driver cannot tell them apart.

WHAT IT SENDS

One `CLEAR_FAULT` poll per id, which is what lerobot's own handshake uses to
ask "are you there and are you healthy":

    standard 11-bit frame, arbitration_id = motor id, data = FF FF FF FF FF FF FF FB

That framing is the **Damiao-style** one. Robstride motors only speak it when
their comm mode is set to MIT; `RobstrideMotorsBus`'s own docstring says "the
motors need to be switched to MIT control mode to be compatible with this
implementation", and `tables.py` carries a `CommMode` enum of
`PrivateProtocole / CANopen / MIT`. In RobStride's native private protocol the
motors use **29-bit extended** ids instead.

So this tool listens for both, and the two silences mean different things:

  - nothing at all          -> wiring, termination, bitrate, or no 48 V
  - extended frames only    -> motors are in the private protocol; switch them
                               to MIT comm mode (RobStride MotorStudio) or use
                               Seeed's own `rebot` package instead
  - standard frames, 7 ids  -> gate 1 passed

Also prints `data[0]` of every reply, which settles the feedback-id question:
lerobot decodes into its state cache via `_id_to_name[data[0]]`, built from
`recv_id if recv_id is not None else id`. Seeed document plain ids 1..7, while
lerobot's *Damiao* B601 config pairs each joint with a feedback id 0x11..0x17.
Guess wrong and every frame is dropped as "Unmapped CAN frame".

    tools/can_up.sh can0
    ./lerobot_alohamini/.venv/bin/python tools/can_check.py --port can0
    ./lerobot_alohamini/.venv/bin/python tools/can_check.py --port can0 --listen 10
"""

from __future__ import annotations

import argparse
import sys
import time

CLEAR_FAULT = 0xFB

#: Joint order on the B601, CAN ids 1..7. From Seeed's B601-RS wiki.
JOINTS = (
    (1, "shoulder_pan", "RS06"),
    (2, "shoulder_lift", "RS06"),
    (3, "elbow_flex", "RS06"),
    (4, "wrist_flex", "RS00"),
    (5, "wrist_yaw", "RS00"),
    (6, "wrist_roll", "RS00"),
    (7, "gripper", "RS00"),
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--interface",
        default="socketcan",
        help="python-can interface type; 'slcan' for a serial dongle",
    )
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument(
        "--listen",
        type=float,
        default=0.0,
        help="seconds to listen passively BEFORE polling; use this to catch "
        "motors that broadcast on their own, which also reveals extended-id "
        "private-protocol traffic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.2,
        help="seconds to wait for each motor's reply",
    )
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        print(
            "python-can is not installed. Install it into the project venv:\n"
            "  /home/inko/.local/uvenv/bin/uv pip install \\\n"
            "      --python ./lerobot_alohamini/.venv/bin/python python-can",
            file=sys.stderr,
        )
        return 1

    try:
        bus = can.interface.Bus(
            channel=args.port, interface=args.interface, bitrate=args.bitrate
        )
    except Exception as exc:  # noqa: BLE001 - surface the real cause verbatim
        print(f"could not open {args.port} ({args.interface}): {exc}", file=sys.stderr)
        print("\n  tools/can_up.sh " + args.port, file=sys.stderr)
        return 1

    print(f"=== {args.port} open at {args.bitrate} bit/s, classic CAN")
    print("read-only: this tool never enables torque\n")

    extended_seen: list[can.Message] = []

    if args.listen > 0:
        print(f"listening passively for {args.listen:.1f} s ...")
        deadline = time.monotonic() + args.listen
        seen = 0
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            seen += 1
            if msg.is_extended_id:
                extended_seen.append(msg)
            if seen <= 20:
                kind = "ext" if msg.is_extended_id else "std"
                print(f"  {kind} 0x{msg.arbitration_id:08X}  {bytes(msg.data).hex()}")
        print(f"  {seen} frame(s) unprompted\n")

    print(f"{'id':>3}  {'joint':<15}{'model':<7}{'reply':<8}"
          f"{'arb':>10}{'data[0]':>9}  payload")
    print("-" * 74)

    answered: list[int] = []
    for motor_id, joint, model in JOINTS:
        _drain(bus)
        frame = can.Message(
            arbitration_id=motor_id,
            data=[0xFF] * 7 + [CLEAR_FAULT],
            is_extended_id=False,
        )
        try:
            bus.send(frame)
        except Exception as exc:  # noqa: BLE001
            print(f"{motor_id:>3}  {joint:<15}{model:<7}send failed: {exc}")
            continue

        reply = _await_reply(bus, args.timeout, extended_seen)
        if reply is None:
            print(f"{motor_id:>3}  {joint:<15}{model:<7}{'SILENT':<8}")
            continue

        answered.append(motor_id)
        d0 = reply.data[0] if len(reply.data) else -1
        print(
            f"{motor_id:>3}  {joint:<15}{model:<7}{'ok':<8}"
            f"{reply.arbitration_id:>10X}{d0:>9}  {bytes(reply.data).hex()}"
        )

    bus.shutdown()
    return _verdict(answered, extended_seen)


def _drain(bus) -> None:
    """Discard anything already queued, so a reply cannot be mistaken."""
    while bus.recv(timeout=0.0) is not None:
        pass


def _await_reply(bus, timeout: float, extended_seen: list) -> object | None:
    """Wait for one standard-id reply, noting any extended frames in passing."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None:
            return None
        if msg.is_extended_id:
            extended_seen.append(msg)
            continue
        return msg
    return None


def _verdict(answered: list[int], extended_seen: list) -> int:
    expected = [m for m, _, _ in JOINTS]
    missing = [m for m in expected if m not in answered]

    print()
    if not missing:
        print(f"GATE 1 PASSED: all {len(expected)} motors answered.")
        print(
            "Note which data[0] the replies carry -- if it is the device id "
            "(1..7), leave recv_id=None in the bus config; if it is 0x10+id, "
            "set recv_id or every frame is dropped as unmapped."
        )
        return 0

    print(f"GATE 1 FAILED: {len(answered)}/{len(expected)} answered, "
          f"missing ids {missing}")

    if extended_seen:
        print(
            f"\n{len(extended_seen)} EXTENDED-id frame(s) were seen. That is "
            "RobStride's native private protocol, not the MIT-mode framing "
            "lerobot's RobstrideMotorsBus speaks. The motors are alive and the "
            "bus is wired correctly -- they are in the wrong communication "
            "mode.\n"
            "  -> switch them to MIT comm mode with RobStride MotorStudio, or\n"
            "  -> use Seeed's own `rebot` package, which is written for this arm"
        )
    elif not answered:
        print(
            "\nTotal silence. In order of likelihood: no 48 V on the arm; "
            "CAN H/L swapped; missing 120 R termination; wrong bitrate "
            "(must be 1 Mbit/s classic, not FD); adapter not on the same "
            "ground as the arm."
        )
    else:
        print(
            "\nA partial bus is NOT safe to drive: a joint that does not "
            "answer is a joint that is not holding. Find it before stage 2."
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
