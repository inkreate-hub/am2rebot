#!/usr/bin/env python
"""Why is the bus silent? Distinguish "nobody home" from "wrong bitrate".

    python tools/can_errcheck.py              # can0, motor id 1
    python tools/can_errcheck.py --id 5

Needs no python-can and no root: raw AF_CAN sockets are in the stdlib, and
error-frame reporting is enabled per-socket, not per-interface.

THE DISTINCTION IT MAKES

A silent CAN bus has two completely different causes and they need opposite
fixes. SocketCAN error frames tell them apart, because acknowledgement happens
in the data-link layer, below any motor protocol:

  CAN_ERR_ACK / ACK-slot errors
      We transmitted a well-formed frame and NOT ONE node pulled the ACK slot
      low. Every powered CAN node on the right bitrate acknowledges every
      well-formed frame regardless of what protocol it speaks. So this means
      there is no live node on the wire at all:
        - the arm is not powered
        - CANH/CANL are swapped, or one is not connected
        - a break in the harness, or the wrong connector
      It specifically RULES OUT "the motors are alive but speaking RobStride's
      private protocol" -- that would ACK and simply not reply.

  STUFF / FORM / BIT / CRC errors
      Somebody IS driving the wire, but we cannot agree on the bits. That is a
      bitrate mismatch (or termination/signal integrity), not a dead arm.

  nothing at all
      We never even got on the wire. Interface down, or already BUS-OFF.

See docs/B601_BRINGUP.md stage 1.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

CAN_ERR_FLAG = 0x20000000
CAN_EFF_FLAG = 0x80000000

#: Error classes, from <linux/can/error.h>, carried in the error frame's id.
ERR_CLASS = {
    0x00000001: "TX_TIMEOUT (no transmit completion)",
    0x00000002: "LOSTARB (arbitration lost)",
    0x00000004: "CRTL (controller state change)",
    0x00000008: "PROT (protocol violation)",
    0x00000010: "TRX (transceiver)",
    0x00000020: "ACK  <-- nobody acknowledged",
    0x00000040: "BUSOFF",
    0x00000080: "BUSERROR",
    0x00000100: "RESTARTED",
}

#: data[1], controller status.
ERR_CRTL = {
    0x01: "RX buffer overflow", 0x02: "TX buffer overflow",
    0x04: "RX warning", 0x08: "TX warning",
    0x10: "RX error-passive", 0x20: "TX error-passive",
    0x40: "back to error-active",
}

#: data[2], protocol error type. These are the "wrong bitrate" family.
ERR_PROT = {
    0x01: "single bit error", 0x02: "frame format error",
    0x04: "bit stuffing error", 0x08: "could not send dominant bit",
    0x10: "could not send recessive bit", 0x20: "bus overload",
    0x40: "active error announcement", 0x80: "error on transmit",
}

#: data[3], where in the frame it went wrong.
ERR_LOC = {
    0x03: "start of frame", 0x02: "ID bits 28-21", 0x06: "ID bits 20-18",
    0x04: "substitute RTR", 0x05: "ID extension", 0x07: "ID bits 17-13",
    0x0F: "ID bits 12-5", 0x0E: "ID bits 4-0", 0x0C: "RTR bit",
    0x0D: "reserved bit 1", 0x09: "reserved bit 0", 0x0B: "data length code",
    0x0A: "data section", 0x08: "CRC sequence", 0x18: "CRC delimiter",
    0x19: "ACK slot", 0x1B: "ACK delimiter", 0x1A: "end of frame",
    0x12: "intermission",
}


def decode(can_id: int, data: bytes) -> list[str]:
    out = []
    for bit, text in ERR_CLASS.items():
        if can_id & bit:
            out.append(f"class: {text}")
    if len(data) >= 2 and data[1]:
        for bit, text in ERR_CRTL.items():
            if data[1] & bit:
                out.append(f"  ctrl: {text}")
    if len(data) >= 3 and data[2]:
        for bit, text in ERR_PROT.items():
            if data[2] & bit:
                out.append(f"  prot: {text}")
    if len(data) >= 4 and data[3]:
        out.append(f"   loc: {ERR_LOC.get(data[3], hex(data[3]))}")
    if len(data) >= 7:
        out.append(f"   counters: tx={data[6]} rx={data[7] if len(data) > 7 else '?'}")
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="can0")
    p.add_argument("--id", type=lambda s: int(s, 0), default=1,
                   help="target motor id (1..7)")
    p.add_argument("--host-id", type=lambda s: int(s, 0), default=0xFD)
    p.add_argument("--frames", type=int, default=3)
    p.add_argument("--wait", type=float, default=1.5,
                   help="seconds to listen after each frame")
    args = p.parse_args()

    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    except AttributeError:
        print("This kernel/python has no AF_CAN support.", file=sys.stderr)
        return 2

    # Error frames are opt-in PER SOCKET -- no root, no interface change.
    err_opt = getattr(socket, "CAN_RAW_ERR_FILTER", 2)
    s.setsockopt(socket.SOL_CAN_RAW, err_opt, struct.pack("=I", 0x1FFFFFFF))
    # Do not show us our own frames; we only care about the bus's reaction.
    loopback = getattr(socket, "CAN_RAW_LOOPBACK", 3)
    try:
        s.setsockopt(socket.SOL_CAN_RAW, loopback, struct.pack("=i", 0))
    except OSError:
        pass

    try:
        s.bind((args.port,))
    except OSError as exc:
        print(f"cannot bind {args.port}: {exc}", file=sys.stderr)
        return 2
    s.settimeout(args.wait)

    # RobStride's 29-bit id: [28:24] comm type, [23:8] host id, [7:0] motor id.
    # Type 0 is "get device id" -- the most harmless thing on the bus, and a
    # live motor answers it. But the ACK test does not even need a reply.
    arb = ((0 << 24) | (args.host_id << 8) | (args.id)) | CAN_EFF_FLAG
    print(f"{args.port}: sending {args.frames} x 'get device id' to motor "
          f"{args.id} (host 0x{args.host_id:02X})")
    print(f"extended arbitration id 0x{arb & 0x1FFFFFFF:08X}\n")

    errors: list[str] = []
    replies = 0
    for n in range(args.frames):
        try:
            s.send(struct.pack("=IBBBB8s", arb, 8, 0, 0, 0, b"\x00" * 8))
        except OSError as exc:
            print(f"  send failed: {exc}")
            if "Network is down" in str(exc):
                print("\n  -> the interface is DOWN or BUS-OFF. Recover with:")
                print("       sudo ip link set can0 down && "
                      "sudo ip link set can0 up type can bitrate 1000000")
                return 1
            break

        deadline = time.monotonic() + args.wait
        while time.monotonic() < deadline:
            try:
                frame = s.recv(16)
            except socket.timeout:
                break
            can_id, dlc = struct.unpack("=IB", frame[:5])
            data = frame[8:8 + dlc]
            if can_id & CAN_ERR_FLAG:
                for line in decode(can_id, frame[8:16]):
                    print(f"  [{n}] {line}")
                    errors.append(line)
            else:
                replies += 1
                print(f"  [{n}] REPLY id=0x{can_id & 0x1FFFFFFF:08X} "
                      f"data={data.hex(' ')}")
        time.sleep(0.05)

    print("\n" + "=" * 58)
    joined = " ".join(errors)
    if replies:
        print(f"VERDICT: {replies} real frame(s) came back. The bus works and "
              f"a motor is answering -- the problem is above the link layer.")
        return 0
    if "ACK" in joined or "ACK slot" in joined:
        print("VERDICT: ACK ERROR -- no node on the wire acknowledged.")
        print("""
Nothing is electrically present and listening. Every powered CAN node on the
correct bitrate ACKs every well-formed frame no matter what protocol it speaks,
so this is NOT a protocol or host-id problem. Check, in this order:

  1. IS THE ARM POWERED? 48 V at the motors. This has never been confirmed.
     Meter the rail at the arm, not at the PSU terminals.
  2. CANH / CANL SWAPPED. The single most common wiring fault, and it produces
     exactly this. Swap them and re-run -- it cannot hurt anything.
  3. Continuity CANH and CANL end to end, and that you are on the arm's CAN
     connector rather than a different one.
  4. Termination: ~60 ohm across CANH-CANL with everything off. 120 ohm means
     only your adapter is terminated; open means neither.""")
        return 1
    if any(k in joined for k in ("stuff", "format", "bit error", "CRC",
                                 "recessive", "dominant")):
        print("VERDICT: FRAMING ERRORS -- somebody IS driving the wire, but we "
              "disagree on the bits.")
        print("""
This is good news: the arm is alive. It is a bitrate or signal-integrity
problem, not a dead bus.

  1. Bitrate. We are at 1 Mbit/s. If the motors are at 500 kbit/s nothing will
     ever decode. Re-run at 500k:
       sudo ip link set can0 down
       sudo ip link set can0 up type can bitrate 500000
  2. Termination and stub length -- 60 ohm total, short drops.""")
        return 1
    if not errors:
        print("VERDICT: silence, and not even an error frame.")
        print("""
We may not have reached the wire at all. Check `ip -s -d link show can0`:
  - state BUS-OFF  -> bounce the link (sudo), the controller has given up
  - TX packets not increasing -> the adapter is not transmitting
Otherwise the controller is already error-passive and has stopped complaining;
bounce the link to reset the counters and run this again.""")
        return 1
    print("VERDICT: errors, but not a clean ACK or framing signature:")
    for e in dict.fromkeys(errors):
        print(f"  {e}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
