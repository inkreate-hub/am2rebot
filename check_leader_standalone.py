#!/usr/bin/env python3
"""Assembly check for the AM-ARM200 leader — 7 Feetech STS3215 servos.

Standalone: needs only pyserial, so it runs without the lerobot install.
Speaks the Feetech SCS/STS serial protocol directly.

READ-ONLY. Only ever sends PING (0x01) and READ (0x02). No WRITE instruction is
implemented, so it cannot change an ID, a baudrate, or enable torque.

Usage:
    python3 check_leader_standalone.py [--port /dev/ttyACM0] [--watch]

--watch re-reads positions ~5x/s: wiggle one joint at a time and check that the
row which moves is the row you expect.
"""

import argparse
import sys
import termios
import time

import serial

# Mirrors _ARM_PROFILES["am-leader-6dof"] in
# lerobot_alohamini/src/lerobot/teleoperators/so_leader/so_leader.py
EXPECTED = [
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_yaw", 5),
    ("wrist_roll", 6),
    ("gripper", 7),
]
NAME_BY_ID = {mid: n for n, mid in EXPECTED}
EXPECTED_IDS = set(NAME_BY_ID)

# STS/SCS control table
ADDR_MODEL = 3  # 2 bytes
ADDR_PRESENT_POSITION = 56  # 2 bytes

INST_PING = 0x01
INST_READ = 0x02

# lerobot's Feetech default is 1 Mbaud; the rest are the factory options a
# mis-flashed servo could be sitting on.
BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400, 19_200]

SCAN_IDS = range(0, 21)  # 1-7 expected; 8-20 + 0 catch unprogrammed strays


def _checksum(body):
    return (~sum(body)) & 0xFF


def _packet(motor_id, inst, params=()):
    body = [motor_id, len(params) + 2, inst, *params]
    return bytes([0xFF, 0xFF, *body, _checksum(body)])


def _read_status(ser, expect_params=0):
    """Parse one status packet. Returns (id, error, params) or None on timeout.

    Reads the whole frame in ONE bulk call rather than byte-at-a-time. On a
    cdc_acm bridge (e.g. the Waveshare bus servo adapter) each single-byte read
    costs a full USB poll interval, so hunting the header one byte at a time
    overruns any sane deadline and every reply looks like silence.
    """
    want = 2 + 2 + expect_params + 1  # header, id+len, err+params, checksum
    buf = ser.read(want)
    if len(buf) < want:
        # Top up once: the frame can straddle two USB transfers.
        buf += ser.read(want - len(buf))

    idx = buf.find(b"\xff\xff")
    if idx < 0 or len(buf) - idx < 5:
        return None
    frame = buf[idx + 2 :]
    motor_id, length = frame[0], frame[1]
    payload = frame[2 : 2 + (length - 1)]
    if not payload:
        return None
    return motor_id, payload[0], payload[1:]


def ping(ser, motor_id):
    ser.reset_input_buffer()
    ser.write(_packet(motor_id, INST_PING))
    ser.flush()
    r = _read_status(ser, expect_params=0)
    return r is not None and r[0] == motor_id


def read_u16(ser, motor_id, addr):
    ser.reset_input_buffer()
    ser.write(_packet(motor_id, INST_READ, (addr, 2)))
    ser.flush()
    r = _read_status(ser, expect_params=2)
    if r is None or r[0] != motor_id or len(r[2]) < 2:
        return None
    return r[2][0] | (r[2][1] << 8)  # Feetech STS is little-endian


def scan(port):
    print(f"== Scanning {port} ==")
    hits = {}
    opened_any = False
    for baud in BAUDRATES:
        try:
            with serial.Serial(port, baud, timeout=0.02, write_timeout=0.2) as ser:
                opened_any = True
                found = [i for i in SCAN_IDS if ping(ser, i)]
        except termios.error as e:
            # The CH9102 does not accept every rate in BAUDRATES and raises
            # termios EINVAL rather than a SerialException. Not a fault -- skip
            # the rate and keep going, otherwise one unsupported speed aborts a
            # scan that has already found every servo.
            print(f"  baudrate {baud:>9}: unsupported by this adapter ({e})")
            continue
        except serial.SerialException as e:
            print(f"  baudrate {baud:>9}: {e}")
            continue
        if found:
            hits[baud] = found
            print(f"  baudrate {baud:>9}: ids {found}")
    if not opened_any:
        print(f"\nFAIL: could not open {port} at any baudrate.")
        print("  Another process may hold it -- check with: fuser -v " + port)
        return None
    if not hits:
        print("\nFAIL: port opens but nothing answered at any baudrate.")
        print("  Check servo power, the data chain seating, and the port.")
        return None
    return hits


def verdict(hits):
    ok = True
    if len(hits) > 1:
        print(
            "\nWARN: servos answered at more than one baudrate. The bus must be "
            "uniform before teleop."
        )
        ok = False
    for baud, ids in sorted(hits.items()):
        ids = set(ids)
        missing = sorted(EXPECTED_IDS - ids)
        extra = sorted(ids - EXPECTED_IDS)
        print(f"\n== Verdict @ {baud} baud ==")
        print(f"  {len(ids & EXPECTED_IDS)}/7 expected ids present")
        if missing:
            print("  MISSING: " + ", ".join(f"id {m} ({NAME_BY_ID[m]})" for m in missing))
            ok = False
        if extra:
            print(f"  UNEXPECTED ids {extra} — unprogrammed or duplicate servo")
            ok = False
        if not missing and not extra:
            print("  OK: ids 1-7 all present, no strays.")
    return ok


def positions(port, baud, watch):
    with serial.Serial(port, baud, timeout=0.02, write_timeout=0.2) as ser:
        first = True
        while True:
            rows = []
            for name, mid in EXPECTED:
                model = read_u16(ser, mid, ADDR_MODEL) if first else None
                pos = read_u16(ser, mid, ADDR_PRESENT_POSITION)
                if pos is None:
                    rows.append(f"  id {mid}  {name:<14}    --   no reply")
                else:
                    deg = pos * 360.0 / 4096.0
                    model_s = f"  model {model}" if model else ""
                    rows.append(f"  id {mid}  {name:<14} {pos:>5}  {deg:>6.1f}deg{model_s}")
            if watch:
                print("\033[2J\033[H", end="")
            print(f"\n== Present_Position @ {baud} baud (raw ticks, 0-4095) ==")
            if watch:
                print("Wiggle ONE joint at a time. Ctrl-C to stop.")
            print("\n".join(rows))
            first = False
            if not watch:
                return
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--watch", action="store_true")
    args = ap.parse_args()

    hits = scan(args.port)
    if hits is None:
        return 1
    verdict(hits)
    baud = max(hits, key=lambda b: len(hits[b]))
    try:
        positions(args.port, baud, args.watch)
    except KeyboardInterrupt:
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
