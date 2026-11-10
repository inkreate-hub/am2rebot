#!/usr/bin/env bash
# Bring a SocketCAN link up for the B601-RS: classic CAN, 1 Mbit/s.
#
# NOT CAN FD. Seeed document the arm as classic CAN at 1 Mbit/s, and
# lerobot's RobstrideMotorsBus takes use_can_fd=False at that bitrate.
# motorbridge only exposes from_socketcanfd, which is why it is not the
# transport we use.
#
#   tools/can_up.sh            # can0
#   tools/can_up.sh can1
set -euo pipefail

IFACE="${1:-can0}"
BITRATE=1000000

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "error: no interface '$IFACE'." >&2
  echo "  USB-CAN adapter plugged in? Check:  ip -brief link show type can" >&2
  echo "  Driver loaded?                      lsmod | grep -E 'gs_usb|slcan|peak'" >&2
  exit 1
fi

# Down first: re-running with the link already up fails with EBUSY, and the
# bitrate cannot be changed while the interface is live.
sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" up type can bitrate "$BITRATE"

echo "=== $IFACE up at $BITRATE bit/s (classic CAN)"
ip -details link show "$IFACE"
echo
echo "Watch raw traffic with:   candump -tz $IFACE"
echo "Probe the seven motors:   ./lerobot_alohamini/.venv/bin/python tools/can_check.py --port $IFACE"
