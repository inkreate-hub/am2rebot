#!/usr/bin/env bash
# Identify the attached USB-CAN adapter and say which driver path it needs.
#
#   tools/can_identify.sh
#
# Written because the adapter on hand is a "USB-TO-CAN UTC.T01", which appears
# in no public SocketCAN driver list. The question it answers is binary and
# decides everything downstream:
#
#   native SocketCAN (gs_usb / peak_usb / ...) -> `can0` appears, motorbridge
#                                                 and python-can both just work
#   slcan (serial line CAN)                    -> no `can0` until `slcand`
#                                                 turns a /dev/tty* into one
#   proprietary (CH340/CH341 + vendor frames)  -> there is NO `can0`, ever.
#                                                 motorbridge cannot drive it.
#                                                 Different adapter required.
#
# The last case is the one worth catching early: a CH340-based "USB-CAN"
# enumerates as a plain serial port and looks superficially fine, but it speaks
# a vendor packet format that SocketCAN knows nothing about. Everything above it
# then fails in a way that looks like a dead bus.

set -uo pipefail

say() { printf '%s\n' "$*"; }
hr() { printf -- '---------------------------------------------------------\n'; }

say "=== USB devices (external only) ==="
# Filter the root hubs and the known-internal Dell/Latitude peripherals so the
# adapter is not lost in the noise.
mapfile -t external < <(lsusb | grep -vEi \
  'root hub|Synaptics|IMC Networks|Intel Corp. AX|Integrated Camera|Fingerprint')

if [ "${#external[@]}" -eq 0 ]; then
  say "(none)"
  hr
  say "No external USB device is attached."
  say
  say "The adapter is not reaching this machine. Check, in this order:"
  say "  1. Is the USB cable a DATA cable? A charge-only cable gives exactly"
  say "     this result - silence, no dmesg line, no device node."
  say "  2. Try the other USB port, and a different cable."
  say "  3. Does the adapter show a power LED when plugged into the laptop?"
  say "  4. Watch the kernel live while you plug it in:"
  say "         sudo dmesg -w"
  say "     A healthy plug-in prints 'new full-speed USB device' within ~1 s."
  say "     Absolute silence means the host never saw an electrical connection."
  exit 1
fi

printf '%s\n' "${external[@]}"
hr

# --- classify ------------------------------------------------------------
# VID:PID -> driver path. Only adapters whose identity is actually established
# are claimed here; anything else falls through to the honest "unknown" branch
# rather than being guessed at.
verdict=""
detail=""
for line in "${external[@]}"; do
  id=$(printf '%s' "$line" | grep -oE '[0-9a-f]{4}:[0-9a-f]{4}' | head -1)
  case "$id" in
    1d50:606f) verdict="native";      detail="candleLight / CANable (gs_usb)";;
    0c72:000c|0c72:000d|0c72:0012|0c72:0013)
               verdict="native";      detail="PEAK PCAN-USB (peak_usb)";;
    0c72:0011|0c72:0014)
               verdict="native";      detail="PEAK PCAN-USB FD (peak_usb)";;
    1cbe:0001) verdict="native";      detail="TI / Rusoku TouCAN";;
    0483:5740) verdict="slcan";       detail="STM32 virtual COM - CANable with slcan firmware";;
    16d0:117e) verdict="slcan";       detail="USBtin / slcan";;
    1a86:7523) verdict="proprietary"; detail="CH340 serial bridge";;
    1a86:55d4|1a86:55d3)
               verdict="proprietary"; detail="CH9102 serial bridge";;
    1a86:5512) verdict="proprietary"; detail="CH341 parallel/serial bridge";;
    04d8:0053) verdict="outoftree";   detail="CANalyst-II (needs CANalystII-SocketCAN module)";;
  esac
  [ -n "$verdict" ] && { say "Matched $id -> $detail"; break; }
done

say "=== device nodes ==="
ls -la /dev/ttyACM* /dev/ttyUSB* /dev/pcan* 2>/dev/null || say "(no serial/pcan nodes)"
say
say "=== socketcan interfaces ==="
ip -br link show type can 2>/dev/null | grep -q . \
  && ip -details link show type can \
  || say "(no can interface yet)"
hr

case "$verdict" in
  native)
    say "VERDICT: native SocketCAN adapter ($detail)."
    say "Bring the link up at the B601's 1 Mbit/s, classic CAN (not FD):"
    say "    tools/can_up.sh can0"
    say "Then:  tools/mb_bringup.py --scan"
    ;;
  slcan)
    say "VERDICT: slcan adapter ($detail). It needs a translator daemon;"
    say "there is no can0 until you start one. 's8' = 1 Mbit/s:"
    say "    sudo slcand -o -c -s8 /dev/ttyACM0 can0"
    say "    sudo ip link set can0 up"
    say "Then:  tools/mb_bringup.py --scan"
    ;;
  proprietary)
    say "VERDICT: *** this is a serial bridge, not a SocketCAN adapter. ***"
    say
    say "$detail speaks a vendor-specific packet format. SocketCAN will never"
    say "give you a can0 for it, and motorbridge has no transport for it."
    say "A CH340-based 'USB-CAN' is the classic trap here: the serial port"
    say "appears, so the adapter looks alive, but no CAN tool can use it."
    say
    say "Options, best first:"
    say "  1. Use a supported adapter. Seeed's own wiki specifies PCAN-USB"
    say "     (DIP switch on 120R); a CANable with candleLight firmware is the"
    say "     cheap equivalent and is natively supported."
    say "  2. Check whether the vendor ships Linux SocketCAN firmware for it."
    say "  3. Last resort: the vendor's own Windows/Python tool, which will not"
    say "     integrate with motorbridge or lerobot."
    ;;
  outoftree)
    say "VERDICT: $detail - works on Linux, but needs an out-of-tree kernel"
    say "module built and installed first. Not plug-and-play."
    ;;
  *)
    say "VERDICT: UNKNOWN adapter - not in this script's table."
    say
    say "Do not guess. Determine it from evidence, in this order:"
    say "  1. Record the VID:PID printed above."
    say "  2. Plug/unplug while watching:  sudo dmesg -w"
    say "     - 'cdc_acm ... ttyACM0' or 'ch341 ... ttyUSB0' => SERIAL bridge."
    say "       Then it is either slcan or proprietary; try the slcand line"
    say "       below, and if the bus stays silent it is proprietary."
    say "           sudo slcand -o -c -s8 /dev/ttyACM0 can0"
    say "           sudo ip link set can0 up"
    say "     - 'gs_usb' / 'peak_usb' / a new 'can0' link => NATIVE. Use"
    say "           tools/can_up.sh can0"
    say "  3. 'lsusb -v -d <VID:PID> | head -40' names the vendor outright."
    ;;
esac
