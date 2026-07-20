#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-slcan}"
IFACE="${2:-can0}"
SERIAL_DEVICE="${3:-/dev/ttyUSB0}"

sudo modprobe can
sudo modprobe can_raw

if [[ "$MODE" == "slcan" ]]; then
  if [[ ! -e "$SERIAL_DEVICE" ]]; then
    echo "Serial CAN device does not exist: $SERIAL_DEVICE" >&2
    exit 1
  fi
  sudo modprobe slcan
  sudo pkill -x slcand 2>/dev/null || true
  # -s8 is 1 Mbit/s, required by the RS03 private CAN protocol.
  sudo slcand -o -c -s8 "$SERIAL_DEVICE" "$IFACE"
elif [[ "$MODE" != "native" ]]; then
  echo "Usage: $0 [slcan|native] [interface] [serial-device]" >&2
  exit 2
fi

sudo ip link set "$IFACE" down 2>/dev/null || true
if [[ "$MODE" == "native" ]]; then
  sudo ip link set "$IFACE" type can bitrate 1000000 restart-ms 100
fi
sudo ip link set "$IFACE" txqueuelen 100
sudo ip link set "$IFACE" up

python3 - <<'PY'
import socket
s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.close()
print("CAN_RAW socket: OK")
PY
ip -details -statistics link show "$IFACE"
echo "SocketCAN interface $IFACE is ready."
