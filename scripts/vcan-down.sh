#!/usr/bin/env bash
#
# Tear down a virtual CAN bus created by scripts/vcan-up.sh.
#
#   sudo scripts/vcan-down.sh          # vcan0
#   sudo VCAN_IFACE=vcan1 scripts/vcan-down.sh
set -euo pipefail

IFACE="${VCAN_IFACE:-vcan0}"

if [[ "$(id -u)" != "0" ]] && ! command -v sudo >/dev/null 2>&1; then
    echo "error: need root (or sudo) to remove ${IFACE}" >&2
    exit 1
fi

echo "==> removing ${IFACE}"
sudo ip link del dev "${IFACE}" 2>/dev/null || true

echo "==> ${IFACE} removed"
