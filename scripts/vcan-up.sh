#!/usr/bin/env bash
#
# Bring up a virtual CAN bus (vcan0 by default) for local HIL development.
# Idempotent: deletes any existing device first, so it is safe to re-run.
#
#   sudo scripts/vcan-up.sh            # vcan0
#   sudo VCAN_IFACE=vcan1 scripts/vcan-up.sh
#
# Requires root (sudo) for `modprobe` and `ip link`.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IFACE="${VCAN_IFACE:-vcan0}"

if [[ "$(id -u)" != "0" ]] && ! command -v sudo >/dev/null 2>&1; then
    echo "error: need root (or sudo) to configure ${IFACE}" >&2
    exit 1
fi

echo "==> loading vcan kernel module"
sudo modprobe vcan || true

echo "==> creating ${IFACE}"
sudo ip link del dev "${IFACE}" 2>/dev/null || true
sudo ip link add dev "${IFACE}" type vcan
sudo ip link set up "${IFACE}"

echo "==> ${IFACE} is up (tear down with scripts/vcan-down.sh)"
