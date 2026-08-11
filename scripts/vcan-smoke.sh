#!/usr/bin/env bash
#
# Bring up a virtual CAN bus (vcan0) and exercise the real SocketCAN path:
#   embrig test <vehicle> scripts/loopback.yaml --interface vcan0
#
# The frame is transmitted on the bus and received back on the same socket,
# proving the socketcan send → receive round trip works outside the simulator.
#
# Requires root (sudo) for `modprobe` and `ip link`. On CI this runs in a
# `vcan-smoke` job; locally it also verifies the vcan module is present.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IFACE="${VCAN_IFACE:-vcan0}"

"${ROOT}/scripts/vcan-up.sh"

echo "==> building with socketcan feature"
cargo build -q --workspace --features socketcan --manifest-path "${ROOT}/Cargo.toml"

echo "==> running loopback test on ${IFACE}"
"${ROOT}/target/debug/embrig" test \
    "${ROOT}/examples/ev-powertrain/vehicle.yaml" \
    "${ROOT}/scripts/loopback.yaml" \
    --interface "${IFACE}"

echo "==> smoke test passed"
