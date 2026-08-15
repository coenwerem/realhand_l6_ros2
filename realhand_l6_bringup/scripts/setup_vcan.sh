#!/usr/bin/env bash
# Bring up a virtual CAN interface for hardware free runs of the driver.
# Reversible kernel interface, nothing touches a real bus.
set -e
IFACE="${1:-vcan0}"
sudo modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  sudo ip link add dev "$IFACE" type vcan
fi
sudo ip link set up "$IFACE"
ip -br link show "$IFACE"
