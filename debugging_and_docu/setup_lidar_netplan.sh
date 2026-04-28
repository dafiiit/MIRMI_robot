#!/usr/bin/env bash
set -euo pipefail

# Persistent network setup for MID360 data path.
# Keeps existing 192.168.0.1/24 and adds 192.168.1.50/24 on enP8p1s0.

if [[ ${EUID} -ne 0 ]]; then
  echo "Run as root: sudo bash debugging_and_docu/setup_lidar_netplan.sh"
  exit 1
fi

NETPLAN_FILE="/etc/netplan/01-netcfg.yaml"
BACKUP_FILE="/etc/netplan/01-netcfg.yaml.bak.$(date +%Y%m%d_%H%M%S)"

if [[ -f "$NETPLAN_FILE" ]]; then
  cp "$NETPLAN_FILE" "$BACKUP_FILE"
  echo "Backup created: $BACKUP_FILE"
fi

cat > "$NETPLAN_FILE" <<'YAML'
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enP8p1s0:
      dhcp4: no
      addresses:
        - 192.168.0.1/24
        - 192.168.1.50/24
    usb0:
      dhcp4: yes
YAML

netplan generate
netplan apply

echo "Applied netplan. Current IPv4 for enP8p1s0:"
ip -4 addr show enP8p1s0

echo "LIDAR reachability test:"
ping -c 3 -W 1 192.168.1.145 || true
