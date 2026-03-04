#!/bin/bash

# Check if script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "Error: This script must be run as sudo or as root."
  echo "Usage: sudo $0"
  exit 1
fi

set -e

adapter=""
for dev in /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  if udevadm info -a -n "$dev" | grep -qi 'canable'; then
    adapter="$dev"
    echo "Found CAN adapter at $adapter"
    break
  fi
done

if [ -z "$adapter" ]; then
  echo "No CAN adapter found" >&2
  exit 1
fi

slcand -o -c -s6 "$adapter"
ip link set can0 up