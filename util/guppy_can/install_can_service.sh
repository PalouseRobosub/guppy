#!/bin/bash

# Check if script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "Error: This script must be run as sudo or as root."
  echo "Usage: sudo $0"
  exit 1
fi

set -e

cp setup_can.sh /etc/setup_can.sh

cp setup_can.service /etc/systemd/system/guppy_can.service

systemctl enable --now guppy_can.service