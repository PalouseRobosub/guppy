#!/bin/bash

# Check if script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "Error: This script must be run as sudo or as root."
  echo "Usage: sudo $0"
  exit 1
fi

set -e

apt update
apt install -y net-tools openssh-server dnsmasq

cp 99-guppy-netplan.yaml /etc/netplan/99-guppy-netplan.yaml
netplan apply

cp dnsmasq.conf /etc/dnsmasq.conf
systemctl enable --now dnsmasq
systemctl restart dnsmasq