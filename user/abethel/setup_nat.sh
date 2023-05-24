#!/usr/bin/env bash
set -xeuo pipefail
IFS=$'\n\t'

INTERNAL="enp7s0f1"
EXTERNAL="wlp0s20f3"

echo 1 > /proc/sys/net/ipv4/ip_forward
iptables -t nat -A POSTROUTING -o "$EXTERNAL" -j MASQUERADE
iptables -A INPUT -i "$INTERNAL" -j ACCEPT
iptables -A INPUT -i "$EXTERNAL" -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A OUTPUT -j ACCEPT
