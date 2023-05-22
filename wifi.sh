#!/bin/bash
set -eux

# sudo su
[ "$USER" != root ] && exec sudo "$0"
# echo "nameserver 8.8.8.8" > /etc/resolv.conf
systemd-resolve --set-dns 8.8.8.8 --interface $1
route del -net 0.0.0.0 gw 192.168.1.1
route add -net 0.0.0.0 gw 192.168.1.39
