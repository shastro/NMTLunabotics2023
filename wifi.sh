#!/bin/bash
set -ex

# sudo su
#[ "$USER" != root ] && exec sudo "$0"
systemd-resolve --set-dns 8.8.8.8 --interface $1
route del -net 0.0.0.0 gw 192.168.1.1
route add -net 0.0.0.0 gw 192.168.1.39
