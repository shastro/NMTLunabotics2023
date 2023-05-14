#!/bin/bash

sudo ip link set up dev wlp0s20f3
sudo ip addr add 192.168.4.1/24 dev wlp0s20f3
sudo sysctl net.ipv4.ip_forward=1

sudo systemctl stop NetworkManager
sleep 5
sudo dhcpd
sleep 5
sudo systemctl restart hostapd

