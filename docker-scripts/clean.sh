#!/bin/bash

read -p "This script cleans docker containers on the jetson. Are you on the jetson? (y/n): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1
echo "You damn well better be on the jetson. If you're lying to me I'll send your mom all your porn."

# clean STUPID src code archives - 4GB
sudo rm /usr/src/public_sources.tbz2 /usr/src/rootfs_src.tbz2

[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"
find /var/lib/docker/containers/ -type f -name "*.log" -delete
> /var/log/uvcdynctrl-udev.log
docker system prune
systemctl restart docker

