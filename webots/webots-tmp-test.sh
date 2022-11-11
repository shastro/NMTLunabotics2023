#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Temporary shell script for testing Webots.
xhost +local:root
sudo docker run \
     --rm \
     -it \
     -e DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -v /dev/dri:/dev/dri:rw \
     -v $PWD/worlds:/webots \
     cyberbotics/webots \
     webots
