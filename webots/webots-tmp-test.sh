#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Temporary shell script for testing Webots.
sudo docker run \
     --rm \
     -it \
     -e DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -v /dev/dri:/dev/dri:rw \
     cyberbotics/webots \
     webots
