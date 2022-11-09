#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Generates a shell inside the ROS container.

sudo docker build . -t lunabotics-2023-ros
sudo docker run \
     --rm \
     -it \
     -v $PWD/catkin_ws:/catkin_ws \
     lunabotics-2023-ros
