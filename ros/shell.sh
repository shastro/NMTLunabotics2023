#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Generates a shell inside the ROS container.

DIR="$(dirname "$0")"

sudo docker build "$DIR" -t lunabotics-2023-ros
sudo docker run \
     --rm \
     -it \
     -v $DIR/catkin_ws:$DIR/catkin_ws \
     -h "$(hostname)-ROS" \
     lunabotics-2023-ros \
     sh -c "
         cd $PWD
         bash
     "
