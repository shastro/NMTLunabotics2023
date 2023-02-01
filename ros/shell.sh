#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Generates a shell inside the ROS container.

DIR="$(realpath "$(dirname "$0")")"

sudo docker build "$DIR" -t lunabotics-2023-ros
sudo docker run \
     --rm \
     -it \
     -v $DIR/catkin_ws:$DIR/catkin_ws \
     -h "$(hostname)-ROS" \
     lunabotics-2023-ros \
     sh -c "
         cd $PWD
         export HOME="$HOME"
         export PS1='\u@\h:\w\$ '
         [ -e $DIR/catkin_ws/devel/setup.sh ] && . $DIR/catkin_ws/devel/setup.sh
         bash
     "

# ROS might create files owned by root here; it'll cause all kinds of
# problems with git.
sudo chown -R "$(id -u):$(id -g)" "$DIR"
