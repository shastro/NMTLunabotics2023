#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.sh" --
[ -e "/ros/catkin_ws/devel/setup.sh" ] && source "/ros/catkin_ws/devel/setup.sh" --
exec "$@"
