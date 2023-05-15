#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
[ -e "/ros/catkin_ws/devel/setup.bash" ] && source "/ros/catkin_ws/devel/setup.bash" --
exec "$@"
