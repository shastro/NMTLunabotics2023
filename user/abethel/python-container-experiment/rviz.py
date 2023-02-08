#!/usr/bin/env python3

"""Runs RViz and connects it to ROS."""

import containers

# use the `pkg` argument here to require that the container have rviz
# installed
containers.roscmd('rviz', pkg='ros-noetic-rviz').run()
