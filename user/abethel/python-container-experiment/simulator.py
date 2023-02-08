#!/usr/bin/env python3

"""Runs a Webots simulation as the ROS master."""

from containers import Compose, Container

(Compose('simulator')
 .add('ros', Container('./ros', 'roscore')
      .volume('./ros', '/lunabotics/ros'))
 .add('webots', Container('./webots/docker', 'webots /webots_worlds/gridmap_testbed.wbt')
      .env('ROS_MASTER_URI', 'http://ros:11311')
      .volume('./webots/worlds', '/webots_worlds')
      .add_pkg('hello')
      .graphics())
 .network_host('ros-docker-net')
 .run())
