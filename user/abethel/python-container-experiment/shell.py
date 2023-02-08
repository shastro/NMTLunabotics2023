#!/usr/bin/env python3

"""Connects to ROS and drops you in a terminal."""

import os

os.system('sudo docker exec -it containers-ros-1 /bin/bash')
