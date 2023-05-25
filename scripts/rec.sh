#!/bin/bash

rosbag record --duration=1m -o /mnt/media/docker -a > /dev/null 2>&1 &
