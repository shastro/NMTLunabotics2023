FROM ros:noetic-ros-core

RUN apt-get update -y

# Standard tools
RUN apt-get install -y clang make gcc g++ inetutils-ping net-tools git

# 2. Install ROS packages from apt.
RUN apt-get install -y \
    ros-noetic-move-base \
    ros-noetic-rviz \
    ros-noetic-rosmon \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher


# TODO: Add building catkin workspace to build step
