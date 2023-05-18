FROM ros:noetic-ros-core

RUN apt-get update -y

RUN DEBIAN_FRONTEND='noninteractive' apt-get install -y libpcl-dev
# Standard tools
RUN apt-get install -y \
    clang \
    make \
    gcc \
    g++ \
    inetutils-ping \
    net-tools \
    git \
    tmux \
    python3-catkin-tools

# 2. Install ROS packages from apt.
RUN apt-get install -y \
    ros-noetic-eigen-conversions \
    ros-noetic-gazebo-ros \
    ros-noetic-grid-map-core \
    ros-noetic-grid-map-ros \
    ros-noetic-joint-state-publisher \
    ros-noetic-move-base \
    ros-noetic-ompl \
    ros-noetic-realsense2-camera \
    ros-noetic-robot-pose-ekf \
    ros-noetic-robot-state-publisher \
    ros-noetic-rosmon \
    ros-noetic-rviz \
    ros-noetic-tf2-tools \
    ros-noetic-usb-cam \
    ros-noetic-compressed-image-transport \
    ros-noetic-rosmon \
    ros-noetic-pcl-ros \
    ros-noetic-tf-conversions \
    ros-noetic-grid-map-filters \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-xacro

COPY ros/scripts /ros/scripts
COPY ros/catkin_ws /ros/catkin_ws
COPY can /can

# Add /catkin_ws to the ROS environment.
COPY ros/ros_entrypoint.sh /ros_entrypoint.sh
