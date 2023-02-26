FROM cyberbotics/webots:R2022b-ubuntu20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    # ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-controller-manager \
    ros-noetic-webots-ros \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-grid-map \
    ros-noetic-eigen-conversions 


RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libeigen3-dev \
    python3-catkin-tools \
    qt5-default \
    libflann-dev \
    python3-pcl \
    pcl-tools \
    ros-noetic-pcl-ros \
    ros-noetic-tf-conversions \
    ros-noetic-ompl

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-ros \
    ros-noetic-turtlebot3*

WORKDIR /lunabotics/ros/catkin_ws
RUN mkdir /lunabotics/ros/catkin_ws/src

COPY webots_entrypoint /webots_entrypoint
COPY webots.conf /root/.config/Cyberbotics/Webots-R2022b.conf
ENTRYPOINT ["/webots_entrypoint"]
