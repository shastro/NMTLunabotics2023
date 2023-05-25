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
python3-catkin-tools \
libeigen3-dev

# 2. Install ROS packages from apt.
RUN apt-get install -y \
ros-noetic-roscpp \
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
ros-noetic-xacro \
ros-noetic-teb-local-planner

# Install visual and debugging tools
RUN apt-get install -y \
ros-noetic-rosbag \
ros-noetic-plotjuggler \
ros-noetic-rqt-tf-tree \
ros-noetic-rqt-plot \
ros-noetic-rqt-image-view  \
ros-noetic-dynamic-reconfigure \
ros-noetic-rqt-reconfigure


RUN apt-get install -y software-properties-common
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
|| apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update
RUN apt-get install -y \
ros-noetic-librealsense2

## Elevation Mapping GPU dependencies
#RUN apt-get install -y python3-pip
#RUN apt-get install -y wget
#RUN wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
#RUN apt-get -y update 
#RUN apt-get -y install \
#    autoconf \
#    bc \
#    build-essential \
#    g++-8 \
#    gcc-8 \
#    clang-8 \
#    lld-8 \
#    gettext-base \
#    gfortran-8 \
#    iputils-ping \
#    libbz2-dev \
#    libc++-dev \
#    libcgal-dev \
#    libffi-dev \
#    libfreetype6-dev \
#    libhdf5-dev \
#    libjpeg-dev \
#    liblzma-dev \
#    libncurses5-dev \
#    libncursesw5-dev \
#    libpng-dev \
#    libreadline-dev \
#    libssl-dev \
#    libsqlite3-dev \
#    libxml2-dev \
#    libxslt-dev \
#    locales \
#    moreutils \
#    openssl \
#    python-openssl \
#    rsync \
#    scons \
#    python3-pip \
#    libopenblas-dev
#RUN python3 -m pip install --upgrade pip
#RUN python3 -m pip install aiohttp numpy=='1.19.4' scipy=='1.5.3'
#RUN python3 -m pip install --upgrade protobuf
#RUN python3 -m pip install --no-cache https://developer.download.nvidia.cn/compute/redist/jp/v51/pytorch/torch-2.0.0a0+8aa34602.nv23.03-cp38-cp38-linux_aarch64.whl
#RUN pip3 install Cython
#RUN pip3 install cupy
#RUN apt-get install -y ros-melodic-pybind11-catkin
#RUN apt-get install -y ros-melodic-grid-map-core ros-melodic-grid-map-msgs
#RUN apt-get install gfortran
#RUN apt-get install -y libopencv-dev
#RUN apt-get install -y libgmp-dev
#RUN apt-get install -y libmpfr-dev
#RUN apt-get install -y libboost-all-dev
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

COPY ros/scripts /ros/scripts
COPY ros/catkin_ws/.catkin_workspace /ros/catkin_ws/.catkin_workspace

# Incremental-ish builds to avoid full rebuilds. Keep these in order
# from least to most often changed.
# Uncomment the ones to be compiled
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ros/catkin_ws/src/kindr /ros/catkin_ws/src/kindr
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/kindr_ros /ros/catkin_ws/src/kindr_ros
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/message_logger /ros/catkin_ws/src/message_logger
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/realsense-ros /ros/catkin_ws/src/realsense-ros
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

#COPY ros/catkin_ws/src/elevation_mapping /ros/catkin_ws/src/elevation_mapping
#RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

#COPY ros/catkin_ws/src/elevation_mapping_demos /ros/catkin_ws/src/elevation_mapping_demos
#RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/elevation_mapping /ros/catkin_ws/src/elevation_mapping_cupy
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY can /can

RUN apt-get install -y \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-gmapping 

COPY ros/catkin_ws/src/motor_bridge /ros/catkin_ws/src/motor_bridge
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/pad_send /ros/catkin_ws/src/pad_send
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/david_config /ros/catkin_ws/src/david_config
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

COPY ros/catkin_ws/src/david_navigation /ros/catkin_ws/src/david_navigation
RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build

# Build any packages not listed above.
#RUN rm -rf /ros/catkin_ws/src
#COPY ros/catkin_ws/src /ros/catkin_ws/src
#RUN cd /ros/catkin_ws && /ros_entrypoint.sh catkin build


# Add /catkin_ws to the ROS environment.
COPY ros/ros_entrypoint.sh /ros_entrypoint.sh
