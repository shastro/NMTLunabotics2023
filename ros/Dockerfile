FROM ros:noetic-ros-core

RUN apt-get update -y

# Standard tools
RUN apt-get install -y clang \
    make gcc g++ \
    inetutils-ping net-tools openssh-server\
    git tmux

# 2. Install ROS packages from apt.
RUN apt-get install -y \
    ros-noetic-move-base \
    ros-noetic-rviz \
    ros-noetic-rosmon \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-realsense2-camera \
    ros-noetic-ompl \
    ros-noetic-grid-map-core \
    ros-noetic-grid-map-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-gazebo-ros

RUN mkdir /var/run/sshd
RUN echo 'root:mypassword' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config
RUN service ssh start
EXPOSE 22
EXPOSE 11311
CMD ["/usr/sbin/sshd", "-D"]    

COPY scripts /scripts
COPY catkin_ws /catkin_ws

# RUN cd /catkin_ws && \
#     rm -rf /catkin_ws/build /catkin_ws/devel && \
#     /ros_entrypoint.sh catkin_make

# Add /catkin_ws to the ROS environment.
# COPY ros_entrypoint.sh /ros_entrypoint.sh

