FROM ros:noetic-ros-core

WORKDIR /lunabotics/ros/catkin_ws
RUN mkdir /lunabotics/ros/catkin_ws/src

COPY ./webots_entrypoint /webots_entrypoint
ENTRYPOINT ["/webots_entrypoint"]


RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-webots-ros \
    && rm -rf /var/lib/apt/lists/*

# RUN rosdep init
# RUN sh -c "/install_webots_ros"
# # Setup webots ros
# RUN apt-get update && apt-get install -q -y --no-install-recommends \
#     git \
#     && rm -rf /var/lib/apt/lists/*

# RUN git clone https://github.com/cyberbotics/webots_ros.git

# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src --rosdistro noetic
# RUN catkin_make
# RUN source /lunabotics/ros/catkin_ws/devel/setup.bash
