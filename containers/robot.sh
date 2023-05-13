#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'
set -x

# robot.sh: runs the main robot container.

DIR=../
IMAGE_NAME=lunabotics-2023-ros
CONTAINER_NAME=lunabotics-2023-robot

# Ensure CAN is up-to-date.
make -C $DIR/can/bus_spec

# Always run as root.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

# Build the image.
docker build "$DIR" -f "$DIR"/Dockerfile_full_build -t $IMAGE_NAME

it=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')

params=(
    # Detach from the container, and create a fake virtual terminal.
    -dt

    # Use host's network interfaces to listen to ROS, and operate the
    # CAN bus.
    --network=host

    # Allow access to devices.
    --volume=/dev:/dev

    # Device rules for accessing Realsense cameras.
    --device-cgroup-rule "c 81:* rmw"
    --device-cgroup-rule "c 189:* rmw"
    
    # Environmental variables
    -e ROS_MASTER_URI=http://$it:11311
    -e ROS_IP=$it

    # Name the container.
    --name=$CONTAINER_NAME
)

# Clean up the container if it's already running.
docker stop $CONTAINER_NAME || true
docker rm $CONTAINER_NAME || true

# Clean up on ctrl-C.
cleanup () {
    docker stop $CONTAINER_NAME || true
    docker rm $CONTAINER_NAME || true
    exit 1
}
trap cleanup INT

# Set up the ROS core.
docker run "${params[@]}" $IMAGE_NAME roscore

# Run the camera node.
docker exec -d $CONTAINER_NAME /ros_entrypoint.sh /scripts/unfuck_realsense

# Run the motor_bridge node.
docker exec -d $CONTAINER_NAME /ros_entrypoint.sh rosrun \
    motor_bridge motor_bridge

# Run cameras
docker exec -dt $CONTAINER_NAME /ros_entrypoint.sh roslaunch --wait \
    realsense2_camera rs_camera.launch \
    camera:=d455_1 serial_no:=213522250920 filters:=pointcloud

docker exec -dt $CONTAINER_NAME /ros_entrypoint.sh roslaunch --wait \
    realsense2_camera rs_camera.launch \
    camera:=d455_2 serial_no:=213522253528 filters:=pointcloud

docker exec -dt $CONTAINER_NAME /ros_entrypoint.sh roslaunch --wait \
    realsense2_camera rs_camera.launch \
    camera:=l515_1 serial_no:=f1381818 filters:=pointcloud

docker exec -dt $CONTAINER_NAME /ros_entrypoint.sh roslaunch --wait \
    realsense2_camera rs_camera.launch \
    camera:=l515_2 serial_no:=f0461308 filters:=pointcloud

docker exec -dt $CONTAINER_NAME /ros_entrypoint.sh roslaunch --wait \
    realsense2_camera rs_t265.launch
