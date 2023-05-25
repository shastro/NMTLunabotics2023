#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'
set -x

# robot.sh: runs the main robot container.

disable_rebuild=false
while getopts 'd' OPTION; do
    case "$OPTION" in
        d)
            disable_rebuild=true
            ;;
    esac
done

DIR=../
IMAGE_NAME=lunabotics-2023-ros
CONTAINER_NAME=lunabotics-2023-robot

# Ensure CAN is up-to-date.
make -C $DIR/can/bus_spec

# Always run as root.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

# Build the image.
if ! $disable_rebuild; then
    docker build "$DIR" -f "$DIR"/Dockerfile_full_build -t $IMAGE_NAME
fi

it=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')

params=(
    # Detach from the container, and create a fake virtual terminal.
    -dt

    --rm
    # Use host's network interfaces to listen to ROS, and operate the
    # CAN bus.
    --network=host

    # Allow access to devices.
    --volume=/dev:/dev

    # Device rules for accessing Realsense cameras.
    --device-cgroup-rule "c 81:* rmw"
    --device-cgroup-rule "c 189:* rmw"
    --privileged

    # Environmental variables
    -e ROS_MASTER_URI=http://jetson:11311
    -e ROS_IP=$it

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
docker run "${params[@]}" $IMAGE_NAME rosrun usb_cam_excav usb_cam_node image_width:=320 image_height:=240 framerate:=30 

sleep 5

docker run "${params[@]}" $IMAGE_NAME rosrun usb_cam_fwd usb_cam_node image_width:=320 image_height:=240 framerate:=30 

sleep 5

libcamera-vid -n -t 0 --inline -o 'udp://192.168.1.45:5000?overrun_nonfatal=1&fifo_size=50000000' --width 320 --height 180 --codec h264
