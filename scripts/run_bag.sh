#!/usr/bin/env bash
set -xeuo pipefail
IFS=$'\n\t'

BAG=_2023-05-24-16-20-17_4.bag
# BAG=2023-05-25-03-19-23.bag

stat "$BAG"

roslaunch david_config mapping.launch &
sleep 2

CAMS=(d455_1 d455_2 l515_1 l515_2 t265)
for cam in "${CAMS[@]}"; do
    rosrun tf2_ros static_transform_publisher \
           0 0 0 1.5707963268 3.1415926535 1.5707963268 \
           ${cam}_link ${cam}_depth_optical_frame &
done

mk_topics () {
    for cam in "${CAMS[@]}"; do
        echo "/$cam/depth/color/points"
    done
    echo /usb_cam/image_raw
}
rosbag play "$BAG" --topics /tf $(mk_topics) --loop &

rviz

kill $(jobs -p)
