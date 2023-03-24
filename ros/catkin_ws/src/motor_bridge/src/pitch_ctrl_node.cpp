#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/Pitch.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"

enum motor {
    BOTH = 0,
    LEFT = 1,
    RIGHT = 2
};

void pitchCallback(const motor_bridge::Pitch::ConstPtr &msg) {
    int length = msg->length;
    motor m = (motor) msg->motor;
    std::cout << "Pitch Message Received. length: "
        << length << " motor: " << m << std::endl;

    uint8_t message[8];
    SocketCAN can(CAN_BUS);
    int can_id;

    if (m = LEFT) {
        david_pitch_ctrl_left_t left = {
            .count = (uint64_t)length,
            .tolerance = 100,
        };
        david_pitch_ctrl_left_pack(message, &left, sizeof(message));
        can_id = DAVID_PITCH_CTRL_LEFT_FRAME_ID;
    } else if (m = RIGHT) {
        david_pitch_ctrl_right_t right = {
            .count = (uint64_t)length,
            .tolerance = 100,
        };
        david_pitch_ctrl_right_pack(message, &right, sizeof(message));
        can_id = DAVID_PITCH_CTRL_RIGHT_FRAME_ID;
    } else {
        david_pitch_ctrl_both_t both = {
            .count = (uint64_t)length,
            .tolerance = 100,
        };
        david_pitch_ctrl_both_pack(message, &both, sizeof(message));
        can_id = DAVID_PITCH_CTRL_BOTH_FRAME_ID;
    }

    can.transmit(message, can_id);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pitch_ctrl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pitch_control", 1024, pitchCallback);

    // Callback event loop
    ros::spin();

    return 0;
}
