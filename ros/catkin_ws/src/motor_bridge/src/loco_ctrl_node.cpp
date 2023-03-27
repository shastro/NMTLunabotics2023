#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/Drive.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"
SocketCAN can;

enum motor {LEFT = 1, RIGHT = 2};

void locoCallback(const motor_bridge::Drive::ConstPtr &msg) {
    int speed = msg->speed;
    int m = msg->motor;
    std::cout << "Drive Message Received. motor: " << m
        << "speed: " << speed
        << std::endl;

    uint8_t message[8];

    if (m == LEFT) {
        david_loco_ctrl_left_t left = {
            .velocity = (uint64_t)speed
        };
        david_loco_ctrl_left_pack(message, &left, sizeof(message));
        can_id = DAVID_LOCO_CTRL_LEFT_FRAME_ID;
    } else {
        david_loco_ctrl_right_t right = {
            .velocity = (uint64_t)speed
        };
        david_loco_ctrl_right_pack(message, &right, sizeof(message));
        can_id = DAVID_LOCO_CTRL_RIGHT_FRAME_ID;
    }

    can.transmit(can_id, message);
}

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "loco_ctrl_node");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("loco_control", 1024, locoCallback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err << std::endl;
        return 1;
    }
}
