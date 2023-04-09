#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/System.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"
SocketCAN can;

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum dir { BACKWARD = -1, STOP = 0, FORWARD = 0 };

void steppCallback(const motor_bridge::System::ConstPtr &msg) {
    motor m = (motor)msg->extend.motor;
    dir d = (dir)msg->extend.direction;
    int rpm = msg->extend.rpm;
    std::cout << "Stepp Message Received. motor: " << m
        << " direction: " << d
        << " rmp: " << rpm << std::endl;

    uint8_t message[8];
    int can_id;

    if (m == LEFT) {
        david_stepper_ctrl_left_t left = {
            .rpm = (uint32_t)rpm,
            .direction = (uint32_t)d,
        };
        david_stepper_ctrl_left_pack(message, &left, sizeof(message));
        can_id = DAVID_STEPPER_CTRL_LEFT_FRAME_ID;
    } else if (m == RIGHT) {
        david_stepper_ctrl_right_t right = {
            .rpm = (uint32_t)rpm,
            .direction = (uint32_t)d,
        };
        david_stepper_ctrl_right_pack(message, &right, sizeof(message));
        can_id = DAVID_STEPPER_CTRL_RIGHT_FRAME_ID;
    } else {
        david_stepper_ctrl_both_t both = {
            .rpm = (uint32_t)rpm,
            .direction = (uint32_t)d,
        };
        david_stepper_ctrl_both_pack(message, &both, sizeof(message));
        can_id = DAVID_STEPPER_CTRL_BOTH_FRAME_ID;
    }

    can.transmit(can_id, message);
}

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "stepp_ctrl");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("stepp_control", 1024, steppCallback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err << std::endl;
        return 1;
    }
}
