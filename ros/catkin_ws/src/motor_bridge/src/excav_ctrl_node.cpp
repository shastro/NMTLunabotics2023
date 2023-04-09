#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/System.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"
SocketCAN can;


void excavCallback(const motor_bridge::System::ConstPtr &msg) {
    int speed = msg->digger.rpm;
    std::cout << "Drive Message Received."
        << "speed: " << speed
        << std::endl;

    uint8_t message[8];
    int can_id;

    david_excav_ctrl_t left = {
        .rpm = (uint64_t)speed
    };
    david_excav_ctrl_pack(message, &left, sizeof(message));
    can_id = DAVID_EXCAV_CTRL_FRAME_ID;

    can.transmit(can_id, message);
}

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "excav_ctrl");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("excav_control", 1024, excavCallback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err << std::endl;
        return 1;
    }
}
