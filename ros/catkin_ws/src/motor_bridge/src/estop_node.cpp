#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/Estop.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"
SocketCAN can;


void estopCallback(const motor_bridge::Estop::ConstPtr &msg) {
    bool stop = msg->stop;
    std::cout << "Stop Message Received."
        << "stop: " << stop
        << std::endl;
    if (stop) {
        uint8_t message[8];
        int can_id;

        david_e_stop_t estop = {};
        david_e_stop_pack(message, &estop, sizeof(message));
        can_id = DAVID_E_STOP_FRAME_ID;

        can.transmit(can_id, message);
    }
}

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "estop");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("estop", 1024, estopCallback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err << std::endl;
        return 1;
    }
}
