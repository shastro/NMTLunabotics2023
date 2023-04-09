#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/System.h>

#include "can/david.h"
#include "can_interface.hpp"

#define CAN_BUS "can0"
SocketCAN can;


void estopCallback(const motor_bridge::System::ConstPtr &msg) {
    bool stop = msg->estop;

    uint8_t message[8];
    int can_id;

    std::cout << "Stop Message Received: ";

    if (stop) {
        std::cout << "Stop" << std::endl;
        david_e_stop_t estop = {};
        david_e_stop_pack(message, &estop, sizeof(message));
        can_id = DAVID_E_STOP_FRAME_ID;

        can.transmit(can_id, message);
    } else {
        std::cout << "Not Stop" << std::endl;
        david_e_start_t estart = {};
        david_e_start_pack(message, &estart, sizeof(message));
        can_id = DAVID_E_START_FRAME_ID;

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
