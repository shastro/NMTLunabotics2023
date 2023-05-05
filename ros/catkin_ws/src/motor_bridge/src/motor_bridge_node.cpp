#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/canshit.hpp"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "vcan0"
static SocketCAN can;

motor_bridge::System lastMsg;
void callback(const motor_bridge::System::ConstPtr &msg);

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "motor_bridge");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("system", 5, callback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err;
        return 1;
    }
}

void callback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.e_stop == msg->e_stop) {
        uint8_t buff[8];
        int can_id = pack_msg(msg->e_stop, buff);
        std::cout << msg->e_stop;
        try {
            can.transmit(can_id, buff);
        } catch (std::string err) {
            std::cout << "\e[93mEstop: " << err << "\e[0m" << std::endl;
        }
    }
    
    if (lastMsg.pitch_ctrl == msg->pitch_ctrl) {
        uint8_t buff[8];
        int can_id = pack_msg(msg->pitch_ctrl, buff);
        std::cout << msg->pitch_ctrl;
        try {
            can.transmit(can_id, buff);
        } catch (std::string err) {
            std::cout << "\e[93mPitch: " << err << "\e[0m" << std::endl;
        }
    }

    if (lastMsg.loco_ctrl == msg->loco_ctrl) {
        uint8_t buff[8];
        int can_id = pack_msg(msg->loco_ctrl, buff);
        std::cout << msg->loco_ctrl;
        try {
            can.transmit(can_id, buff);
        } catch (std::string err) {
            std::cout << "\e[93mLoco: " << err << "\e[0m" << std::endl;
        }
    }
    
    if (lastMsg.excav_ctrl == msg->excav_ctrl) {
        uint8_t buff[8];
        int can_id = pack_msg(msg->excav_ctrl, buff);
        std::cout << msg->excav_ctrl;
        try {
            can.transmit(can_id, buff);
        } catch (std::string err) {
            std::cout << "\e[93mExcav: " << err << "\e[0m" << std::endl;
        }
    }
    
    if (lastMsg.stepper_ctrl == msg->stepper_ctrl) {
        uint8_t buff[8];
        int can_id = pack_msg(msg->stepper_ctrl, buff);
        std::cout << msg->stepper_ctrl;
        try {
            can.transmit(can_id, buff);
        } catch (std::string err) {
            std::cout << "\e[93mStepper: " << err << "\e[0m" << std::endl;
        }
    }
    
    lastMsg = *msg;
}
