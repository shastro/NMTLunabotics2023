#include <iostream>
#include <string>
#include <ncurses.h>

#include <ros/ros.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/canshit.hpp"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "vcan0"
static SocketCAN can;

motor_bridge::System lastMsg;
bool reduce_can = true;
void callback(const motor_bridge::System::ConstPtr &msg);

int main(int argc, char **argv) {
    try {

        // Setup ncurses
        initscr();
        cbreak();
        keypad(stdscr, true);
        noecho();
        halfdelay(1);

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
    erase();
    move(0, 0);

    uint8_t buff[8];
    int can_id;
    std::stringstream out;

    can_id = pack_msg(msg->e_stop, buff);
    out << msg->e_stop;
    try {
        can.transmit(can_id, buff);
    } catch (std::string err) {
        out << "\e[93mEstop: " << err << "\e[0m" << std::endl;
    }

    can_id = pack_msg(msg->pitch_ctrl, buff);
    out << msg->pitch_ctrl;
    try {
        can.transmit(can_id, buff);
    } catch (std::string err) {
        out << "\e[93mPitch: " << err << "\e[0m" << std::endl;
    }

    can_id = pack_msg(msg->loco_ctrl, buff);
    out << msg->loco_ctrl;
    try {
        can.transmit(can_id, buff);
    } catch (std::string err) {
        //out << "\e[93mLoco: " << err << "\e[0m" << std::endl;
    }

    can_id = pack_msg(msg->excav_ctrl, buff);
    out << msg->excav_ctrl;
    try {
        can.transmit(can_id, buff);
    } catch (std::string err) {
        //out << "\e[93mExcav: " << err << "\e[0m" << std::endl;
    }

    can_id = pack_msg(msg->stepper_ctrl, buff);
    out << msg->stepper_ctrl;
    try {
        can.transmit(can_id, buff);
    } catch (std::string err) {
        //out << "\e[93mStepper: " << err << "\e[0m" << std::endl;
    }

    printw(out.str().c_str());
    refresh();

    lastMsg = *msg;
}
