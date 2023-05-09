#include <iostream>
#include <string>
#include <ncurses.h>

#include <ros/ros.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/canshit.hpp"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "can0"
static SocketCAN can;

motor_bridge::System lastMsg;
bool reduce_can = true;
void callback(const motor_bridge::System::ConstPtr &msg);
void send(uint8_t* buff, int id, const char* name);

int main(int argc, char **argv) {
    try {

        // Setup ncurses
        initscr();
        cbreak();
        keypad(stdscr, true);
        noecho();
        halfdelay(1);
        start_color();
        init_pair(1, COLOR_YELLOW, COLOR_BLACK);

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
    if (lastMsg == *msg) { return; }
    erase();
    move(0, 0);

    uint8_t buff[8];
    int can_id;

    can_id = pack_msg(msg->e_stop, buff);
    printw(printable(msg->e_stop).c_str());
    send(buff, can_id, "EStop");

    can_id = pack_msg(msg->pitch_ctrl, buff);
    printw(printable(msg->pitch_ctrl).c_str());
    send(buff, can_id, "Pitch");

    can_id = pack_msg(msg->loco_ctrl, buff);
    printw(printable(msg->loco_ctrl).c_str());
    //send(buff, can_id, "Loco");
    david_loco_ctrl_t t {
        .left_vel = msg->loco_ctrl.left_vel,
        .right_vel = msg->loco_ctrl.right_vel,
    };
    david_loco_ctrl_pack(buff, &t, sizeof(t));
    can.transmit(DAVID_LOCO_CTRL_FRAME_ID, buff);

    can_id = pack_msg(msg->excav_ctrl, buff);
    printw(printable(msg->excav_ctrl).c_str());
    send(buff, can_id, "Excav");

    can_id = pack_msg(msg->stepper_ctrl, buff);
    printw(printable(msg->stepper_ctrl).c_str());
    send(buff, can_id, "Stepper");

    refresh();

    lastMsg = *msg;
}

void send(uint8_t* buff, int id, const char* name) {
    try {
        can.transmit(id, buff);
    } catch (std::string err) {
        attron(COLOR_PAIR(1));
        printw(name);
        printw(": ");
        printw(err.c_str());
        printw("\n");
        refresh();
        attroff(COLOR_PAIR(1));
        refresh();
    }
}

