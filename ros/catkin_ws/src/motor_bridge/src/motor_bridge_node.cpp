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
        init_pair(2, COLOR_WHITE, COLOR_BLACK);

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

void print(std::string s, int color) {
    attron(COLOR_PAIR(color));
    printw(s.c_str());
    attroff(COLOR_PAIR(color));
    refresh();
}

void print(const char* s, int color) {
    attron(COLOR_PAIR(color));
    printw(s);
    attroff(COLOR_PAIR(color));
    refresh();
}

void callback(const motor_bridge::System::ConstPtr &msg) {
    erase();
    move(0, 0);

    e_stop();
    pitch_ctrl();

    lastMsg = *msg;
}

void e_stop() {
    if (msg->e_stop == lastMsg.e_stop) { return; }
    
    int id = DAVID_E_STOP_FRAME_ID;
    uint8_t buff[8];
    david_e_stop_t t {
        .stop = msg->e_stop.stop,
    };
    david_e_stop_pack(buff, &t, sizeof(t));

    try {
        can.transmit(id, buff);
        print(printable(msg->e_stop), 1);
        refresh();
    } catch (std::string err) {
        print("EStop: ", 2);
        print(err, 2);
        print("\n", 2);
    }
}

void pitch_ctrl() {
    if (msg->pitch_ctrl == lastMsg.pitch_ctrl) { return; }

    int id = DAVID_PTICH_CTRL_FRAME_ID;
    uint8_t buff[8];
    david_pitch_ctrl_t t {
        .set_point = msg->pitch_ctrl.set_point;
        .left_offset = msg->pitch_ctrl.left_offset;
        .right_offset = msg->pitch_ctrl.right_offset;
    };
    david_pitch_ctrl_pack(buff, &t, sizeof(t));

    try {
        can.transmit(id, buff);
        print(printable(msg->pitch_ctrl), 1);
        refresh();
    } catch (std::string err) {
        print("Pitch Ctrl: ", 2);
        print(err, 2);
        print("\n", 2);
    }
}

