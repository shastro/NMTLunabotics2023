#include <iostream>
#include <string>
#include <ncurses.h>
#include <signal.h>

#include <ros/ros.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/canshit.hpp"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "vcan0"
static SocketCAN can;

motor_bridge::System lastMsg;
struct sigaction sigIntHandler;

void callback(const motor_bridge::System::ConstPtr &msg);
void send(uint8_t* buff, int id, const char* name, std::string print);
void stop(const motor_bridge::System::ConstPtr &msg);
void pitch(const motor_bridge::System::ConstPtr &msg);
void loco(const motor_bridge::System::ConstPtr &msg);
void excav(const motor_bridge::System::ConstPtr &msg);
void stepper(const motor_bridge::System::ConstPtr &msg);

void exit(int s);

int main(int argc, char **argv) {
    // Setup ncurses
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();
    halfdelay(1);
    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);

    sigIntHandler.sa_handler = exit;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

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
    if (lastMsg == *msg) { return; }
    erase();
    move(0, 0);

    stop(msg);
    pitch(msg);
    loco(msg);
    excav(msg);
    stepper(msg);

    lastMsg = *msg;

    sigaction(SIGINT, &sigIntHandler, NULL);
}

void stop(const motor_bridge::System::ConstPtr &msg) {
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(msg->e_stop, buff);
    send(buff, can_id, "EStop", printable(msg->e_stop));
}

void pitch(const motor_bridge::System::ConstPtr &msg) {
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(msg->pitch_ctrl, buff);
    send(buff, can_id, "Pitch", printable(msg->pitch_ctrl));
}

void loco(const motor_bridge::System::ConstPtr &msg) {
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(msg->loco_ctrl, buff);
    send(buff, can_id, "Loco", printable(msg->loco_ctrl));
}

void excav(const motor_bridge::System::ConstPtr &msg) {
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(msg->excav_ctrl, buff);
    send(buff, can_id, "Excav", printable(msg->excav_ctrl));
}

void stepper(const motor_bridge::System::ConstPtr &msg) {
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(msg->stepper_ctrl, buff);
    send(buff, can_id, "Stepper", printable(msg->stepper_ctrl));
}

void send(uint8_t* buff, int id, const char* name, std::string print) {
    try {
        can.transmit(id, buff);
        attron(COLOR_PAIR(1));
        printw(print.c_str());
        attroff(COLOR_PAIR(1));
    } catch (std::string err) {
        attron(COLOR_PAIR(2));
        printw(name);
        printw(": ");
        printw(err.c_str());
        printw("\n");
        attroff(COLOR_PAIR(2));
    }
    refresh();
}

void exit(int s) {
    endwin();
    exit(1);
}
