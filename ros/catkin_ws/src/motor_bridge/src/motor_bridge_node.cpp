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
bool debug = true;

motor_bridge::EStop last_stop;
motor_bridge::PitchCtrl last_pitch;
motor_bridge::LocoCtrl last_loco;
motor_bridge::ExcavCtrl last_excav;
motor_bridge::StepperCtrl last_stepper;
motor_bridge::MastCtrl last_mast;

void stop(const motor_bridge::EStop::ConstPtr &msg);
void pitch(const motor_bridge::PitchCtrl::ConstPtr &msg);
void loco(const motor_bridge::LocoCtrl::ConstPtr &msg);
void excav(const motor_bridge::ExcavCtrl::ConstPtr &msg);
void stepper(const motor_bridge::StepperCtrl::ConstPtr &msg);
void mast(const motor_bridge::MastCtrl::ConstPtr &msg);

void send(uint8_t* buff, int id, const char* name, std::string print);

int main(int argc, char **argv) {
    // Setup ncurses
    if (debug) {
        /*
        initscr();
        cbreak();
        keypad(stdscr, true);
        noecho();
        halfdelay(1);
        start_color();
        init_pair(1, COLOR_WHITE, COLOR_BLACK);
        init_pair(2, COLOR_YELLOW, COLOR_BLACK);
        */
    }

    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "motor_bridge");
        ros::NodeHandle nh;
        ros::Subscriber stop_sub = nh.subscribe("system/stop", 5, stop);
        ros::Subscriber pitch_sub = nh.subscribe("system/pitch_ctrl", 5, pitch);
        ros::Subscriber loco_sub = nh.subscribe("system/loco_ctrl", 5, loco);
        ros::Subscriber excav_sub = nh.subscribe("system/excav_ctrl", 5, excav);
        ros::Subscriber stepper_sub = nh.subscribe("system/stepper_ctrl", 5, stepper);
        ros::Subscriber mast_sub = nh.subscribe("system/mast_ctrl", 5, mast);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cerr << err;
        return 1;
    }
}

void stop(const motor_bridge::EStop::ConstPtr &msg) {
    if (*msg == last_stop) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "EStop", printable(*msg));
    last_stop = *msg;
}

void pitch(const motor_bridge::PitchCtrl::ConstPtr &msg) {
    if (*msg == last_pitch) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "Pitch", printable(*msg));
    last_pitch = *msg;
}

void loco(const motor_bridge::LocoCtrl::ConstPtr &msg) {
    if (*msg == last_loco) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "Loco", printable(*msg));
    last_loco = *msg;
}

void excav(const motor_bridge::ExcavCtrl::ConstPtr &msg) {
    if (*msg == last_excav) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "Excav", printable(*msg));
    last_excav = *msg;
}

void stepper(const motor_bridge::StepperCtrl::ConstPtr &msg) {
    if (*msg == last_stepper) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "Stepper", printable(*msg));
    last_stepper = *msg;
}

void mast(const motor_bridge::MastCtrl::ConstPtr &msg) {
    if (*msg == last_mast) { return; }
    uint8_t buff[8];
    int can_id;
    can_id = pack_msg(*msg, buff);
    send(buff, can_id, "Mast", printable(*msg));
    last_mast = *msg;
}

void send(uint8_t* buff, int id, const char* name, std::string print) {
    if (debug) {
    //move(0, 0);
    //clear();
    }
    try {
        can.transmit(id, buff);
        if (debug) {
            std::cout << print;
            /*
            attron(COLOR_PAIR(1));
            printw(print.c_str());
            attroff(COLOR_PAIR(1));
            */
        }
    } catch (std::string err) {
        if (debug) {
            std::cerr << err << std::endl;
            /*
            attron(COLOR_PAIR(2));
            printw(name);
            printw(": ");
            printw(err.c_str());
            printw("\n");
            attroff(COLOR_PAIR(2));
            */
        }
    }
    //if (debug) { refresh(); }
}

