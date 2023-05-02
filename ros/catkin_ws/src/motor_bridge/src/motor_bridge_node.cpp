#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/david.h"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "vcan0"
static SocketCAN can;

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

motor_bridge::System lastMsg;

//TODO better ros logging
std::ostream& operator<<(std::ostream& os, const motor& m);
std::ostream& operator<<(std::ostream& os, const direction& d);
void estopCallback(const motor_bridge::System::ConstPtr &msg);
void pitchCallback(const motor_bridge::System::ConstPtr &msg);
void locoCallback(const motor_bridge::System::ConstPtr &msg);
void excavCallback(const motor_bridge::System::ConstPtr &msg);
void steppCallback(const motor_bridge::System::ConstPtr &msg);
void callback(const motor_bridge::System::ConstPtr &msg);

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "motor_bridge");
        std::cout << "motor_bridge started" << std::endl;
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("system", 1024, callback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cout << err << std::endl;
        return 1;
    }
}

void callback(const motor_bridge::System::ConstPtr &msg) {
    estopCallback(msg);
    pitchCallback(msg);
    excavCallback(msg);
    locoCallback(msg);
    steppCallback(msg);
    lastMsg = *msg;
}

void estopCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.estop == msg->estop) { return; }
    bool stop = msg->estop;

    uint8_t message[8];
    int can_id;

    if (stop) {
        std::cout << "ESTOP YES" << std::endl;
        david_e_stop_t estop = {};
        david_e_stop_pack(message, &estop, sizeof(message));
        can_id = DAVID_E_STOP_FRAME_ID;

    } else {
        std::cout << "ESTOP NO" << std::endl;
        david_e_start_t estart = {};
        david_e_start_pack(message, &estart, sizeof(message));
        can_id = DAVID_E_START_FRAME_ID;
    }
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "estop failed: " << err << std::endl;
        std::cout << "\e[0m";
    }
}

void pitchCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.pitch == msg->pitch) { return; }
    direction dir = (direction)msg->pitch.direction;
    motor m = (motor)msg->pitch.motor;

    std::cout << "Pitch Message Received. dir: " << dir << " motor: " << m << std::endl;

    uint8_t message[8];
    int can_id = DAVID_PITCH_CTRL_BOTH_FRAME_ID;


    if (m == LEFT) {
        david_pitch_ctrl_left_t left = {
            .command = (uint8_t)dir,
        };
        david_pitch_ctrl_left_pack(message, &left, sizeof(message));
        can_id = DAVID_PITCH_CTRL_LEFT_FRAME_ID;
    } else if (m == RIGHT) {
        david_pitch_ctrl_right_t right = {
            .command = (uint8_t)dir,
        };
        david_pitch_ctrl_right_pack(message, &right, sizeof(message));
        can_id = DAVID_PITCH_CTRL_RIGHT_FRAME_ID;
    } else {
        david_pitch_ctrl_both_t both = {
            .command = (uint8_t)dir,
        };
        david_pitch_ctrl_both_pack(message, &both, sizeof(message));
        can_id = DAVID_PITCH_CTRL_BOTH_FRAME_ID;
    }
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "pitch failed: " << err << std::endl << std::endl;
        std::cout << "\e[0m";
    }
}

void locoCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.left == msg->left && lastMsg.right == msg->right) { return; }
    int lspeed = msg->left.rpm;
    int rspeed = msg->right.rpm;

    std::cout << "Drive Message Received."
        << " left motor: " << lspeed
        << ", right motor: " << rspeed << std::endl;

    uint8_t message[8];
    int can_id;
    david_loco_ctrl_left_t left = {
        .velocity = (uint64_t)lspeed
    };
    david_loco_ctrl_left_pack(message, &left, sizeof(message));
    can_id = DAVID_LOCO_CTRL_LEFT_FRAME_ID;
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "left motor failed: " << err << std::endl;
        std::cout << "\e[0m";
    }

    david_loco_ctrl_right_t right = {
        .velocity = (uint64_t)rspeed
    };
    david_loco_ctrl_right_pack(message, &right, sizeof(message));
    can_id = DAVID_LOCO_CTRL_RIGHT_FRAME_ID;
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "right motor failed: " << err << std::endl;
        std::cout << "\e[0m";
    }
}

void excavCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.digger == msg->digger) { return; }
    int speed = msg->digger.rpm;

    std::cout << "Digger Message Received."
        << "speed: " << speed << std::endl;

    uint8_t message[8];
    int can_id;

    david_excav_ctrl_t e = {
        .rpm = (uint64_t)speed
    };
    david_excav_ctrl_pack(message, &e, sizeof(message));
    can_id = DAVID_EXCAV_CTRL_FRAME_ID;

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "excav failed: " << err << std::endl;
        std::cout << "\e[0m";
    }
}

void steppCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.extend == msg->extend) { return; }
    motor m = (motor)msg->extend.motor;
    direction d = (direction)msg->extend.direction;
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

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        std::cout << "\e[93m";
        std::cout << "stepp failed: " << err << std::endl;
        std::cout << "\e[0m";
    }
}

std::ostream& operator<<(std::ostream& s, const motor& m) {
    switch (m) {
        case BOTH:
            return s << "BOTH";
        case LEFT:
            return s << "LEFT";
        case RIGHT:
            return s << "RIGHT";
    }
}

std::ostream& operator<<(std::ostream& s, const direction& d) {
    switch (d) {
        case STOP:
            return s << "STOP";
        case FORWARD:
            return s << "FORWARD";
        case BACKWARD:
            return s << "BACKWARD";
    }
}
