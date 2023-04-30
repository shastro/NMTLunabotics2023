#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h>

#include <motor_bridge/*.h>
#include <std_msgs/String.h>

#include "can/david.h"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "can0"
static SocketCAN can;

// Enums fore eou
enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

// Last message sent to check if message has changed
motor_bridge::System lastMsg;

std::ostream& operator<<(std::ostream& os, const motor& m);
std::ostream& operator<<(std::ostream& os, const direction& d);
void estopCallback(const motor_bridge::System::ConstPtr &msg);
void pitchCallback(const motor_bridge::System::ConstPtr &msg);
void locoCallback(const motor_bridge::System::ConstPtr &msg);
void excavCallback(const motor_bridge::System::ConstPtr &msg);
void steppCallback(const motor_bridge::System::ConstPtr &msg);
void callback(const motor_bridge::System::ConstPtr &msg);

// Topic Handlers
ros::NodeHandle nh;
ros::Subscriber sub;

int main(int argc, char **argv) {
    try {
        can = SocketCAN(CAN_BUS);

        ros::init(argc, argv, "motor_bridge");
        ROS_INFO("motor_bridge started");
        sub = nh.subscribe("system", 5, callback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        ROS_ERROR(err);
        return 1;
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

void callback(const motor_bridge::System::ConstPtr &msg) {
    estopCallback(msg);
    pitchCallback(msg);
    excavCallback(msg);
    locoCallback(msg);
    steppCallback(msg);
    lastMsg = *msg;
}

void estopCallback(const motor_bridge::System::ConstPtr &msg) {
    if (msg->estop == lastMsg.estop) { return; }

    uint8_t message[8];
    int can_id;

    if (msg->estop) {
        ROS_INFO("WE NO GO");

    } else {
        ROS_INFO("MOVING IS ALLOWED");
    }

    david_e_stop_t e = {
        .stop=msg->estop
    };
    david_e_stop_pack(message, &e, 8);
    can_id = DAVID_E_STOP_FRAME_ID;

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("estop send failed: " << err);
    }
}

void pitchCallback(const motor_bridge::System::ConstPtr &msg) {
    if (msg->pitch == lastMsg.pitch) { return; }
    lastPitchDir = dir;

    direction dir = (direction)msg->pitch.direction;
    ROS_INFO_STREAM("Pitch Message Received. dir: " << dir
            << " home: " << msg->pitch.home);

    uint8_t message[8];
    int can_id = DAVID_PITCH_CTRL_FRAME_ID;
    david_pitch_ctrl_t pitch = {
        .home = (uint8_t)msg->pitch.home,
        .direction = (uint8_t)msg->pitch.direction
    };
    david_pitch_ctrl_pack(message, &both, 8);

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("pitch send failed: " << err);
    }
}

void locoCallback(const motor_bridge::System::ConstPtr &msg) {
    int lspeed = msg->left.rpm;
    int rspeed = msg->right.rpm;
    if (lspeed == lastLSpeed && rspeed == lastRSpeed) { return; }
    lastLSpeed = lspeed;
    lastRSpeed = rspeed;

    ROS_INFO_STREAM("Drive Message Received."
            << " left motor: " << lspeed
            << ", right motor: " << rspeed);

    uint8_t message[8];
    int can_id;
    david_loco_ctrl_t left = {
        .left_vel = (uint64_t)lspeed,
        .right_vel = (uint64_t)rspeed
    };
    david_loco_ctrl_pack(message, &left, 8);
    can_id = DAVID_LOCO_CTRL_FRAME_ID;
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("loco send failed: " << err);
    }
}

void excavCallback(const motor_bridge::System::ConstPtr &msg) {
    direction d = (direction)msg->digger.dir;
    if (d == lastExcavDir) { return; }
    lastExcavDir = d;

    ROS_INFO_STREAM("Digger Message Received."
            << "dir: " << d);

    uint8_t message[8];
    int can_id;

    david_excav_ctrl_t e = {
        .dir = (uint64_t)d
    };
    david_excav_ctrl_pack(message, &e, sizeof(message));
    can_id = DAVID_EXCAV_CTRL_FRAME_ID;

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("excav send failed: " << err);
    }
}

void steppCallback(const motor_bridge::System::ConstPtr &msg) {
    direction d = (direction)msg->extend.direction;
    int rpm = msg->extend.rpm;
    if (d == lastSteppDir && rpm == lastSteppRpm) { return; }
    lastSteppDir = d;
    lastSteppRpm = rpm;

    ROS_INFO_STREAM("Stepp Message Received. "
            << " direction: " << d
            << " rmp: " << rpm);

    uint8_t message[8];
    int can_id = DAVID_STEPPER_CTRL_FRAME_ID;

    david_stepper_ctrl_both_t both = {
        .rpm = (uint32_t)rpm,
        .direction = (uint32_t)d,
    };
    david_stepper_ctrl_pack(message, &both, sizeof(message));

    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("stepp failed: " << err);
    }
}
