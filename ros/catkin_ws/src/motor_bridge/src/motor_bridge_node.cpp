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
        ROS_INFO("motor_bridge started");
        ros::NodeHandle nh;
        ros::Subscriber sub =
            nh.subscribe("system", 5, callback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        ROS_ERROR(err);
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
    if (lastMsg.e_stop == msg->e_stop) { return; }
    ROS_INFO_STREAM("Estop: " << msg->e_stop.stop);

    uint8_t message[8];
    int can_id = DAVID_E_STOP_FRAME_ID;

    david_e_stop_t e_stop = {
        .stop = (uint8_t)msg->e_stop.stop
    };
    david_e_stop_pack(message, &e_stop, sizeof(message));

    try {
        if (!testing) { can.transmit(can_id, message); }
    } catch (std::string err) {
        ROS_WARN_STREAM("estop failed: " << err);
    }
}

void pitchCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.pitch_ctrl == msg->pitch_ctrl) { return; }
    ROS_INFO_STREAM("Pitch Message Received."
            << "dir: " << (direction)msg->pitch_ctrl.direction
            << "home: " << msg->pitch_ctrl.home);

    uint8_t message[8];
    int can_id = DAVID_PITCH_CTRL_FRAME_ID;

    david_pitch_ctrl_t pitch = {
        .home = (uint8_t)msg->pitch_ctrl.home,
        .direction = (uint8_t)msg->pitch_ctrl.direction
    };
    david_pitch_ctrl_both_pack(message, &both, sizeof(message));

    try {
        if (!testing) { can.transmit(can_id, message); }
    } catch (std::string err) {
        ROS_WARN_STREAM("pitch failed: " << err);
    }
}

void locoCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.loco_ctrl == msg->loco_ctrl) { return; }
    ROS_INFO_STREAM("Drive Message Received."
            << " left motor: " << msg->loco_ctrl.left_vel
            << ", right motor: " << msg->loco_ctrl.right_vel);

    uint8_t message[8];
    int can_id = DAVID_LOCO_CTRL_FRAME_ID;

    david_loco_ctrl_t left = {
        .left_vel = msg->loco_ctrl.left_vel,
        .right_vel = msg->loco_ctrl.right_vel
    };
    david_loco_ctrl_pack(message, &left, sizeof(message));

    try {
        if (!testing) { can.transmit(can_id, message); }
    } catch (std::string err) {
        ROS_WARN_STREAM("left motor failed: " << err);
    }
}

void excavCallback(const motor_bridge::System::ConstPtr &msg) {
    if (lastMsg.excav_ctrl == msg->excav_ctrl) { return; }
    ROS_INFO_STREAM("Digger Message Received."
            << "dir: " << (direction)msg->excav_ctrl.direction);

    uint8_t message[8];
    int can_id;

    david_excav_ctrl_t e = {
        .direction = msg->excav_ctrl.direction
    };
    david_excav_ctrl_pack(message, &e, sizeof(message));
    can_id = DAVID_EXCAV_CTRL_FRAME_ID;

    try {
        if (!testing) { can.transmit(can_id, message); }
    } catch (std::string err) {
        ROS_WARN_STREAM("excav failed: " << err);
    }
}

void steppCallback(const motor_bridge::System::ConstPtr &msg) {
    motor m = (motor)msg->extend.motor;
    direction d = (direction)msg->extend.direction;
    int rpm = msg->extend.rpm;
    if (m == lastSteppM && d == lastSteppDir && rpm == lastSteppRpm) { return; }
    lastSteppM = m;
    lastSteppDir = d;
    lastSteppRpm = rpm;

    ROS_INFO_STREAM("Stepp Message Received. motor: " << m
            << " direction: " << d
            << " rmp: " << rpm);

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
        if (!testing) { can.transmit(can_id, message); }
    } catch (std::string err) {
        ROS_WARN_STREAM("stepp failed: " << err);
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
