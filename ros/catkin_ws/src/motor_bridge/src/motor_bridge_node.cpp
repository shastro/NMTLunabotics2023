#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h>

#include <motor_bridge/System.h>

#include "can/david.h"
#include "can_interface.hpp"
#include <linux/can.h>

#define CAN_BUS "can0"
static SocketCAN can;

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

bool lastEstop;
motor lastPitchM;
direction lastPitchDir;
int lastLSpeed;
int lastRSpeed;
int lastExcavSpeed;
motor lastSteppM;
direction lastSteppDir;
int lastSteppRpm;

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
            nh.subscribe("system", 1024, callback);

        // Callback event loop
        ros::spin();
    } catch (std::string err) {
        std::cout << err << std::endl;
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

void estopCallback(const motor_bridge::System::ConstPtr &msg) {
    bool stop = msg->estop;
    /*if (stop == lastEstop) { return; }
    lastEstop = stop;*/

    uint8_t message[8];
    int can_id;

    if (stop) {
        ROS_INFO("ESTOP YES");
        david_e_stop_t estop = {};
        david_e_stop_pack(message, &estop, sizeof(message));
        can_id = DAVID_E_STOP_FRAME_ID;

    } else {
        ROS_INFO("NOT ESTOP");
        david_e_start_t estart = {};
        david_e_start_pack(message, &estart, sizeof(message));
        can_id = DAVID_E_START_FRAME_ID;
    }
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("estop failed: " << err);
    }
}

void pitchCallback(const motor_bridge::System::ConstPtr &msg) {
    direction dir = (direction)msg->pitch.direction;
    motor m = (motor)msg->pitch.motor;
    /*if (dir == lastPitchDir && m == lastPitchM) { return; }
    lastPitchDir = dir;
    lastPitchM = m;*/

    ROS_INFO_STREAM("Pitch Message Received. dir: " << dir << " motor: " << m);

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
        ROS_WARN_STREAM("pitch failed: " << err);
    }
}

void locoCallback(const motor_bridge::System::ConstPtr &msg) {
    int lspeed = msg->left.rpm;
    int rspeed = msg->right.rpm;
    /*if (lspeed == lastLSpeed && rspeed == lastRSpeed) { return; }
    lastLSpeed = lspeed;
    lastRSpeed = rspeed;*/

    ROS_INFO_STREAM("Drive Message Received."
        << " left motor: " << lspeed
        << ", right motor: " << rspeed);

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
        ROS_WARN_STREAM("left motor failed: " << err);
    }

    david_loco_ctrl_right_t right = {
        .velocity = (uint64_t)rspeed
    };
    david_loco_ctrl_right_pack(message, &right, sizeof(message));
    can_id = DAVID_LOCO_CTRL_RIGHT_FRAME_ID;
    try {
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("right motor failed: " << err);
    }
}

void excavCallback(const motor_bridge::System::ConstPtr &msg) {
    int speed = msg->digger.rpm;
    /*if (speed == lastExcavSpeed) { return; }
    lastExcavSpeed = speed;*/

    ROS_INFO_STREAM("Digger Message Received."
        << "speed: " << speed);

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
        ROS_WARN_STREAM("excav failed: " << err);
    }
}

void steppCallback(const motor_bridge::System::ConstPtr &msg) {
    motor m = (motor)msg->extend.motor;
    direction d = (direction)msg->extend.direction;
    int rpm = msg->extend.rpm;
    /*if (m == lastSteppM && d == lastSteppDir && rpm == lastSteppRpm) { return; }
    lastSteppM = m;
    lastSteppDir = d;
    lastSteppRpm = rpm;*/

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
        can.transmit(can_id, message);
    } catch (std::string err) {
        ROS_WARN_STREAM("stepp failed: " << err);
    }
}

void callback(const motor_bridge::System::ConstPtr &msg) {
    estopCallback(msg);
    pitchCallback(msg);
    excavCallback(msg);
    locoCallback(msg);
    steppCallback(msg);
}
