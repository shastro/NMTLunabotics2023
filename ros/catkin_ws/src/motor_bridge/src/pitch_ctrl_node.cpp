#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/Pitch.h>

#include "can/david.h"

enum motor {
    BOTH = 0,
    LEFT = 1,
    RIGHT = 2
};

static struct {
    const int HOME_LENGTH = 0;
    const int MAX_LENGTH = 1024;
    int target_length; // set by callback.
    int actual_length; // actual length as estimated by telemetry.
    const int FORWARD = 0;
    const int BACKWARD = 1;
    int motor_direction;
    int motor_on;
} PitchMotorState;

void pitchCallback(const motor_bridge::Pitch::ConstPtr &msg) {
    int length = msg->length;
    motor m = msg->motor;
    std::cout << "Pitch Message Received. length: "
        << length << " motor: " << m << std::endl;

    uint8_t message[8];

    if (m = LEFT) {
        david_pitch_ctrl_left_t left = {
            .count = (uint64_t)length,
            .tolerance = 100,
        }
        david_pitch_ctrl_left_pack(message, &left, sizeof(message));
    } else if (m = RIGHT) {
        david_pitch_ctrl_right_t right = {
            .count = (uint64_t)length,
            .tolerance = 100,
        }
        david_pitch_ctrl_right_pack(message, &right, sizeof(message));
    } else {
        david_pitch_ctrl_both_t both = {
            .count = (uint64_t)length,
            .tolerance = 100,
        }
        david_pitch_ctrl_both_pack(message, &both, sizeof(message));
    }

    //TODO: send message
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pitch_ctrl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pitch_control", 1024, pitchCallback);

    // Callback event loop
    ros::spin();

    return 0;
}
