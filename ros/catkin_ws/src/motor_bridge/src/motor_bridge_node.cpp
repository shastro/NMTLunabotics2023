#include <iostream>

#include <ros/ros.h>
#include <unistd.h>

#include <std_msgs/String.h>

#include <motor_bridge/Pitch.h>

#include "can/david.h"

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

void pitchCallback(const motor_bridge::Pitch::ConstPtr& msg) {
    int length = msg->length;
    std::cout << "Pitch Message Received. length: " << length << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_bridge");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("pitch_control", 1024, pitchCallback);

  // Callback event loop
  ros::spin();
  
  return 0;
}
