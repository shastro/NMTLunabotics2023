#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <motor_bridge/Pitch.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "david_keyboard");

  std::cout << "david keyboard initialized" << std::endl;
  std::cout << "Please don't type anything other than 0-9\n
                I'm not cleaning any input" << std::endl;
  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<std_msgs::String>("/pitch_control", 1000);
  
  ros::Rate rate(10);
  motor_bridge::Pitch p;

  int max = 1024;
  while (ros::ok()) {
    int l;
    cin >> l;
    l = l < 0 ? 0 : l;
    l = l > 9 ? 9 : l;
    l = (l / 9) * max;
    p.length = l;
    pub.publish(msg);
  }
}
