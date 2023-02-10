#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor_bridge");

  std::cout << "motor_bridge initialized" << std::endl;
  ros::NodeHandle nh;
  ros::Publisher publisher =
      nh.advertise<std_msgs::String>("/publisher_test", 1000);

  while (1) {
    std_msgs::String s;
    s.data = "hello";
    publisher.publish(s);
    sleep(1);
  }
}
