#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.>
#include <motor_bridge/LocoCtrl.h>
#include <sstream>
#include <geometry_msgs/Vector3.h>

ros::Publisher base_pub;

// const static double RAD_PER_SEC_TO_PERCENT = 0.5; // Need to measure this

// http://wiki.ros.org/diff_drive_controller

int main(int argc, char **argv) {
  using namespace boost::numeric::ublas;
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle handle;

  base_pub = handle.advertise<motor_bridge::LocoCtrl>("system/loco_ctrl", 1000);
  ros::Rate loop_rate(10);

  ros::Subscriber cmd_sub = handle.subscribe("/cmd_vel", 1000, cmd_vel_callback);

}

void cmd_vel_callback(const geometry_msgs::Twist &msg) {
  geometry_msgs::Vector3
}
