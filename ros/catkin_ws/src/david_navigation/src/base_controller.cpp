#include <geometry_msgs/Twist.h>
#include <motor_bridge/LocoCtrl.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

ros::Publisher base_pub;

// Radian per second of velocity to percent. Need to measure this.
#define RAD_PER_SEC_TO_PERCENT(rps) ((rps)*0.5)

// Radius of each wheel.
#define WHEEL_RADIUS 30

// Width of the robot; specifically the distance from the center of
// the left tread to the center of the right tread.
#define ROBOT_WIDTH 49.5

void cmd_vel_callback(const geometry_msgs::Twist &msg) {
    // From https://wiki.ros.org/navigation/Tutorials/RobotSetup,
    // §1.5:
    //
    // > This means there must be a node subscribing to the "cmd_vel"
    // > topic that is capable of taking (vx, vy, vtheta) <==>
    // > (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)
    // > velocities and converting them into motor commands to send to
    // > a mobile base.

    double rad_per_second = 5.43144; // rad/s at 99% power
    double rps_per_vel = rad_per_second/ 99; // rad/s per power percentage
    
    double v = msg.linear.x;
    double omega = msg.angular.z;

    // Using formulas from https://wiki.ros.org/diff_drive_controller.
    double omega_l = (v - omega * ROBOT_WIDTH / 2) / WHEEL_RADIUS;
    double omega_r = (v + omega * ROBOT_WIDTH / 2) / WHEEL_RADIUS;

    motor_bridge::LocoCtrl loco_msg;
    loco_msg.left_vel = omega_l / rps_per_vel;
    loco_msg.right_vel = omega_r / rps_per_vel;

    base_pub.publish(loco_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller");

    ros::NodeHandle handle;

    base_pub =
        handle.advertise<motor_bridge::LocoCtrl>("system/loco_ctrl", 1000);
    ros::Rate loop_rate(10);

    ros::Subscriber cmd_sub =
        handle.subscribe("/cmd_vel", 1000, cmd_vel_callback);
}
