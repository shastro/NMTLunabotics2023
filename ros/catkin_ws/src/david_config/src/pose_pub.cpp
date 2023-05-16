#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_pub;

void Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose= msg->pose;
    pose_pub.publish(pose_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/camera/odom/sample", 10, Callback);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_with_covariance", 10);
    ros::spin();

    return 0;
}

