#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <unistd.h>
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "webots_pose");

    std::cout << "webots_pose initialized" << std::endl;
    ros::NodeHandle nh;
    ros::Publisher publisher = 
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1000);

    ros::ServiceClient self_client = nh.serviceClient<webots_ros::node_uint64>("david_sim/supervisor/get_self");
    webots_ros::get_uint64 self_id;
    self_id.request.ask = true;
    if (self_client.call(self_id)) {
        std::stringstream s;
        s << "Self ID: " << self_id.response.value;
        ROS_INFO(s.str());
    } else {
        ROS_ERROR("Failed service call david_sim/supervisor/get_self");
        return 1;
    }

    ros::ServiceClient selected_client = nh.serviceClient<webots_ros::node_uint64>("david_sim/supervisor/get_selected");
    webots_ros::get_uint64 selected;
    self.request.ask = true;
    if (selected_client.call(selected)) {
        std::stringstream s;
        s << "Self ID: " << selected.response.value;
        ROS_INFO(s.str());
    } else {
        ROS_ERROR("Failed service call david_sim/supervisor/get_selected");
        return 1;
    }
    
    ros::ServiceClient pose_client = nh.serviceClient<webots_ros::node_get_pose>("david_sim/supervisor/node/get_pose");
    webots_ros::node_get_pose webots_pose;
    webots_pose.request.node = self_id.value;
    webots_pose.request.from_node = selected.value;
    if (pose_client.call(webots_pose)) {
        ROS_INFO("Pose called successfully");
    } else {
        ROS_ERROR("Failed service call david_sim/supervisor/node/get_pose");
        return 1;
    }
    
    while (1) {
        geometry_msgs::PoseWithCovarianceStamped p;
        p.header.frame_id = "Webots Pose";
        p.header.stamp = ros::Time::now();
        
        //Todo: pass service call into pose
        
        publisher.publish(p);
        sleep(1);
    }
}
