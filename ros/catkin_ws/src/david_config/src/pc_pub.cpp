#include <iostream>
#include <ros/param.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

class PosePub {
    ros::Publisher pub;

  public:
    PosePub(ros::NodeHandle &nh, std::string topic)
        : pub(nh.advertise<sensor_msgs::PointCloud>(topic, 10)) {}

    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        sensor_msgs::PointCloud new_msg;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, new_msg);
        pub.publish(new_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh;

    std::vector<std::string> names;
    ros::param::getParamNames(names);

    // Hey beanie check out this fucked shit
    std::vector<std::pair<ros::Subscriber, std::unique_ptr<PosePub>>> subs;
    for (auto &name : names) {
        if (name.find("pc_pub") != std::string::npos) {
            std::string t_sub;
            ros::param::get(name, t_sub);
            std::string t_pub =
                t_sub.substr(0, t_sub.find('/')) + "/pointcloud";
            std::cout << ">> " << t_pub << std::endl;

            std::unique_ptr<PosePub> pub(new PosePub(nh, t_pub));
            subs.push_back(std::make_pair(
                nh.subscribe(t_sub, 10, &PosePub::callback, &*pub),
                std::move(pub)));
        }
    }

    ros::spin();

    return 0;
}
