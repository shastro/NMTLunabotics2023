#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/point_cloud_conversion.h>

class PosePub {
    ros::Publisher pub;

public:
    PosePub(ros::NodeHandle &nh, std::string topic) :
        pub(nh.advertise<sensor_msgs::PointCloud>(topic, 10)) {}

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud new_msg;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, new_msg);
        pub.publish(new_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh;
    
    std::vector<std::string> names;
    ros::param::getParamNames(names);
    
    // Hey beanie check out this fucked shit
    for (auto &name : names) {
        std::string t_sub;
        ros::param::get(name, topic)
        std::string t_pub = t_sub.substr(0, t_pub.find("/", 1));
        t_pub += "/pointcloud";
        
        PosePub* pub = new PosePub(nh, topic);
        ros::Subscriber* sub = new ros::Subscriber(
            nh.subscribe(topic, 10, &PosePub::callback, pub));
    }

    ros::spin();

    return 0;
}
