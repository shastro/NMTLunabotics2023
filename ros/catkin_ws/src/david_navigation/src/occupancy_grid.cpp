#include <iostream>
#include <ros/ros.h>
#include <ros/param.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher pub;
std::string layer = "traversability";

void callback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map, msg->layers);
    
    int min = 100;
    int max = -100;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        float val = map.at(layer, *it);
        if (val < min) {
            min = val;
        }
        if (val > max) {
            max = val;
        }
    }
    
    nav_msgs::OccupancyGrid grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, layer, min, max, grid);
    
    pub.publish(grid);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "occupancy_grid");
    ros::NodeHandle nh;

    ros::Subscriber sub =
        nh.subscribe("/elevation_mapping/elevation_map_raw", 2, callback);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 2);

    ros::spin();

    return 0;
}
