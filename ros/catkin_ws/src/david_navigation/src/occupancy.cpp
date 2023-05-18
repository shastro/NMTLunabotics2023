#include <iostream>
#include <ros/param.h>
#include <ros/ros.h>
#include <grid_map_msgs::GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs::OccupancyGrid.h>

ros::Publisher pub;
std::string layer = "traversability"

void callback(grid_map::GridMap::ConstPtr& map) {
    int min;
    int max;
    ros::Params::get("ignore_points_above", min);
    ros::Params::get("ignore_points_below", max);

    for (GridMapIterator it(map); !it.isPastEnd() ++it) {
        float val = map.at(layer, *it);
        if (val < min) {
            min = val;
        }
        if (val > max) {
            max = val;
        }
    }
    
    nav_msgs::OccupancyGrid grid;
    GridMapRosConverter::toOccupancyGrid(map, layer, min, max, grid);
    pub.publish(grid);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "occupancy_grid");
    ros::NodeHandle nh;

    ros::Subscriber sub =
        nh.subscriber("elevation_mapping/elevation_mapping_raw", 2, callback);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 2);

    ros::spin();

    return 0;
}
