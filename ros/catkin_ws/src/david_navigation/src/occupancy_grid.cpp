#include <iostream>
#include <ros/ros.h>
#include <ros/param.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher pub;
std::string layer = "elevation";

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

    grid_map::Matrix mx;
    int nRows = mx.rows();
    int nCols = mx.cols();
    nav_msgs::OccupancyGrid grid;
    grid.data.resize(nRows*nCols);
    
    for (int j = 0; j < nCols; j++) {
        for (int i = 0; i < nRows; i++) {
            float xgrad = (((i+1 == nRows)? 0 : mx.coeff(i+1, j)) - ((i-1 == -1)? 0 : mx.coeff(i-1, j)))/(max-min);
            float ygrad = ((j+1 == nRows)? 0 : mx.coeff(i, j+1)) - ((j-1 == -1)? 0 : mx.coeff(i, j-1))/(max-min);
            float val  = xgrad*xgrad + ygrad*ygrad;
            grid.data[i + nRows*j] = (100*val);
        }
    }
    
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
