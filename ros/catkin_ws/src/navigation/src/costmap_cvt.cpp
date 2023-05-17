#include <ros/ros.h>
#include <grid_map/GridMap.h>
#include <grid_map_costmap_2d/Costmap2DConverter.hppz>

ros::Publisher pub;

void callback(grid_map::GridMap::ConstPtr& grid_map) {
    // Largest min possible and smallest max possible
    float g_map_min;
    float g_map_max;
    ros::Param::get("ignore_points_above", g_map_min);
    ros::Param::get("ignore_points_below", g_map_max);
    
    // Max slope
    float max_slope;
    ros::Param::get("max_slope", max_slope);

    // Layer from gridmap to get
    std::string layer = "elevation";

    costmap_2d::Costmap2D costmap;
    costmap.resizeMap(grid_map.info.length_x, grid_map.info.length_y,
            grid_map.info.resolution, grid_map.info.pose.position.x,
            grid_map.info.pose.position.y);
    setCostmap2DFromGridMap(*grid_map, layer, costmap);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_cvt");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/elevation_mapping/elevation_map",
            5, callback);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 5);
    ros::Spin();
}
