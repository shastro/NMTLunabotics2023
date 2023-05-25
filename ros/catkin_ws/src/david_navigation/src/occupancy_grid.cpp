#include <iostream>
#include <ros/ros.h>
#include <ros/param.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

ros::Publisher pub;
ros::NodeHandle* nh;

std::string layer = "elevation";

void callback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map, msg->layers);

    float rate;
    float threshold;
    float power;
    nh->param<float>("/rate", rate, 0.1f);
    nh->param<float>("/threshold", threshold, 0.55f);
    nh->param<float>("/power", power, 0.4f);

    float final_threshold = powf(threshold, 1.0/power);
    
    float min = 100.0;
    float max = -100.0;

    grid_map::Matrix mx = map.get(layer);
    int nRows = mx.rows();
    int nCols = mx.cols();
    int sz = map.getSize().prod();
    nav_msgs::OccupancyGrid grid;
    grid.data.resize(sz);

    float median = 0;

    for (int j = 0; j < nCols; j++) {
        for (int i = 0; i < nRows; i++) {
            float val = mx.coeff(i,j);
            if (val < min) {
                min = val;
            }
            if (val > max) {
                max = val;
            }

#define signum(x) ((x > 1)? 1 : ((x < -1)? -1 : x));
#define sign(x) ((x > 1) - (x < -1))
            if (val != 0) {
                median += rate * sign(val - median);
            }
        }
    }

    float range = max-min;
    float norm_median = median / range;

    for (int j = 1; j < nCols-1; j++) {
        for (int i = 1; i < nRows-1; i++) {
            float filter_val = (2.0*mx.coeff(i, j) + mx.coeff(i-1, j) + mx.coeff(i+1, j) + mx.coeff(i, j-1) + mx.coeff(i, j+1) )/6.0;
            float norm_val = (filter_val - min)/range;
            norm_val = (isnan(norm_val))? norm_median : norm_val;
            float diff = norm_val - norm_median;
            // grid.data[(nRows-1-i) + nRows*(nCols-1-j)] = 100.0*(diff > final_threshold);
            grid.data[(nRows-1-i) + nRows*(nCols-1-j)] = 100.0*abs(diff);
            // grid.data[(nRows-1-i) + nRows*(nCols-1-j)] = isnan(norm_val)? 53 : 100.0*abs(norm_val);
        }
    }

    // #define threshold 0.5
    // for (int j = 0; j < nCols; j++) {
    //     for (int i = 0; i < nRows; i++) {
    //         float xgrad = (((i+1 == nRows)? 0 : mx.coeff(i+1, j)) - ((i-1 == -1)? 0 : mx.coeff(i-1, j)))/(max-min);
    //         float ygrad = ((j+1 == nRows)? 0 : mx.coeff(i, j+1)) - ((j-1 == -1)? 0 : mx.coeff(i, j-1))/(max-min);
    //         float val  = 1.0 - xgrad*xgrad + ygrad*ygrad;
    //         grid.data[i + nRows*j] = 100*val;
    //         // grid.data[i + nRows*j] = (val > threshold) 100 : 0;
    //     }
    // }
    
    // // Centers
    // #define threshold 0.5
    // for (int j = 1; j < nCols-1; j++) {
    //     for (int i = 1; i < nRows-1; i++) {
    //         float xgrad = (mx.coeff(i+1, j) - mx.coeff(i-1, j))/2.0*(max-min);
    //         float ygrad = (mx.coeff(i, j+1) - mx.coeff(i, j-1))/2.0*(max-min);
    //         float val  = 1.0 - xgrad*xgrad + ygrad*ygrad;
    //         grid.data[(nRows-1-i) + nRows*(nCols-1-j)] = val * 100.0; //(val > threshold)? 100 : 0;
    //     }
    // }

    // // Boundaries
    // for (int i = 1; i < nRows-1; i++) {
    //     float grad = (mx.coeff(i, 1) - mx.coeff(i, 0))/(max-min);
    //     float val  = 1.0 - grad*grad;
    //     // grid.data[i + nRows*0] = (val > threshold)? 100 : 0;
    //     grid.data[i + nRows*0] = 100*val;
    // }

    // for (int i = 1; i < nRows-1; i++) {
    //     float grad = (mx.coeff(i, nCols-1) - mx.coeff(i, nCols-2))/(max-min);
    //     float val  = 1.0 - grad*grad;
    //     // grid.data[i + nRows*(nCols-1)] = (val > threshold)? 100 : 0;
    //     grid.data[i + nRows*(nCols-1)] = 100.0*val;
    // }

    // for (int j = 0; j < nCols; j++) {
    //     float grad = (mx.coeff(1, j) - mx.coeff(0, j))/(max-min);
    //     float val  = 1.0 - grad*grad;
    //     // grid.data[0 + nRows*j] = (val > threshold)? 100 : 0;
    //     grid.data[0 + nRows*j] = 100.0*val;
    // }

    // for (int j = 0; j < nCols; j++) {
    //     float grad = (mx.coeff(nRows-1, j) - mx.coeff(nRows-2, j))/(max-min);
    //     float val  = 1.0 - grad*grad;
    //     // grid.data[nRows-1 + nRows*j] = (val > threshold)? 100 : 0;
    //     grid.data[nRows-1 + nRows*j] = 100.0*val;
    // }

    grid.header.frame_id = map.getFrameId();
    grid.header.stamp.fromNSec(map.getTimestamp());
    grid.info.map_load_time = grid.header.stamp;  // Same as header stamp as we do not load the map.
    grid.info.resolution = map.getResolution();
    grid.info.width = map.getSize()(0);
    grid.info.height = map.getSize()(1);
    grid_map::Position position = map.getPosition() - 0.5 * map.getLength().matrix();
    grid.info.origin.position.x = position.x();
    grid.info.origin.position.y = position.y();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    pub.publish(grid);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "occupancy_grid");
    ros::NodeHandle nodehandle;
    nh = &nodehandle;

    ros::Subscriber sub =
        nh->subscribe("/elevation_mapping/elevation_map", 2, callback);
    pub = nh->advertise<nav_msgs::OccupancyGrid>("/map", 2);

    ros::spin();

    return 0;
}
