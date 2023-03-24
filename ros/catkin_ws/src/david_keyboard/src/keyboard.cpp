#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <motor_bridge/Pitch.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "david_keyboard");

    std::cout << "david keyboard initialized" << std::endl;
    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<motor_bridge::Pitch>("pitch_control", 1000);

    ros::Rate rate(10);
    motor_bridge::Pitch p;

    int max = 1024;
    int min = 0;
    char key;
    int l = 0;

    while (ros::ok()) {
        key = getchar();

        if (key == '[') {
            l -= 64;
            l = (l < min) ? min : l;
            std::cout << "<<: " << l << std::endl;
        } else if (key == ']') {
            l += 64;
            l = (l > max) ? max : l;
            std::cout << ">>: " << l << std::endl;
        } else {
            std::cout << "Invalid key pressed. Please press [ or ]." << std::endl;
        }

        p.length = l;
        pub.publish(p);
    }
}
