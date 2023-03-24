#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <motor_bridge/Pitch.h>

enum motor {
    BOTH = 0,
    LEFT = 1,
    RIGHT = 2
};

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
    motor m = BOTH;

    while (ros::ok()) {
        key = std::cin.get();
        std::cin.ignore(100, '\n');

        switch (key) {
            case '[':
                l -= 64;
                l = (l < min) ? min : l;
                std::cout << "<<: " << l << std::endl;
                break;
            case ']':
                l += 64;
                l = (l > max) ? max : l;
                std::cout << ">>: " << l << std::endl;
                break;
            case '{':
                l = 0;
                std::cout << "<<: " << l << std::endl;
                break;
            case '}':
                l = 1024;
                std::cout << ">>: " << l << std::endl;
                break;
            case 'b':
                m = BOTH;
                break;
            case 'l':
                m = LEFT;
                break;
            case 'r':
                m = RIGHT;
                break;
            default:
                std::cout << "Invalid key pressed" << std::endl;
                break;
        }

        p.length = l;
        p.motor = m;
        pub.publish(p);
    }
}
