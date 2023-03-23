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
    while (ros::ok()) {
        int input;
        std::cout << "Enter a digit between 0 and 9: ";
        while (!(std::cin >> input) || input < 0 || input > 9 || std::cin.peek() != '\n') {
            std::cout << "Invalid input. Please enter a digit between 0 and 9: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        input =  (input * max) / 9;
        p.length = input;
        pub.publish(p);
    }
}
