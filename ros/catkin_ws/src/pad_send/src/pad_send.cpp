#include <iostream>
#include <pad_send/Gamepad.h>
#include <ros/ros.h>
#include "joystick/gamepad.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "david_joystick");
    ros::NodeHandle nh;
    int max = 1024;
    int dead = 200;
    ros::Publisher pub = nh.advertise<pad_send::Gamepad>("/gamepad", 5);
    GamepadHandler gh("/dev/input/js0", max, dead);
    pad_send::Gamepad g;
    bool going = true;
    while (going) {
        gh.update();

        g.max = max;

        g.left_stick_x = gh.left_stick.x;
        g.left_stick_y = gh.left_stick.y;
        g.right_stick_x = gh.right_stick.x;
        g.right_stick_y = gh.right_stick.y;

        g.left_trigger = gh.left_trigger;
        g.right_trigger = gh.right_trigger;

        g.dpad_left = gh.dpad.left;
        g.dpad_right = gh.dpad.right;
        g.dpad_up = gh.dpad.up;
        g.dpad_down = gh.dpad.down;

        g.A = gh.buttons.A;
        g.B = gh.buttons.B;
        g.X = gh.buttons.X;
        g.Y = gh.buttons.Y;
        g.left_bumper = gh.buttons.left_bumper;
        g.right_bumper = gh.buttons.right_bumper;
        g.left_extra = gh.buttons.left_extra;
        g.right_extra = gh.buttons.right_extra;
        g.xbox = gh.buttons.xbox;
        g.left_thumb = gh.buttons.left_thumb;
        g.right_thumb = gh.buttons.right_thumb;

        pub.publish(g);
        //if (gh.buttons.left_extra)
        //    going = false;
    }
    return 0;
}
