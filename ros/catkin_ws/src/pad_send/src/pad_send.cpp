#include <iostream>
#include <fcntl.h>
#include <motor_bridge/System.h>
#include <ros/ros.h>
#include "joystick/gamepad.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pad_send");
    ros::NodeHandle nh;
    int max = 1024;
    int min = -max;
    int dead = 200;
    ros::Publisher pub = nh.advertise<motor_bridge::System>("/system", 5);
    int fd, num_of_joysticks, joynum, max_joy_index = -1;
    char name_of_joy[80];
    char controller_input[80];
    num_of_joysticks = 0;
    for (joynum = 0; joynum < 16; joynum++)
    {
        sprintf(name_of_joy, "/dev/input/js%d", joynum);
        fd = open(name_of_joy, O_RDONLY);
        if (fd >= 0)
        {
            num_of_joysticks++;
            if (joynum > max_joy_index)
                max_joy_index = joynum;
            close(fd);
        }
    }
    sprintf(controller_input, "/dev/input/js%d", max_joy_index);
    printf("%s", controller_input);
    GamepadHandler g(controller_input, max, dead);
    motor_bridge::System s;
    bool going = true;
    bool estopped = false;
    int count = 0;
    while (going) {
        g.update();
        if (count < 100)
            count++;

        std::cout << g << std::endl;

        if (g.buttons.xbox && count > 50) {
            s.estop = true;
            going = false;
        }

        // Estop
        if (g.buttons.A)
            estopped = true;
        if (g.buttons.B)
            estopped = false;
        s.estop = estopped;

        // Drive left stick
        /*
        int left = g.left_stick.x + g.left_stick.y;
        int right = g.left_stick.y - g.left_stick.x;
        left = (left > max) ? max : left;
        left = (left < min) ? min : left;
        right = (right > max) ? max : right;
        right = (right < min) ? min : right;
        s.left.rpm = left;
        s.right.rpm = right;
        */

        // Drive both sticks
        s.left.rpm = g.left_stick.y;
        s.right.rpm = g.right_stick.y;

        // Pitch dpad up/down
        if (g.dpad.up)
            s.pitch.direction = 1;
        else if (g.dpad.down)
            s.pitch.direction = 2;
        else
            s.pitch.direction = 0;
        s.pitch.motor = 0;

        // Extend bumpers
        s.extend.direction = g.buttons.left_bumper ? 2 : 0;
        s.extend.direction = g.buttons.right_bumper ? 1 : s.extend.direction;
        s.extend.rpm = (s.extend.direction != 0) ? max : 0;
        s.extend.motor = 0;

        // Digger triggers
        s.digger.rpm = g.right_trigger;
        s.digger.rpm = (g.left_trigger > 0) ? -g.left_trigger : s.digger.rpm;

        pub.publish(s);
    }
    return 0;
}
