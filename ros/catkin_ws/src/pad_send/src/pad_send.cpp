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

    // Get joystick last plugged in
    char name_of_joy[80];
    char controller_input[80];
    int joynum = 16;
    bool found = false;
    while (joynum >= 0 && !found) {
        joynum--;
        sprintf(name_of_joy, "/dev/input/js%d", joynum);
        int fd = open(name_of_joy, O_RDONLY);
        if (fd >= 0) {
            found = true;
        }
    }
    sprintf(controller_input, "/dev/input/js%d", joynum);
    std:: cout << controller_input << std::endl;
    GamepadHandler g(controller_input, max, dead);

    std::stringstream ctrls;
    ctrls << "A - Estop, B - Un Estop, XBOX - quit\n";
    ctrls << "Left stick - left tread, right stick - right tread\n";
    ctrls << "Dpad: up/down - pitch up/down\n";
    ctrls << "Dpad + X: up/down - home pitch up/down\n";
    ctrls << "Bumpers: left - arm extend, right - arm retract\n";
    ctrls << "Bumpers + X: left - arm extend full, right - arm retract full\n";
    std::cout << ctrls.str();

    // Get input and create message
    motor_bridge::System s;
    bool going = true;
    bool estopped = false;
    int count = 0;
    while (going) {
        g.update();
        if (count < 50) {
            count++;
            continue;
        }

        // Adjust mode
        if (g.buttons.X) {
            if (g.dpad.left) {
                std::cout << "Adjust left pitch" << std::endl;
            } else if (g.dpad.right) {
                std::cout << "Adjust right pitch" << std::endl;
            }

            if (g.buttons.left_bumper) {
                std::cout << "Adjust left stepper" << std::endl;
            } else if (g.buttons.right_bumper) {
                std::cout << "Adjust right stepper" << std::endl;
            }
        }
                

        //std::cout << g << std::endl;

        // Estop
        /*
        if (g.buttons.A)
            estopped = true;
        if (g.buttons.B)
            estopped = false;
        if (g.buttons.xbox) {
            estopped = true;
            going = false;
        }
        s.e_stop.stop = estopped;

        // Drive with both sticks
        s.loco_ctrl.left_vel = g.left_stick.y;
        s.loco_ctrl.right_vel = g.right_stick.y;

        // Pitch control with dpad
        if (g.dpad.up) {
            s.pitch_ctrl.home = 0;
            s.pitch_ctrl.direction = 1;
        } else if (g.dpad.down) {
            s.pitch_ctrl.home = 0;
            s.pitch_ctrl.direction = 2;
        } else if (g.dpad.up && g.buttons.X) {
            s.pitch_ctrl.home = 1;
            s.pitch_ctrl.direction = 1;
        } else if (g.dpad.down && g.buttons.X) {
            s.pitch_ctrl.home = 1;
            s.pitch_ctrl.direction = 2;
        } else {
            s.pitch_ctrl.home = 0;
            s.pitch_ctrl.direction = 0;
        }

        // Extend bumpers
        s.stepper_ctrl.rpm = 1024;
        if (g.buttons.left_bumper) {
            s.stepper_ctrl.home = 0;
            s.stepper_ctrl.direction = 1;
        } else if (g.buttons.right_bumper) {
            s.stepper_ctrl.home = 0;
            s.stepper_ctrl.direction = 2;
        } else if (g.buttons.left_bumper && g.buttons.X) {
            s.stepper_ctrl.home = 1;
            s.stepper_ctrl.direction = 1;
        } else if (g.buttons.right_bumper && g.buttons.X) {
            s.stepper_ctrl.home = 1;
            s.stepper_ctrl.direction = 2;
        } else {
            s.stepper_ctrl.home = 0;
            s.stepper_ctrl.direction = 0;
            s.stepper_ctrl.rpm = 0;
        }

        // Digger triggers
        if (g.left_trigger > 0) {
            s.excav_ctrl.direction = 1;
        } else if (g.right_trigger > 0) {
            s.excav_ctrl.direction = 2;
        } else {
            s.excav_ctrl.direction = 0;
        }
        pub.publish(s);
        */
    }
    return 0;
}
