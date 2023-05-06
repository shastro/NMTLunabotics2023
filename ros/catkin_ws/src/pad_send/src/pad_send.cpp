#include <iostream>
#include <sys/stat.h>
#include <ncurses.h>
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

    // Setup ncurses
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();
    halfdelay(1);

    // Configure joystick
    char name_of_joy[80];
    char controller_input[80];
    int joynum = 16;
    bool found = false;
    struct stat buf;
    while (joynum >= 0 && !found) {
        joynum--;
        sprintf(name_of_joy, "/dev/input/js%d", joynum);
        int fd = stat(name_of_joy, &buf);
        if (fd >= 0) {
            found = true;
        }
    }
    sprintf(controller_input, "/dev/input/js%d", joynum);
    GamepadHandler g(controller_input, max, dead);

    // Message loop
    motor_bridge::System s;
    bool going = true;
    bool estopped = false;
    int count = 0;
    bool lastX = false;
    bool lastY = false;
    bool lastA = false;
    bool lastB = false;
    bool home = false;


    while (going) {
        g.update();
        if (count < 10) {
            count++;
            continue;
        }

        std::stringstream out;

        // Control scheme
        out << controller_input << "\n";
        out << "A - Toggle Estop, X - toggle home, XBOX - quit\n";
        out << "Left stick - left tread, right stick - right tread\n";
        out << "Dpad: up/down - pitch\n";
        out << "Bumpers: left - arm extend, right - arm retract\n";

        //std::cout << g << std::endl;

        // Estop
        if (g.buttons.A && ! lastA) {
            estopped = !estopped;
        }

        if (g.buttons.xbox) {
            estopped = true;
            going = false;
        }
        s.e_stop.stop = estopped;
        if (estopped)
            out << "Estopped\n";

        // Home mode
        if (g.buttons.X && !lastX) {
            home = !home;
        }
        if (home)
            out << "Homing\n";

        // Drive with both sticks
        s.loco_ctrl.left_vel = g.left_stick.y;
        s.loco_ctrl.right_vel = g.right_stick.y;
        if (s.loco_ctrl.left_vel != 0 && s.loco_ctrl.right_vel != 0) {
            out << "Driving - L: " << s.loco_ctrl.left_vel
                << ", R: " << s.loco_ctrl.right_vel << "\n";
        }

        // Pitch control with dpad
        s.pitch_ctrl.home = (int)home;
        if (g.dpad.up) {
            s.pitch_ctrl.direction = 1;
            out << "Pitch Up\n";
        } else if (g.dpad.down) {
            s.pitch_ctrl.direction = 2;
            out << "Pitch Down\n";
        } else {
            s.pitch_ctrl.direction = 0;
        }

        // Extend bumpers
        s.stepper_ctrl.rpm = 1024;
        s.stepper_ctrl.home = (int)home;
        if (g.buttons.left_bumper) {
            s.stepper_ctrl.direction = 1;
            out << "Arm Extend\n";
        } else if (g.buttons.right_bumper) {
            s.stepper_ctrl.direction = 2;
            out << "Arm Retract\n";
        } else {
            s.stepper_ctrl.direction = 0;
            s.stepper_ctrl.rpm = 0;
        }

        // Digger triggers
        if (g.left_trigger > 0) {
            s.excav_ctrl.direction = 1;
            out << "Dig Forward\n";
        } else if (g.right_trigger > 0) {
            s.excav_ctrl.direction = 2;
            out << "Dig Backward\n";
        } else {
            s.excav_ctrl.direction = 0;
        }

        // Update last state
        lastX = g.buttons.X;
        lastY = g.buttons.Y;
        lastA = g.buttons.A;
        lastB = g.buttons.B;

        // Print
        clear();
        move(0, 0);
        printw(out.str().c_str());
        refresh();

        pub.publish(s);
    }
    return 0;
}
