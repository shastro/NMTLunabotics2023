#include <iostream>
#include <algorithm>
#include <sys/stat.h>
#include <ncurses.h>
#include <motor_bridge/System.h>
#include <ros/ros.h>
#include "joystick/gamepad.hpp"
#include "can/limits.hpp"

// The Adjustables
int max = 98;   // Max value for sticks/triggers
int min = -max; // Min val ... well yeah
int dead = 20;  // Can't be too touchy. < dead = 0
int rate = 100; //in hertz
double pitch_delta = 8.0 / rate; // in mm/s
double stepper_delta = 20.0 / rate; // in mm/s

enum class mode {
    normal,
    adjust,
    home,
    stop
};

mode m = mode::normal;

int main(int argc, char *argv[]) {
    // Set up ros
    ros::init(argc, argv, "pad_send");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<motor_bridge::System>("/system", 5);
    ros::Rate nyquil(rate);

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

    double pitch = (PITCH_CTRL_SET_POINT_MAX -
            PITCH_CTRL_SET_POINT_MIN) / 2;
    double pitch_left_offset = 0;
    double pitch_right_offset = 0;

    double stepper = 0;

    bool pitch_home = false;
    bool mast_home = false;
    bool stepper_home = false;

    bool lastX = false;
    bool lastY = false;
    bool lastA = false;
    bool lastB = false;

    int count = 0;

    while (true) {
        g.update();
        erase();
        move(0, 0);
        if (count < 50) {
            count++;
            continue;
        }

        // Quit
        if (g.buttons.xbox) {
            s.e_stop.stop = true;
            pub.publish(s);
            endwin();
            return 0;
        }

        std::stringstream out;

        // Control scheme TODO update
        out << "X - Resume, Y - Estop, A - Adjust, B - Home, XBOX - quit\n";
        out << "Cannot Adjust or Home when Estopped\n\n";

        //std::cout << g << std::endl;

        // Estop
        if (g.buttons.Y) {
            m = mode::stop;
        }

        // Adjust
        if (g.buttons.A && m != mode::stop) {
            m = mode::adjust;
        }

        // Homing
        if (g.buttons.B && m != mode::stop) {
            m = mode::home;
        }

        // Normal
        if (g.buttons.X) {
            m = mode::normal;
        }

        switch(m) {
            case mode::normal:
                // Controls
                out << "Normal Operation\n\n";
                out << "Drive: Sticks\n";
                out << "Excavate: Triggers\n";
                out << "Pitch: Dpad Up/Down\n";
                out << "Extend/Retract: Dpad Left/Right\n";
                out << "Mast: Bumpers\n";
                out << "\n";

                // Un home everyting
                pitch_home = false;
                stepper_home = false;
                mast_home = false;
                s.e_stop.stop = false;

                // Pitch control with dpad
                if (g.dpad.up) {
                    pitch += pitch_delta;
                    pitch = std::min(pitch,
                            PITCH_CTRL_SET_POINT_MAX);
                    out << "Pitch Up: " << pitch << "\n";
                } else if (g.dpad.down) {
                    pitch -= pitch_delta;
                    pitch = std::max(pitch,
                            PITCH_CTRL_SET_POINT_MIN);
                    out << "Pitch Down: " << pitch << "\n";
                }
                s.pitch_ctrl.set_point = pitch;

                // Viagra bumpers
                if (g.buttons.left_bumper) {
                    out << "david's shaft left\n";
                    s.mast_ctrl.direction = 0;
                } else if (g.buttons.right_bumper) {
                    out << "david's shaft right\n";
                    s.mast_ctrl.direction = 2;
                } else {
                    s.mast_ctrl.direction = 1;
                }

                // Drive with both sticks
                s.loco_ctrl.left_vel = g.left_stick.y;
                s.loco_ctrl.right_vel = g.right_stick.y;
                if (s.loco_ctrl.left_vel != 0 || s.loco_ctrl.right_vel != 0) {
                    out << "Driving - L: " << s.loco_ctrl.left_vel
                        << ", R: " << s.loco_ctrl.right_vel << "\n";
                }

                // Extend dpad
                if (g.dpad.left) {
                    stepper += stepper_delta;
                    stepper = std::min(stepper,
                            STEPPER_CTRL_SET_POINT_MAX);
                    out << "Arm Extend: " << stepper << "\n";
                } else if (g.dpad.right) {
                    stepper -= stepper_delta;
                    stepper = std::max(stepper,
                            STEPPER_CTRL_SET_POINT_MIN);
                    out << "Arm Retract: " << stepper << "\n";
                }
                s.stepper_ctrl.set_point = stepper;

                // Digger triggers
                if (g.left_trigger > 0) {
                    s.excav_ctrl.vel = -g.left_trigger;
                    out << "Dig Forward: " << s.excav_ctrl.vel << "\n";
                } else if (g.right_trigger > 0) {
                    s.excav_ctrl.vel = g.right_trigger;
                    out << "Dig Backward: " << s.excav_ctrl.vel << "\n";
                } else {
                    s.excav_ctrl.vel = 0;
                }
                break;

            case mode::adjust:
                out << "Adjusting\n\n";
                out << "Left Stick - Left Pitch Arm\n";
                out << "Right Stick - Right Pitch Arm\n";
                out << "\n";

                // Pitch
                pitch_left_offset += pitch_delta * g.left_stick.y / max;
                pitch_left_offset = std::min(pitch_left_offset,
                        PITCH_CTRL_LEFT_OFFSET_MAX);
                pitch_left_offset = std::max(pitch_left_offset,
                        PITCH_CTRL_LEFT_OFFSET_MIN);
                s.pitch_ctrl.left_offset = pitch_left_offset;
                out << "Pitch Left Offset: " << pitch_left_offset << "\n";

                pitch_right_offset += pitch_delta * g.right_stick.y / max;
                pitch_right_offset = std::min(pitch_right_offset,
                        PITCH_CTRL_RIGHT_OFFSET_MAX);
                pitch_right_offset = std::max(pitch_right_offset,
                        PITCH_CTRL_RIGHT_OFFSET_MIN);
                s.pitch_ctrl.right_offset = pitch_right_offset;
                out << "Pitch right Offset: " << pitch_right_offset << "\n";
                break;

            case mode::home:
                out << "Homing\n\n";
                out << "Pitch: Dpad Up or Down\n";
                out << "Extend: Dpad Left or Right\n";
                out << "Mast: Bumper Left or Right\n";
                out << "\n";

                if (g.dpad.up || g.dpad.down) {
                    out << "Pitch Home\n";
                    pitch_home = true;
                    pitch = PITCH_CTRL_SET_POINT_MAX;
                    s.pitch_ctrl.set_point = pitch;
                }

                if (g.dpad.left || g.dpad.right) {
                    out << "Extend Home\n";
                    stepper_home = true;
                    stepper = STEPPER_CTRL_SET_POINT_MIN;
                    s.stepper_ctrl.set_point = stepper;
                }

                if (g.buttons.left_bumper || g.buttons.right_bumper) {
                    out << "Mast Home\n";
                    mast_home = true;
                    s.mast_ctrl.direction = 1;
                }
                break;

            case mode::stop:
                out << "Contemplating Life\n";
                s.e_stop.stop = true;
                break;

            default:
                break;
        }

        // Sending homing messages
        s.pitch_ctrl.home = pitch_home;
        s.stepper_ctrl.home = stepper_home;
        s.mast_ctrl.home = mast_home;

        // Update last state
        lastX = g.buttons.X;
        lastY = g.buttons.Y;
        lastA = g.buttons.A;
        lastB = g.buttons.B;

        // Print
        printw(out.str().c_str());
        refresh();

        pub.publish(s);
        nyquil.sleep();
    }
    endwin();
    return 0;
}
