#include <iostream>
#include <sys/stat.h>
#include <ncurses.h>
#include <motor_bridge/System.h>
#include <ros/ros.h>
#include "joystick/gamepad.hpp"
#include "can/limits.hpp"

int map(int val, int omin, int omax, int nmin, int nmax) {
    return (val - omin) / (omax - omin) * (nmax - nmin) + nmin;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pad_send");
    ros::NodeHandle nh;
    int max = 100;
    int min = -max;
    int dead = 20;
    ros::Publisher pub = nh.advertise<motor_bridge::System>("/system", 5);
    ros::Rate rate(10);

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
    double pitch_set = (PITCH_CTRL_SET_POINT_MAX -
            PITCH_CTRL_SET_POINT_MIN) / 2;
    double pitch_delta = 0.000001 * PITCH_CTRL_SET_POINT_MAX;
    double stepper_set = (STEPPER_CTRL_SET_POINT_MAX -
            STEPPER_CTRL_SET_POINT_MIN) / 2;
    double stepper_delta = 0.000001 * PITCH_CTRL_SET_POINT_MAX;
    bool estopped = false;
    bool adjust = false;
    int count = 0;
    bool lastX = false;
    bool lastY = false;
    bool lastA = false;
    bool lastB = false;

    while (true) {
        g.update();
        erase();
        move(0, 0);
        if (count < 10) {
            count++;
            continue;
        }

        s.header.stamp = ros::Time::now();
        s.header.frame_id = "Pad Send";

        std::stringstream out;

        // Control scheme
        out << "Y - Toggle Estop, A - Adjust mode, XBOX - quit\n";
        out << "Left stick - left tread, right stick - right tread\n";
        out << "Dpad: up/down - pitch\n";
        out << "Bumpers: left - arm extend, right - arm retract\n";
        out << "B: Arm home\n";

        //std::cout << g << std::endl;

        // Estop
        if (g.buttons.Y && ! lastY) {
            estopped = !estopped;
        }

        if (g.buttons.xbox) {
            estopped = true;
            pub.publish(s);
            endwin();
            return 0;
        }
        s.e_stop.stop = estopped;
        if (estopped)
            out << "Estopped\n";

        // Adjust mode
        if (g.buttons.A && !lastA) {
            adjust = !adjust;
        }

        if (!adjust && !estopped) {
            // Pitch control with dpad
            if (g.dpad.up) {
                pitch_set += pitch_delta;
                if (pitch_set > PITCH_CTRL_SET_POINT_MAX)
                    pitch_set = PITCH_CTRL_SET_POINT_MAX;
                out << "Pitch Up\n";
            } else if (g.dpad.down) {
                pitch_set -= pitch_delta;
                if (pitch_set < PITCH_CTRL_SET_POINT_MIN)
                    pitch_set = PITCH_CTRL_SET_POINT_MIN;
                out << "Pitch Down\n";
            }
            s.pitch_ctrl.set_point = pitch_set;

            // Drive with both sticks
            s.loco_ctrl.left_vel = g.left_stick.y;
            s.loco_ctrl.right_vel = g.right_stick.y;
            if (s.loco_ctrl.left_vel != 0 || s.loco_ctrl.right_vel != 0) {
                out << "Driving - L: " << s.loco_ctrl.left_vel
                    << ", R: " << s.loco_ctrl.right_vel << "\n";
            }

            // Extend bumpers
            if (g.buttons.B) {
                out << "Arm Homing\n";
                stepper_set = STEPPER_CTRL_SET_POINT_MIN;
                s.stepper_ctrl.home = true;
            } else {
                s.stepper_ctrl.home = false;
                if (g.buttons.right_bumper) {
                    stepper_set += stepper_delta;
                    if (stepper_set > STEPPER_CTRL_SET_POINT_MAX)
                        stepper_set = STEPPER_CTRL_SET_POINT_MAX;
                    out << "Arm Extend\n";
                } else if (g.buttons.left_bumper) {
                    stepper_set -= stepper_delta;
                    if (stepper_set < STEPPER_CTRL_SET_POINT_MIN)
                        stepper_set = STEPPER_CTRL_SET_POINT_MIN;
                    out << "Arm Retract\n";
                }
            }
            s.stepper_ctrl.set_point = stepper_set;

            // Digger triggers
            if (g.left_trigger > 0) {
                s.excav_ctrl.vel = -g.left_trigger;
                out << "Dig Forward\n";
            } else if (g.right_trigger > 0) {
                s.excav_ctrl.vel = g.right_trigger;
                out << "Dig Backward\n";
            } else {
                s.excav_ctrl.vel = 0;
            }
        } else {
            out << "Adjust Not Implemented\n";

        }

        // Update last state
        lastX = g.buttons.X;
        lastY = g.buttons.Y;
        lastA = g.buttons.A;
        lastB = g.buttons.B;

        // Print
        printw(out.str().c_str());
        refresh();

        pub.publish(s);
        rate.sleep();
    }
    endwin();
    return 0;
}
