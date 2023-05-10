#include <iostream>
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
double pitch_delta = 8; // in mm/s
double stepper_delta = 20; // in mm/s

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

    double pitch_set = (PITCH_CTRL_SET_POINT_MAX -
            PITCH_CTRL_SET_POINT_MIN) / 2;
    double stepper_set = 0;
    double pitch_left_offset = 0;
    double pitch_right_offset = 0;

    bool estopped = false;
    bool adjust = false;
    bool homing = false;

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
            adjust = false;
            homing = false;
        }

        if (g.buttons.xbox) {
            s.e_stop.stop = true;
            pub.publish(s);
            endwin();
            return 0;
        }
        s.e_stop.stop = estopped;
        if (estopped)
            out << "Estopped\n";

        // Adjust mode
        if (g.buttons.A && !lastA) {
            if (homing || estopped) {
                out << "Cannot adjust while homing or estopped\n";
            } else {
                adjust = !adjust;
            }
        }

        // Homing
        if (g.buttons.B && !lastB) {
            if (adjust || estopped) {
                out << "Cannot home while adjusting or estopped\n";
            } else {
                homing = !homing;
            }
        }

        if (!adjust && !estopped) {
            // Pitch control with dpad
            if (homing) {
                out << "Pitch Homing\n";
                pitch_set = PITCH_CTRL_SET_POINT_MIN;
            } else {
                if (g.dpad.up) {
                    pitch_set += pitch_delta / rate;
                    if (pitch_set > PITCH_CTRL_SET_POINT_MAX)
                        pitch_set = PITCH_CTRL_SET_POINT_MAX;
                    out << "Pitch Up\n";
                } else if (g.dpad.down) {
                    pitch_set -= pitch_delta / rate;
                    if (pitch_set < PITCH_CTRL_SET_POINT_MIN)
                        pitch_set = PITCH_CTRL_SET_POINT_MIN;
                    out << "Pitch Down\n";
                }
            }
            s.pitch_ctrl.set_point = homing;
            s.pitch_ctrl.set_point = pitch_set;

            // Viagra
            if (g.dpad.left) {
                out << "david's shaft left\n";
                s.mast_ctrl.direction = 1;
            } else if (g.dpad.right) {
                out << "david's shaft right\n";
                s.mast_ctrl.direction = 2;
            } else {
                s.mast_ctrl.direction = 0;
            }

            // Drive with both sticks
            s.loco_ctrl.left_vel = g.left_stick.y;
            s.loco_ctrl.right_vel = g.right_stick.y;
            if (s.loco_ctrl.left_vel != 0 || s.loco_ctrl.right_vel != 0) {
                out << "Driving - L: " << s.loco_ctrl.left_vel
                    << ", R: " << s.loco_ctrl.right_vel << "\n";
            }

            // Extend bumpers
            if (homing) {
                out << "Arm Homing\n";
                stepper_set = STEPPER_CTRL_SET_POINT_MIN;
            } else {
                s.stepper_ctrl.home = false;
                if (g.buttons.right_bumper) {
                    stepper_set += stepper_delta / rate;
                    if (stepper_set > STEPPER_CTRL_SET_POINT_MAX)
                        stepper_set = STEPPER_CTRL_SET_POINT_MAX;
                    out << "Arm Extend\n";
                } else if (g.buttons.left_bumper) {
                    stepper_set -= stepper_delta / rate;
                    if (stepper_set < STEPPER_CTRL_SET_POINT_MIN)
                        stepper_set = STEPPER_CTRL_SET_POINT_MIN;
                    out << "Arm Retract\n";
                }
            }
            s.stepper_ctrl.home = homing;
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
            out << "Adjust mode\n";
            pitch_left_offset += pitch_delta * g.left_stick.y;
            if (pitch_left_offset > PITCH_CTRL_LEFT_OFFSET_MAX) {
                pitch_left_offset = PITCH_CTRL_LEFT_OFFSET_MAX;
            } else if (pitch_left_offset < PITCH_CTRL_LEFT_OFFSET_MIN) {
                pitch_left_offset = PITCH_CTRL_LEFT_OFFSET_MIN;
            }

            pitch_right_offset += pitch_delta * g.right_stick.y;
            if (pitch_right_offset > PITCH_CTRL_RIGHT_OFFSET_MAX) {
                pitch_right_offset = PITCH_CTRL_RIGHT_OFFSET_MAX;
            } else if (pitch_right_offset < PITCH_CTRL_RIGHT_OFFSET_MIN) {
                pitch_right_offset = PITCH_CTRL_RIGHT_OFFSET_MIN;
            }
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
        nyquil.sleep();
    }
    endwin();
    return 0;
}
