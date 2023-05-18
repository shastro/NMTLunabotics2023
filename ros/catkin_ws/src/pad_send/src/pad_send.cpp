#include <algorithm>
#include <iostream>
#include <motor_bridge/System.h>
#include <ncurses.h>
#include <ros/ros.h>
#include <sys/stat.h>

#include "can/limits.hpp"
#include "joystick/gamepad.hpp"

// The Adjustables
int max = 98;   // Max value for sticks/triggers
int min = -max; // Min val ... well yeah
int dead = 20;  // Can't be too touchy. < dead = 0
int rate = 100; // in hertz

double stepper_delta = 20.0 / rate; // in mm/s

double min_max_mod = 0.999;
double stepper_max = STEPPER_CTRL_SET_POINT_MAX * min_max_mod;
double stepper_min = STEPPER_CTRL_SET_POINT_MIN * min_max_mod;

enum class PitchSignal { Stop = 0, Extend = 1, Retract = 2 };

// State handling
enum class Mode { Normal, Adjust, Home, Stop };
Mode m = Mode::Normal;

//Publishers
ros::Publisher stop_pub;
ros::Publisher loco_pub;
ros::Publisher pitch_pub;
ros::Publisher stepper_pub;
ros::Publisher excav_pub;
ros::Publisher mast_pub;

class NcursesBuf : public std::streambuf {
  protected:
    virtual int_type overflow(int_type c) {
        printw("%c", static_cast<char>(c));
        return c;
    }
};

int main(int argc, char *argv[]) {
    // Set up ros
    ros::init(argc, argv, "pad_send");
    ros::NodeHandle nh;
    ros::Publisher stop_pub = nh.advertise<motor_bridge::EStop>("/system/stop", 5);
    ros::Publisher pitch_pub = nh.advertise<motor_bridge::PitchCtrl>("/system/pitch_ctrl", 5);
    ros::Publisher loco_pub = nh.advertise<motor_bridge::LocoCtrl>("/system/loco_ctrl", 5);
    ros::Publisher stepper_pub = nh.advertise<motor_bridge::StepperCtrl>("/system/stepper_ctrl", 5);
    ros::Publisher excav_pub = nh.advertise<motor_bridge::ExcavCtrl>("/system/excav_ctrl", 5);
    ros::Publisher mast_pub = nh.advertise<motor_bridge::MastCtrl>("/system/mast_ctrl", 5);
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

    PitchSignal pitch_left = PitchSignal::Stop;
    PitchSignal pitch_right = PitchSignal::Stop;

    double stepper = 0;

    bool stepper_home = false;

    bool lastX = false;
    bool lastY = false;
    bool lastA = false;
    bool lastB = false;

    int count = 0;

    // Create a stream for printing to ncurses.
    NcursesBuf out_buf;
    std::ostream out(&out_buf);

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

        // Control scheme TODO update
        out << "X - Resume, Y - Estop, A - Adjust, B - Home, XBOX - quit\n";
        out << "Cannot Adjust or Home when Estopped\n\n";

        // std::cout << g << std::endl;

        // Estop
        if (g.buttons.Y) {
            m = Mode::Stop;
        }

        // Adjust
        if (g.buttons.A && m != Mode::Stop) {
            m = Mode::Adjust;
        }

        // Homing
        if (g.buttons.B && m != Mode::Stop) {
            m = Mode::Home;
        }

        // Normal
        if (g.buttons.X) {
            m = Mode::Normal;
        }

        switch (m) {
        case Mode::Normal:
            // Controls
            out << "Normal Operation\n\n";
            out << "Drive: Sticks\n";
            out << "Excavate: Triggers\n";
            out << "Pitch: Dpad Up/Down\n";
            out << "Extend/Retract: Dpad Left/Right\n";
            out << "Mast: Bumpers\n";
            out << "\n";

            // Un home everyting
            // pitch_home = false;
            stepper_home = false;
            s.e_stop.stop = false;

            // Pitch control with dpad
            if (g.dpad.up) {
                pitch_left = pitch_right = PitchSignal::Extend;
                out << "Pitch up\n";
            } else if (g.dpad.down) {
                pitch_left = pitch_right = PitchSignal::Retract;
                out << "Pitch down\n";
            }
            s.pitch_ctrl.left = (uint8_t)pitch_left;
            s.pitch_ctrl.right = (uint8_t)pitch_right;

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
                stepper = std::min(stepper, stepper_max);
                out << "Arm Extend: " << stepper << "\n";
            } else if (g.dpad.right) {
                stepper -= stepper_delta;
                stepper = std::max(stepper, stepper_min);
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

        case Mode::Adjust:
            out << "Adjusting\n\n";
            out << "Left Stick - Left Pitch Arm\n";
            out << "Right Stick - Right Pitch Arm\n";
            out << "\n";

            // Pitch
            if (abs(g.left_stick.y) > dead) {
                s.pitch_ctrl.left =
                    (uint8_t)(g.left_stick.y > 0 ? PitchSignal::Extend
                                                 : PitchSignal::Retract);
                out << "Pitch left: " << s.pitch_ctrl.left;
            }

            if (abs(g.right_stick.y) > dead) {
                s.pitch_ctrl.right =
                    (uint8_t)(g.right_stick.y > 0 ? PitchSignal::Extend
                                                  : PitchSignal::Retract);
                out << "Pitch right: " << s.pitch_ctrl.left;
            }
            break;

        case Mode::Home:
            out << "Homing\n\n";
            out << "Extend: Dpad Left or Right\n";
            out << "\n";

            if (g.dpad.left || g.dpad.right) {
                out << "Extend Home\n";
                stepper_home = true;
                stepper = stepper_min;
                s.stepper_ctrl.set_point = stepper;
            }

            break;

        case Mode::Stop:
            out << "Contemplating Life\n";
            s.e_stop.stop = true;
            break;

        default:
            break;
        }

        // Sending homing messages
        s.stepper_ctrl.home = stepper_home;

        // Update last state
        lastX = g.buttons.X;
        lastY = g.buttons.Y;
        lastA = g.buttons.A;
        lastB = g.buttons.B;

        // Print
        refresh();

        pub.publish(s);
        nyquil.sleep();
    }
    endwin();
    return 0;
}
