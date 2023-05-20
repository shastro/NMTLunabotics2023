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

/*
double stepper_delta = 20.0 / rate; // in mm/s
double min_max_mod = 0.999;
double stepper_max = STEPPER_CTRL_SET_POINT_MAX * min_max_mod;
double stepper_min = STEPPER_CTRL_SET_POINT_MIN * min_max_mod;
*/

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

//Messages
motor_bridge::EStop e_stop;
motor_bridge::PitchCtrl pitch_ctrl;
motor_bridge::LocoCtrl loco_ctrl;
motor_bridge::ExcavCtrl excav_ctrl;
motor_bridge::StepperCtrl stepper_ctrl;
motor_bridge::MastCtrl mast_ctrl;

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

    int count = 0;

    // Create a stream for printing to ncurses.
    NcursesBuf out_buf;
    while (true) {
        std::stringstream out;
        g.update();
        erase();
        move(0, 0);
        if (count < 50) {
            count++;
            continue;
        }

        // Quit
        if (g.buttons.xbox) {
           e_stop.stop = true;
            stop_pub.publish(e_stop);
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
        
        stepper_ctrl.left = stepper_ctrl.right = 0;
        e_stop.stop = false;
        loco_ctrl.left_vel = 0;
        loco_ctrl.right_vel = 0;
        pitch_ctrl.left = pitch_ctrl.right = 0;
        
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

            // Pitch control with dpad
            if (g.dpad.up) {
                pitch_ctrl.left  = pitch_ctrl.right = 1;
                out << "Pitch up\n";
            } else if (g.dpad.down) {
                pitch_ctrl.left  = pitch_ctrl.right = 2;
                out << "Pitch down\n";
            } else {
                pitch_ctrl.left  = pitch_ctrl.right = 0;
            }

            // Viagra bumpers
            if (g.buttons.left_bumper) {
                out << "david's shaft left\n";
               mast_ctrl.direction = 0;
            } else if (g.buttons.right_bumper) {
                out << "david's shaft right\n";
               mast_ctrl.direction = 2;
            } else {
               mast_ctrl.direction = 1;
            }

            // Drive with both sticks
           loco_ctrl.left_vel = g.left_stick.y;
           loco_ctrl.right_vel = g.right_stick.y;
            if (loco_ctrl.left_vel != 0 ||loco_ctrl.right_vel != 0) {
                out << "Driving - L: " <<loco_ctrl.left_vel
                    << ", R: " <<loco_ctrl.right_vel << "\n";
            }

            // Extend dpad
            if (g.dpad.right) {
                stepper_ctrl.left = stepper_ctrl.right = 1;
                out << "stepper up\n";
            } else if (g.dpad.left) {
                stepper_ctrl.left = stepper_ctrl.right = 2;
                out << "stepper down\n";
            } else {
                stepper_ctrl.left = stepper_ctrl.right = 0;
            }

            // Digger triggers
            if (g.left_trigger > 0) {
               excav_ctrl.vel = -g.left_trigger;
                out << "Dig Forward: " <<excav_ctrl.vel << "\n";
            } else if (g.right_trigger > 0) {
               excav_ctrl.vel = g.right_trigger;
                out << "Dig Backward: " <<excav_ctrl.vel << "\n";
            } else {
               excav_ctrl.vel = 0;
            }
            break;

        case Mode::Adjust:
            out << "Adjusting\n\n";
            out << "Left Stick - Left Pitch Arm\n";
            out << "Right Stick - Right Pitch Arm\n";
                
            if (g.left_stick.y > 0)
                pitch_ctrl.left = 1;
            else if (g.left_stick.y < 0)
                pitch_ctrl.left = 2;

            if (g.right_stick.y > 0)
                pitch_ctrl.right = 1;
            else if (g.right_stick.y < 0)
                pitch_ctrl.right = 2;
            break;
        
        case Mode::Home:
            break;

        case Mode::Stop:
            e_stop.stop = true;
            break;
        }

        printw(out.str().c_str());
        refresh();
        
        stop_pub.publish(e_stop);
            pitch_pub.publish(pitch_ctrl);
            loco_pub.publish(loco_ctrl);
            excav_pub.publish(excav_ctrl);
            stepper_pub.publish(stepper_ctrl);
            mast_pub.publish(mast_ctrl);
        
        nyquil.sleep();
    }
    endwin();
    return 0;
}
