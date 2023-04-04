#include <iostream>
#include <ncurses.h>

#include <motor_bridge/Digger.h>
#include <motor_bridge/Drive.h>
#include <motor_bridge/Pitch.h>
#include <motor_bridge/Stepp.h>
#include <motor_bridge/Estop.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "joystick/gamepad.hpp"

#define PITCH_STOP 0
#define PITCH_FORWARD 1
#define PITCH_BACKWARD 2

#define print_bold(...)                                                        \
    do {                                                                       \
        attron(A_BOLD);                                                        \
        printw(__VA_ARGS__);                                                   \
        attroff(A_BOLD);                                                       \
    } while (0)

// Control maps
enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum control { PITCH = 0, DRIVE = 1, DEPTH = 2, DIGGER = 3 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

std::ostream &operator<<(std::ostream &s, control c) {
    switch (c) {
        case control::PITCH:
            s << "pitch";
            break;
        case control::DRIVE:
            s << "drive";
            break;
        case control::DEPTH:
            s << "step";
            break;
    }
    return s;
}

std::ostream &operator<<(std::ostream &s, motor m) {
    switch (m) {
        case motor::BOTH:
            s << "both";
            break;
        case motor::LEFT:
            s << "left";
            break;
        case motor::RIGHT:
            s << "right";
            break;
    }
    return s;
}

std::ostream &operator<<(std::ostream &s, direction d) {
    switch (d) {
        case direction::STOP:
            s << "stop";
            break;
        case direction::FORWARD:
            s << "forward";
            break;
        case direction::BACKWARD:
            s << "backward";
            break;
    }
    return s;
}

// State holders
struct pitch_target {
    direction dir;
    friend std::ostream &operator<<(std::ostream &s, const pitch_target &t) {
        s << "Pitch Direction: " << t.dir;
        return s;
    }
};

struct loco_target {
    int left_speed;
    int right_speed;
    int min;
    int max;
    friend std::ostream &operator<<(std::ostream &s, const loco_target &t) {
        s << "Drive left: " << t.left_speed
            << ", Right: " << t.right_speed
            << ", Min: " << t.min
            << ", Max: " << t.max
            << std::endl;
        return s;
    }
};

struct stepp_target {
    int rpm;
    direction dir;
    int min;
    int max;
    friend std::ostream &operator<<(std::ostream &s, const stepp_target &t) {
        s << "Stepper Direction: " << t.dir
            << ", RPM: " << t.rpm
            << ", Min: " << t.min
            << ", Max: " << t.max
            << std::endl;
        return s;
    }
};

struct digger_target {
    int speed;
    int min;
    int max;
    friend std::ostream &operator<<(std::ostream &s, const digger_target &t) {
        s << "Digger speed: " << t.speed
            << ", Min: " << t.min
            << ", Max: " << t.max
            << std::endl;
        return s;
    }
};

struct sys {
    pitch_target pitch;
    loco_target loco;
    stepp_target stepp;
    digger_target digger;
    friend std::ostream &operator<<(std::ostream &s, const sys &t) {
        s << t.pitch << t.loco << t.stepp << t.digger << std::endl;
        return s;
    }
};

static sys motorsys;

// Motion functions
static void drive_stop();
static void pitch_stop();
static void depth_stop();
static void digger_stop();

static void stop_selected();
static void stop_all();
static void estop(); //technically a can handler but mapped to keybind
static void estart();
static void quit();

// Can handlers
static void send_targets();
static void send_pitch();
static void send_loco(motor m);
static void send_stepp();
static void send_digger();

// Display functions
static void print_status();
//static void print_keybinds();

// Publishers
static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;
static ros::Publisher excav_pub;
static ros::Publisher estop_pub;

//Max range of input/outputs
const static int rmax = 1024;
//Joy stick center deadzone, 50-200 recommended
const static int dead = 100;

int main(int argc, char **argv) {
    // Game Controller
    GamepadHandler g("/dev/input/js0", rmax, dead);

    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);
    excav_pub = nh.advertise<motor_bridge::Digger>("/excav_control", 5);
    estop_pub = nh.advertise<motor_bridge::Estop>("/estop", 5);

    // Initialize min and max states
    motorsys.loco.min = -max;
    motorsys.loco.max = max;
    motorsys.stepp.min = 0;
    motorsys.stepp.max = max;
    motorsys.digger.min = -max;
    motorsys.digger.max = max;

    // Setup ncurses.
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    // Refresh at 0.1-second interval.
    halfdelay(1);

    while (true) {
        std::cout << "\033[2J\033[1;1H";
        clear();
        move(0, 0);
        print_status();
        //print_keybinds();
        refresh();
        g.update();

        int n = getch();

        // Drive control - left stick
        int left = gh.left_stick.x + gh.left_stick.y;
        int right = gh.left_stick.y - gh.left_stick.x;
        left = (left > motorsys.loco.max) ? motorsys.loco.max : left;
        left = (left < motorsys.loco.min) ? motorsys.loco.min : left;
        right = (right > motorsys.loco.max) ? motorsys.loco.max : right;
        right = (right < motorsys.loco.min) ? motorsys.loco.min : right;
        motorsys.loco.left_speed = left;
        motorsys.loco.right_speed = right;

        // Pitch control - right stick y
        if (gh.right_stick.y > 0)
            motorsys.pitch.dir = FORWARD;
        else if (gh.right_stick.y < 0)        
            motorsys.pitch.dir = BACKWARD;
        else
            motorsys.pitch.dir = STOP;

        // Extend Control - right stick x
        if (gh.right_stick.x > 0) {
            motorsys.stepp.rpm = motorsys.stepp.max;
            motorsys.stepp.dir = FORWARD;
        } else if (gh.right_stick.x < 0) {
            motorsys.stepp.rpm = motorsys.stepp.max;
            motorsys.stepp.dir = BACKWARD;
        } else {
            motorsys.stepp.rpm = motorsys.stepp.min;
            motorsys.stepp.dir = STOP;
        }

        // Digger control
        // dpad left forward
        // dpad right backward
        if (gh.right_trigger > 0)
            motorsys.digger.speed = motorsys.digger.max;
        else if (gh.left_trigger > 0)
            motorsys.digger.speed = motorsys.digger.min;
        else
            motorsys.digger.speed = 0;

        // Stops
        // B - stop all
        // xbox button - estop
        // left and right bumper - estart
        if (gh.buttons.B)
            stop_all();
        if (gh.buttons.xbox)
            estop();
        if (gh.buttons.left_bumper && gh.buttons.right_bumper)
            estart();

        send_targets();
    }
}

static void stop_all() {
    pitch_stop();
    depth_stop();
    drive_stop();
    digger_stop();
}

static void estop() {
    motor_bridge::Estop msg;
    msg.stop = true;
    estop_pub.publish(msg);
}

static void estart() {
    motor_bridge::Estop msg;
    msg.stop = false;
    estop_pub.publish(msg);
}

static void quit() {
    estop();
    endwin();
}

static void send_targets() {
    send_pitch();
    send_loco(sel_m);
    send_stepp();
    send_digger();
}

static void send_pitch() {
    motor_bridge::Pitch msg;
    msg.motor = 0;
    msg.direction = (int) motorsys.pitch.dir;
    pitch_pub.publish(msg);
}

static void send_loco(motor m) {
    motor_bridge::Drive msg;
    msg.motor = (int)m;
    if (m == RIGHT) {
        msg.speed = motorsys.loco.right_speed;
    } else {
        msg.speed = motorsys.loco.left_speed;
    }
    loco_pub.publish(msg);
}

static void send_stepp() {
    motor_bridge::Stepp msg;
    msg.motor = 0;
    msg.rpm = motorsys.stepp.rpm;
    msg.direction = (int) motorsys.stepp.dir;
    stepp_pub.publish(msg);
}

static void send_digger() {
    motor_bridge::Digger msg;
    msg.speed = motorsys.digger.speed;
    excav_pub.publish(msg);
}

static void print_status() {
    std::stringstream s;
    s << motorsys;
    printw(s.str());
}
/*
   static void print_keybinds() {
   printw("Keybinds\n");
   for (auto bind : bindings) {
   print_bold("%c", std::get<0>(bind));
   printw(" : ");
   printw("%s", std::get<1>(bind).c_str());
   printw("\n");
   }
   }*/
