#include <iostream>
#include <ncurses.h>

/*
#include <motor_bridge/Digger.h>
#include <motor_bridge/Drive.h>
#include <motor_bridge/Pitch.h>
#include <motor_bridge/Stepp.h>
#include <motor_bridge/Estop.h>
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "pad_send/Gamepad.h"

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
enum input { GAMEPAD = 0, KEYBOARD = 1 };

// Print helpers
std::ostream &operator<<(std::ostream &s, control c);
std::ostream &operator<<(std::ostream &s, motor m);
std::ostream &operator<<(std::ostream &s, direction d);
std::ostream &operator<<(std::ostream &s, input in);

// State holders
struct pitch_target;
struct pitch_target {
    direction dir;
    friend std::ostream &operator<<(std::ostream &s, const pitch_target &t) {
        s << "Pitch Direction: " << t.dir << std::endl;
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

// Stop functions
static void stop();
//static void estop();
//static void estart();
//static void quit();

// Can handlers
/*
static void send_targets();
static void send_pitch();
static void send_loco(motor m);
static void send_stepp();
static void send_digger();
*/


// Publishers
/*
static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;
static ros::Publisher excav_pub;
static ros::Publisher estop_pub;
*/

// Gamebinds - string bind string description
/*
static std::pair<std::string, std::string> gamebinds[] = {
    {"Left Stick", "Drive"},
    {"Right Stick Y", "Pitch"},
    {"Right Stick X", "Extend"},
    {"Left Trigger", "Dig Forward"},
    {"Right Trigger", "Dig Reverse"},
    {"B", "Stop all"},
    {"xbox", "Emergency Stop"},
    {"Left & Right Bumper", "Restart after Emergency Stop"}
};

*/

void Callback(const pad_send::Gamepad::ConstPtr& g) {
    // Initialize min and max states
    motorsys.loco.min = -g->max;
    motorsys.loco.max = g->max;
    motorsys.stepp.min = 0;
    motorsys.stepp.max = g->max;
    motorsys.digger.min = -g->max;
    motorsys.digger.max = g->max;

    // Drive with left stick
    int left = g->left_stick_x + g->left_stick_y;
    int right = g->left_stick_y - g->left_stick_x;
    left = (left > motorsys.loco.max) ? motorsys.loco.max : left;
    left = (left < motorsys.loco.min) ? motorsys.loco.min : left;
    right = (right > motorsys.loco.max) ? motorsys.loco.max : right;
    right = (right < motorsys.loco.min) ? motorsys.loco.min : right;
    motorsys.loco.left_speed = left;
    motorsys.loco.right_speed = right;

    // Pitch with right stick y
    if (g->right_stick_y > 0)
        motorsys.pitch.dir = FORWARD;
    else if (g->right_stick_y < 0)
        motorsys.pitch.dir = BACKWARD;
    else
        motorsys.pitch.dir = STOP;

    // Extend with right stick x
    if (g->right_stick_x > 0) {
        motorsys.stepp.rpm = g->right_stick_x;
        motorsys.stepp.dir = FORWARD;
    } else if (g->right_stick_x < 0) {
        motorsys.stepp.rpm = -g->right_stick_x;
        motorsys.stepp.dir = BACKWARD;
    } else {
        motorsys.stepp.rpm = 0;
        motorsys.stepp.dir = STOP;
    }

    // Dig with triggers
    if (g->left_trigger > 0)
        motorsys.digger.speed = -g->left_trigger;
    else if (g->right_trigger > 0)
        motorsys.digger.speed = g->right_trigger;
    else
        motorsys.digger.speed = 0;

    // Buttons
    if (g->B)
        stop();
    //if (g->xbox)
        //estop();
    //if (g->leftBumper && g->rightBumper)
        //estart();
    
    //clear();
    //std::stringstream s;
    //s << motorsys;
    std::cout << motorsys << std::endl;
    //printw(s.str().c_str());
    //refresh();
}   

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "pad_recv");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gamepad", 5, Callback);

    ros::spin();

    /*
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);
    excav_pub = nh.advertise<motor_bridge::Digger>("/excav_control", 5);
    estop_pub = nh.advertise<motor_bridge::Estop>("/estop", 5);
    */
    return 0;
}

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

std::ostream &operator<<(std::ostream &s, input in) {
    switch(in) {
        case input::GAMEPAD:
            s << "gamepad";
            break;
        case input::KEYBOARD:
            s << "keyboard";
            break;
    }
    return s;
}

static void stop() {
    motorsys.loco.left_speed = 0;
    motorsys.loco.right_speed = 0;
    motorsys.pitch.dir = STOP;
    motorsys.stepp.rpm = motorsys.stepp.min;
    motorsys.stepp.dir = STOP;
    motorsys.digger.speed = 0;
}

/*
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
    clear();
    estop();
    endwin();
    still_going = false;
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
*/
