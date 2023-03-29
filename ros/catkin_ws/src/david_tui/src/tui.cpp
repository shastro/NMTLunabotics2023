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

// Selectors
static control sel_c;
static motor sel_m = BOTH;

// Keybind functions
static void sel_pitch() { sel_c = PITCH; }
static void sel_drive() { sel_c = DRIVE; }
static void sel_depth() { sel_c = DEPTH; }

static void sel_both() { sel_m = BOTH; }
static void sel_left() { sel_m = LEFT; }
static void sel_right() { sel_m = RIGHT; }

static void drive_forward();
static void drive_backward();
static void drive_right_forward();
static void drive_left_forward();
static void drive_right_backward();
static void drive_left_backward();
static void drive_spin_right();
static void drive_spin_left();
static void drive_stop();

static void pitch_down();
static void pitch_up();
static void pitch_stop();

static void depth_extend();
static void depth_retract();
static void depth_stop();

static void digger_forward();
static void digger_backward();
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
static void print_keybinds();

// Publishers
static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;
static ros::Publisher excav_pub;
static ros::Publisher estop_pub;

// Keybinds - char k, string description, void function
static std::tuple<char, std::string, std::function<void()>> bindings[] = {
    // Control
    {'p', "Pitch", sel_pitch},
    {'d', "Drive", sel_drive},
    {'a', "Excavation Arm", sel_depth},
    // Motor
    {'0', "Both", sel_both},
    {'1', "Left", sel_left},
    {'2', "Right", sel_both},
    // Command
    {'j', "Drive Both Forward", drive_forward},
    {'k', "Drive Both Backward", drive_backward},
    {'h', "Drive Right Motor Forward", drive_right_forward},
    {'l', "Drive Left Motor Forward", drive_left_forward},
    {'H', "Drive Right Motor Backward", drive_right_backward},
    {'L', "Drive Left Motor Backward", drive_left_backward},
    {'r', "Spin Right (Right backward, Left forward)", drive_spin_right},
    {'R', "Spin Left (Right forward, Left backward)", drive_spin_left},
    {'J', "Pitch Down", pitch_down},
    {'K', "Pitch Up", pitch_up},
    {'o', "Extend depth", depth_extend},
    {'i', "Retract depth", depth_retract},
    {'d', "Digger forward", digger_forward},
    {'D', "Digger backward", digger_backward},
    {'s', "Stop Current", stop_selected},
    {'c', "Stop All", stop_all},
    {'Q', "Emergency Stop", estop},
    {'b', "Restart", estart},
    {'q', "Quit", quit}
};

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);
    excav_pub = nh.advertise<motor_bridge::Digger>("/excav_control", 5);
    estop_pub = nh.advertise<motor_bridge::Estop>("/estop", 5);

    // Initialize min and max states
    motorsys.loco.min = -1024;
    motorsys.loco.max = 1024;
    motorsys.stepp.min = 0;
    motorsys.stepp.max = 1024;
    motorsys.digger.min = -1024;
    motorsys.digger.max = 1024;

    // Setup ncurses.
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    // Refresh at 0.1-second interval.
    halfdelay(1);

    while (true) {
        clear();
        move(0, 0);
        print_status();
        print_keybinds();
        refresh();

        int n = getch();

        for (auto bind : bindings)
            if (n == std::get<0>(bind))
                std::get<2>(bind)();

        send_targets();
    }
}

static void drive_forward() {
    //TODO implement changing speed
    sel_c = DRIVE;
    motorsys.loco.left_speed = motorsys.loco.max;
    motorsys.loco.right_speed = motorsys.loco.max;
}

static void drive_backward() {
    //TODO implement changing speed
    sel_c = DRIVE;
    motorsys.loco.left_speed = motorsys.loco.min;
    motorsys.loco.right_speed = motorsys.loco.min;
}

static void drive_right_forward() {
    sel_c = DRIVE;
    motorsys.loco.right_speed = motorsys.loco.max;
    motorsys.loco.left_speed = 0;
}

static void drive_left_forward() {
    sel_c = DRIVE;
    motorsys.loco.left_speed = motorsys.loco.max;
    motorsys.loco.right_speed = 0;
}

static void drive_right_backward() {
    sel_c = DRIVE;
    motorsys.loco.right_speed = motorsys.loco.min;
    motorsys.loco.left_speed = 0;
}

static void drive_left_backward() {
    sel_c = DRIVE;
    motorsys.loco.left_speed = motorsys.loco.min;
    motorsys.loco.right_speed = 0;
}

static void drive_spin_right() {
    sel_c = DRIVE;
    motorsys.loco.right_speed = motorsys.loco.min;
    motorsys.loco.left_speed = motorsys.loco.max;
}

static void drive_spin_left() {
    sel_c = DRIVE;
    motorsys.loco.right_speed = motorsys.loco.max;
    motorsys.loco.left_speed = motorsys.loco.min;
}

static void drive_stop() {
    sel_c = DRIVE;
    motorsys.loco.left_speed = 0;
    motorsys.loco.right_speed = 0;
}

static void pitch_down() {
    sel_c = PITCH;
    motorsys.pitch.dir = BACKWARD;
}

static void pitch_up() {
    sel_c = PITCH;
    motorsys.pitch.dir = FORWARD;
}

static void pitch_stop() {
    sel_c = PITCH;
    motorsys.pitch.dir = STOP;
}

static void depth_extend() {
    sel_c = DEPTH;
    motorsys.stepp.rpm = motorsys.stepp.max;
    motorsys.stepp.dir = FORWARD;
}

static void depth_retract() {
    sel_c = DEPTH;
    motorsys.stepp.rpm = motorsys.stepp.max;
    motorsys.stepp.dir = BACKWARD;
}

static void depth_stop() {
    sel_c = DEPTH;
    motorsys.stepp.rpm = motorsys.stepp.min;
    motorsys.stepp.dir = STOP;
}

static void digger_forward() {
    sel_c = DIGGER;
    motorsys.digger.speed = motorsys.digger.max;
}

static void digger_backward() {
    sel_c = DIGGER;
    motorsys.digger.speed = motorsys.digger.min;
}

static void digger_stop() {
    sel_c = DIGGER;
    motorsys.digger.speed = 0;
}

static void stop_selected() {
    if (sel_c == PITCH)
        pitch_stop();
    if (sel_c == DEPTH)
        depth_stop();
    if (sel_c = DRIVE)
        drive_stop();
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

static void print_keybinds() {
    printw("Keybinds\n");
    for (auto bind : bindings) {
        print_bold("%c", std::get<0>(bind));
        printw(" : ");
        printw("%s", std::get<1>(bind).c_str());
        printw("\n");
    }
}
