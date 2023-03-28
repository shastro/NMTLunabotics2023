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
enum control { PITCH = 0, LOCO = 1, STEPP = 2, DIGGER = 3 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

// State holders
struct pitch_target {
    direction dir;
};

struct loco_target {
    int l_speed;
    int r_speed;
    int min;
    int max;
};

struct stepp_target {
    int rpm;
    direction dir;
    int min;
    int max;
};

struct digger_target {
    int speed;
    int min;
    int max;
}

static struct {
    pitch_target pitch;
    loco_target loco;
    stepp_target stepp;
    digger_target digger;
} motorsys;

// Initialize min and max states
motorsys.loco.min = -1024;
motorsys.loco.max = 1024;
motorsys.stepp.min = 0;
motorsys.stepp.max = 1024;
motorsus.digger.min = -1024;
motorsus.digger.max = 1024;

// Selectors
static control sel_c;
static motor sel_m = BOTH;

// Keybind functions
static void sel_pitch() { sel_c = PITCH };
static void sel_drive() { sel_c = DRIVE };
static void sel_depth() { sel_c = DEPTH };

static void sel_both() { sel_m = BOTH };
static void sel_left() { sel_m = LEFT };
static void sel_right() { sel_m = RIGHT };

static void drive_forward();
static void drive_backward();
static void drive_stop();

static void pitch_down();
static void pitch_up();
static void pitch_stop();

static void depth_extend();
static void depth_retract();
static void depth_stop();

static void digger_forward();
static void digger_backward();
static void digger_stop()

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
    {'b', "Both", sel_both},
    {'l', "Left", sel_left},
    {'r', "Right", sel_both},
    // Command
    {'j', "Drive Forward", drive_forward},
    {'k', "Drive Backward", drive_backward},
    {'J', "Pitch Down", pitch_down},
    {'K', "Pitch Up", pitch_up},
    {'H', "Extend depth", depth_extend},
    {'L', "Retract depth", depth_retract},
    {'d', "Digger forward", digger_forward},
    {'D', "Digger backward", digger_backward},
    {'s', "Stop Current", stop_selected},
    {'c', "Stop All", stop_all},
    {'Q', "Emergency Stop". estop},
    {'R', "Restart", estart},
    {'q', "Quit", quit},
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

    // Setup motorsys
    motorsys.pitch.length.min = 0;
    motorsys.pitch.length.max = 1024;
    motorsys.loco.speed.min = -1024;
    motorsys.loco.speed.max = 1024;
    motorsys.stepp.rpm.min = 0;
    motorsys.stepp.rpm.max = 1024;

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
    sel_c = LOCO;
    if (sel_m != RIGHT)
        motorsys.loco.left_speed = motorsys.loco.max;
    if (sel_m != LEFT)
        motorsys.loco.right_speed = motorsys.loco.max;
}

static void drive_backward() {
    //TODO implement changing speed
    sel_c = LOCO;
    if (sel_m != RIGHT)
        motorsys.loco.left_speed = motorsys.loco.min;
    if (sel_m != LEFT)
        motorsys.loco.right_speed = motorsys.loco.min;
}

static void drive_stop() {
    sel_c = LOCO;
    if (sel_m != RIGHT)
        motorsys.loco.left_speed = 0;
    if (sel_m != LEFT)
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
    if (sel_C = LOCO)
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
    printw("status here\n");
}

static void print_keybinds() {
    printw("Keybinds\n");
    for (auto k : bindings) {
        print_bold("%c", k.first);
        printw(" : ");
        printw("%s", k.second.c_str());
        printw("\n");
    }
}
