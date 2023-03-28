#include <iostream>
#include <ncurses.h>

#include <motor_bridge/Drive.h>
#include <motor_bridge/Pitch.h>
#include <motor_bridge/Stepp.h>
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

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum control { PITCH = 0, LOCO = 1, STEPP = 2 };
enum direction { BACKWARD = -1, STOP = 0, FORWARD = 1 };

static std::unordered_map<char, std::string> bindings = {
    // Control
    {'p', "Pitch"},
    {'d', "Drive"},
    {'a', "Excavation Arm"},
    // Motor
    {'b', "Both"},
    {'l', "Left"},
    {'r', "Right"},
    // Command
    {'j', "Drive Forward"},
    {'k', "Drive Backward"},
    {'J', "Pitch Down"},
    {'K', "Pitch Up"},
    {'h', "Arm Out"},
    {'l', "Arm In"},
    {'s', "Stop Current"},
    {'c', "Stop All"},
    {'q', "Quit"}};

static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;

struct target {
    int left;
    int right;
    int min;
    int max;
};

struct target_dir {
    direction left;
    direction right;
};

struct pitch_target {
    target length;
};

struct loco_target {
    target speed;
};

struct stepp_target {
    target rpm;
    target_dir dir;
};

static struct {
    pitch_target pitch;
    loco_target loco;
    stepp_target stepp;
} motorsys;

static control sel_c;
static motor sel_m = BOTH;

static void send_targets();
static void send_pitch(motor m);
static void send_loco(motor m);
static void send_stepp(motor m);

static void print_keybinds();

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);

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
        print_keybinds();
        refresh();

        int n = getch();

        switch (n) {
        case 'p':
            sel_c = PITCH;
            break;
        case 'd':
            sel_c = LOCO;
            break;
        case 'e':
            sel_c = STEPP;

        case 'l':
            sel_m = LEFT;
            break;
        case 'r':
            sel_m = RIGHT;
            break;
        case 'b':
            sel_m = BOTH;
            if (sel_c == PITCH) {
                motorsys.pitch.length.left = motorsys.pitch.length.right;
            } else if (sel_c == LOCO) {
                motorsys.loco.speed.left = motorsys.loco.speed.right;
            } else if (sel_c == STEPP) {
                motorsys.stepp.rpm.left = motorsys.stepp.rpm.right;
                motorsys.stepp.dir.left = motorsys.stepp.dir.right;
            }
            break;

        case 'j':
            // TODO implement changeable speed
            sel_c = LOCO;
            if (sel_m != RIGHT) {
                motorsys.loco.speed.left = motorsys.loco.speed.max;
            } else if (sel_m != LEFT) {
                motorsys.loco.speed.right = motorsys.loco.speed.max;
            }
            break;
        case 'k':
            sel_c = LOCO;
            if (sel_m != RIGHT) {
                motorsys.loco.speed.left = motorsys.loco.speed.min;
            } else if (sel_m != LEFT) {
                motorsys.loco.speed.right = motorsys.loco.speed.min;
            }
            break;
        case 'J':
            sel_c = PITCH;
            if (sel_m != RIGHT)
                motorsys.pitch.length.left = PITCH_BACKWARD;
            if (sel_m != LEFT)
                motorsys.pitch.length.right = PITCH_BACKWARD;
            break;
        case 'K':
            sel_c = PITCH;
            if (sel_m != RIGHT)
                motorsys.pitch.length.left = PITCH_FORWARD;
            if (sel_m != LEFT)
                motorsys.pitch.length.right = PITCH_FORWARD;
            break;
        case 'o':
            sel_c = STEPP;
            if (sel_m != RIGHT) {
                motorsys.stepp.rpm.left = motorsys.stepp.rpm.max;
                motorsys.stepp.dir.left = FORWARD;
            } else if (sel_m != LEFT) {
                motorsys.stepp.rpm.right = motorsys.stepp.rpm.max;
                motorsys.stepp.dir.right = FORWARD;
            }
            break;
        case 'i':
            sel_c = STEPP;
            if (sel_m != RIGHT) {
                motorsys.stepp.rpm.left = motorsys.stepp.rpm.max;
                motorsys.stepp.dir.left = BACKWARD;
            } else if (sel_m != LEFT) {
                motorsys.stepp.rpm.right = motorsys.stepp.rpm.max;
                motorsys.stepp.dir.right = BACKWARD;
            }
            break;

        case 's':
            if (sel_c == LOCO) {
                motorsys.loco.speed.left = 0;
                motorsys.loco.speed.right = 0;
            } else if (sel_c == STEPP) {
                motorsys.stepp.rpm.left = 0;
                motorsys.stepp.rpm.right = 0;
                motorsys.stepp.dir.left = STOP;
                motorsys.stepp.dir.right = STOP;
            } else if (sel_c == PITCH) {
                motorsys.pitch.length.left = motorsys.pitch.length.max;
                motorsys.pitch.length.right = motorsys.pitch.length.max;
            }
            break;
        case 'c':
            motorsys.loco.speed.left = 0;
            motorsys.loco.speed.right = 0;
            motorsys.stepp.rpm.left = 0;
            motorsys.stepp.rpm.right = 0;
            motorsys.stepp.dir.left = STOP;
            motorsys.stepp.dir.right = STOP;
            motorsys.pitch.length.left = PITCH_STOP;
            motorsys.pitch.length.right = PITCH_STOP;
            break;
        case 'q':
            motorsys.loco.speed.left = 0;
            motorsys.loco.speed.right = 0;
            motorsys.stepp.rpm.left = 0;
            motorsys.stepp.rpm.right = 0;
            motorsys.stepp.dir.left = STOP;
            motorsys.stepp.dir.right = STOP;
            motorsys.pitch.length.left = PITCH_STOP;
            motorsys.pitch.length.right = PITCH_STOP;
            endwin();
            return 0;

        default:
            // do nothing
            break;
        }
        send_targets();
    }
}

static void send_targets() {
    send_pitch(LEFT);
    send_pitch(RIGHT);
    send_loco(LEFT);
    send_loco(RIGHT);
    send_stepp(LEFT);
    send_stepp(RIGHT);
}

static void send_pitch(motor m) {
    motor_bridge::Pitch msg;
    msg.motor = (int)m;
    if (m == RIGHT) {
        msg.length = motorsys.pitch.length.right;
    } else {
        msg.length = motorsys.pitch.length.left;
    }
    pitch_pub.publish(msg);
}

static void send_loco(motor m) {
    motor_bridge::Drive msg;
    msg.motor = (int)m;
    if (m == RIGHT) {
        msg.speed = motorsys.loco.speed.right;
    } else {
        msg.speed = motorsys.loco.speed.left;
    }
    loco_pub.publish(msg);
}

static void send_stepp(motor m) {
    motor_bridge::Stepp msg;
    msg.motor = (int)m;
    if (m == RIGHT) {
        msg.rpm = motorsys.stepp.rpm.right;
        msg.direction = motorsys.stepp.dir.right;
    } else {
        msg.rpm = motorsys.stepp.rpm.left;
        msg.direction = motorsys.stepp.dir.left;
    }
    stepp_pub.publish(msg);
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
