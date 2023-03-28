#include <iostream>
#include <ncurses.h>

#include <motor_bridge/Pitch.h>
#include <motor_bridge/Drive.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace std;

#define print_bold(...)                                                        \
    do {                                                                       \
        attron(A_BOLD);                                                        \
        printw(__VA_ARGS__);                                                   \
        attroff(A_BOLD);                                                       \
    } while (0)


enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum control { PITCH = 0, LOCO = 1, STEPP = 2};
enum direction { BACKWARD = -1, STOP = 0. FORWARD = 1};

static std::unordered_map<char, std::string> bindings;

//Control
bindings['p'] = "Pitch";
bindings['d'] = "Drive";
bindings['a'] = "Excavation Arm";

//Motor
bindings['b'] = "Both";
bindings['l'] = "Left";
bindings['r'] = "Right";

//Command
bindings['f'] = "Drive Forward";
bindings['r'] = "Drive Backward";
bindings['u'] = "Pitch Up";
bindings['d'] = "Pitch Down";
bindings['o'] = "Arm Out";
bindings['i'] = "Arm In";
bindings['s'] = "Stop Current";
bindings['c'] = "Stop All";
bindings['q'] = "Quit";

static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;

struct target {
    int left;
    int right;
    int min;
    int max;
}

struct target_dir {
    direction left;
    direction right;
}

struct pitch {
    target length;
};

struct loco {
    target speed;
};

struct stepp {
    target rpm;
    target_dir dir;
};

static struct {
    pitch pitch
        loco loco;
    stepp stepp;
} system;

static control sel_c;
static motor sel_m = BOTH;

static void send_pitch();
static void send_loco();
static void send_stepp();

static void print_keybinds();

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);

    // Setup system
    system.pitch.length.min = 0;
    system.pitch.length.max = 1024;
    system.loco.speed.min = -1024;
    system.loco.speed.max = 1024;
    system.stepp.rpm.min = 0;
    system.stepp.rpm.max = 1024;

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
                    system.pitch.length.left = system.pitch.length.right;
                } else if (sel_c == LOCO) {
                    system.loco.speed.left = system.loco.speed.right;
                } else if (sel_c == STEPP) {
                    system.stepp.rpm.left = system.stepp.rpm.right;
                    system.stepp.dir.left = system.stepp.dir.right;
                }
                break;

            case 'f':
                //TODO implement changeable speed
                sel_c = LOCO;
                if (sel_m != RIGHT) {
                    system.loco.speed.left = system.loco.speed.max;
                } else if (sel_m != LEFT) {
                    system.loco.speed.right = system.loco.speed.max;
                }
                break;
            case 'r':
                sel_c = LOCO;
                if (sel_m != RIGHT) {
                    system.loco.speed.left = system.loco.speed.min;
                } else if (sel_m != LEFT) {
                    system.loco.speed.right = system.loco.speed.min;
                }
                break;
            case 'u':
                sel_c = PITCH;
                if (sel_m != RIGHT) {
                    system.pitch.length.left = min(system.pitch.length.left + 64,
                            system.pitch.length.max);
                } else if (sel_m != LEFT) {
                    system.pitch.length.right = min(system.pitch.length.right + 64,
                            system.pitch.length.max);
                }
                break;
            case 'd':
                sel_c = PITCH;
                if (sel_m != RIGHT) {
                    system.pitch.length.left = max(system.pitch.length.left - 64,
                            system.pitch.length.min);
                } else if (sel_m != LEFT) {
                    system.pitch.length.right = max(system.pitch.length.right - 64,
                            system.pitch.length.min);
                }
                break;
            case 'o':
                sel_c = STEPP;
                if (sel_m != RIGHT) {
                    system.stepp.rpm.left = system.stepp.rpm.max;
                    system.stepp.dir.left = FORWARD;
                } else if (sel_m != LEFT) {
                    system.stepp.rpm.right = system.stepp.rpm.max;
                    system.stepp.dir.right = FORWARD;
                }
                break;
            case 'i':
                sel_c = STEPP;
                if (sel_m != RIGHT) {
                    system.stepp.rpm.left = system.stepp.rpm.max;
                    system.stepp.dir.left = BACKWARD;
                } else if (sel_m != LEFT) {
                    system.stepp.rpm.right = system.stepp.rpm.max;
                    system.stepp.dir.right = BACKWARD;
                }
                break;

            case 's':
                if (sel_c == LOCO) {
                    system.loco.speed.left = 0;
                    system.loco.speed.right = 0;
                } else if (sel_c == STEPP) {
                    system.stepp.rpm.left = 0;
                    system.stepp.rpm.right = 0;
                    system.stepp.dir.left = STOP;
                    system.stepp.dir.right = STOP;
                } else if (sel_c == PITCH) {
                    system.pitch.length.left = system.pitch.length.max;
                    system.pitch.length.right = system.pitch.length.max;
                }
                break;
            case 'c':
                system.loco.speed.left = 0;
                system.loco.speed.right = 0;
                system.stepp.rpm.left = 0;
                system.stepp.rpm.right = 0;
                system.stepp.dir.left = STOP;
                system.stepp.dir.right = STOP;
                system.pitch.length.left = system.pitch.length.max;
                system.pitch.length.right = system.pitch.length.max;
                break;
            case 'q':
                system.loco.speed.left = 0;
                system.loco.speed.right = 0;
                system.stepp.rpm.left = 0;
                system.stepp.rpm.right = 0;
                system.stepp.dir.left = STOP;
                system.stepp.dir.right = STOP;
                system.pitch.length.left = system.pitch.length.max;
                system.pitch.length.right = system.pitch.length.max;
                endwin();
                return 0;

            default:
                // do nothing
                break;
        }
        send_targets();
    }
}

static void send_target() {
    send_pitch(LEFT);
    send_pitch(RIGHT);
    send_loco(LEFT);
    send_loco(RIGHT);
    send_stepp(LEFT);
    send_stepp(RIGHT);
}

static void send_pitch(motor m) {
    motor_bridge::Pitch msg;
    msg.motor = (int) m;
    if (m == RIGHT)
        msg.length = system.pitch.length.right;
    else
        msg.length = system.pitch.length.left;
    pitch_pub.publish(msg);
}

static void send_loco(motor m) {
    motor_bridge::Drive msg;
    msg.motor = (int) m;
    if (m == RIGHT)
        msg.speed = system.loco.speed.right;
    else
        msg.speed = system.loco.speed.left;
    loco_pub.publish(msg);
}

static void send_stepp(motor m) {
    motor_bridge::Stepp msg;
    msg.motor = (int) m;
    if (m == RIGHT)
        msg.rpm = system.stepp.rpm.right;
        msg.direction = system.stepp.dir.right;
    else
        msg.rpm = system.stepp.rpm.left;
        msg.direction = system.stepp.dir.left;
    stepp_pub.publish(msg);
}

static void print_keybinds() {
    printw("Keybinds\n");
    for (auto k : keybinds) {
        std::string c{k.first};
        print_bold(c);
        printw(" : ");
        printw(k.second);
        printw("\n");
    }
}
