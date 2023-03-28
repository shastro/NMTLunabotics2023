#include <iostream>
#include <ncurses.h>

#include <motor_bridge/Digger.h>
#include <motor_bridge/Drive.h>
#include <motor_bridge/Estop.h>
#include <motor_bridge/Pitch.h>
#include <motor_bridge/Stepp.h>
#include <ros/ros.h>
#include <sstream>
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

enum class motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum class control { PITCH = 0, LOCO = 1, STEPP = 2 };
enum class direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

std::ostream &operator<<(std::ostream &s, control c) {
    switch (c) {
    case control::PITCH:
        s << "pitch";
        break;
    case control::LOCO:
        s << "loco";
        break;
    case control::STEPP:
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
    {'H', "Arm Out"},
    {'L', "Arm In"},
    {'s', "Stop Current"},
    {'c', "Stop All"},
    {'q', "Quit"},
};

static ros::Publisher pitch_pub;
static ros::Publisher loco_pub;
static ros::Publisher stepp_pub;
static ros::Publisher excav_pub;
static ros::Publisher estop_pub;

struct target {
    int left;
    int right;
    int min;
    int max;
    friend std::ostream &operator<<(std::ostream &s, const target &t) {
        s << "  left: " << t.left << "  right: " << t.right << std::endl
          << "  min: " << t.min << std::endl
          << "  max: " << t.max;
        return s;
    }
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

enum digger_target {
    DIG_STOP,
    DIG_FORWARD,
    DIG_BACKWARD,
};

static struct {
    pitch_target pitch;
    loco_target loco;
    stepp_target stepp;
    digger_target digger;
} motorsys;

static control sel_c;
static motor sel_m = motor::BOTH;

static void estop();
static void send_targets();
static void send_pitch();
static void send_loco(motor m);
static void send_stepp();
static void send_digger();

static void print_status();
static void print_keybinds();

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
    stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);
    excav_pub = nh.advertise<motor_bridge::Digger>("/excav_control", 5);

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

        switch (n) {
        case 'p': // Select Pitch
            sel_c = control::PITCH;
            break;
        case 'd': // Select Drive
            sel_c = control::LOCO;
            break;
        case 'a': // Select Excavation Arm
            sel_c = control::STEPP;
            break;

        case 'l': // Left
            sel_m = motor::LEFT;
            break;
        case 'r': // Right
            sel_m = motor::RIGHT;
            break;
        case 'b': // Both
            sel_m = motor::BOTH;
            if (sel_c == control::PITCH) {
                motorsys.pitch.length.left = motorsys.pitch.length.right;
            } else if (sel_c == control::LOCO) {
                motorsys.loco.speed.left = motorsys.loco.speed.right;
            } else if (sel_c == control::STEPP) {
                motorsys.stepp.rpm.left = motorsys.stepp.rpm.right;
                motorsys.stepp.dir.left = motorsys.stepp.dir.right;
            }
            break;

        case 'j': // Drive Forward
            // TODO implement changeable speed
            sel_c = control::LOCO;
            if (sel_m != motor::RIGHT) {
                motorsys.loco.speed.left = motorsys.loco.speed.max;
            }
            if (sel_m != motor::LEFT) {
                motorsys.loco.speed.right = motorsys.loco.speed.max;
            }
            break;
        case 'k': // Drive Backward
            sel_c = control::LOCO;
            if (sel_m != motor::RIGHT) {
                motorsys.loco.speed.left = motorsys.loco.speed.min;
            }
            if (sel_m != motor::LEFT) {
                motorsys.loco.speed.right = motorsys.loco.speed.min;
            }
            break;
        case 'J': // Pitch Down
            sel_c = control::PITCH;
            motorsys.pitch.length.left = PITCH_BACKWARD;
            motorsys.pitch.length.right = PITCH_BACKWARD;
            break;
        case 'K': // Pitch Up
            sel_c = control::PITCH;
            motorsys.pitch.length.left = PITCH_FORWARD;
            motorsys.pitch.length.right = PITCH_FORWARD;
            break;
        case 'H': // Depth out
            sel_c = control::STEPP;
            motorsys.stepp.rpm.left = motorsys.stepp.rpm.max;
            motorsys.stepp.dir.left = direction::FORWARD;
            motorsys.stepp.rpm.right = motorsys.stepp.rpm.max;
            motorsys.stepp.dir.right = direction::FORWARD;
            break;
        case 'L': // Depth in
            sel_c = control::STEPP;
            motorsys.stepp.rpm.left = motorsys.stepp.rpm.max;
            motorsys.stepp.dir.left = direction::BACKWARD;
            motorsys.stepp.rpm.right = motorsys.stepp.rpm.max;
            motorsys.stepp.dir.right = direction::BACKWARD;
            break;

        case 's': // Stop Current
            if (sel_c == control::LOCO) {
                motorsys.loco.speed.left = 0;
                motorsys.loco.speed.right = 0;
            } else if (sel_c == control::STEPP) {
                motorsys.stepp.rpm.left = 0;
                motorsys.stepp.rpm.right = 0;
                motorsys.stepp.dir.left = direction::STOP;
                motorsys.stepp.dir.right = direction::STOP;
            } else if (sel_c == control::PITCH) {
                motorsys.pitch.length.left = PITCH_STOP;
                motorsys.pitch.length.right = PITCH_STOP;
            }
            break;
        case 'c': // Stop All
            // estop();
            motorsys.loco.speed.left = 0;
            motorsys.loco.speed.right = 0;
            motorsys.stepp.rpm.left = 0;
            motorsys.stepp.rpm.right = 0;
            motorsys.stepp.dir.left = direction::STOP;
            motorsys.stepp.dir.right = direction::STOP;
            motorsys.pitch.length.left = PITCH_STOP;
            motorsys.pitch.length.right = PITCH_STOP;
            break;
        case 'q': // Quit
            estop();
            /*
            motorsys.loco.speed.left = 0;
            motorsys.loco.speed.right = 0;
            motorsys.stepp.rpm.left = 0;
            motorsys.stepp.rpm.right = 0;
            motorsys.stepp.dir.left = STOP;
            motorsys.stepp.dir.right = STOP;
            motorsys.pitch.length.left = PITCH_STOP;
            motorsys.pitch.length.right = PITCH_STOP;
            */
            endwin();
            return 0;

        default:
            // do nothing
            break;
        }
        send_targets();
    }
}

static void estop() {
    motor_bridge::Estop msg;
    msg.stop = true;
    estop_pub.publish(msg);
}

static void send_targets() {
    send_pitch();
    send_loco(motor::LEFT);
    send_loco(motor::RIGHT);
    send_stepp();
    send_digger();
}

static void send_pitch() {
    motor_bridge::Pitch msg;
    msg.motor = (int)motor::BOTH;
    // for now only the right side controls everything, because they
    // have to stay in sync.
    msg.length = motorsys.pitch.length.right;
    pitch_pub.publish(msg);
}

static void send_loco(motor m) {
    motor_bridge::Drive msg;
    msg.motor = (int)m;
    if (m == motor::RIGHT) {
        msg.speed = motorsys.loco.speed.right;
    } else {
        msg.speed = motorsys.loco.speed.left;
    }
    loco_pub.publish(msg);
}

static void send_stepp() {
    motor_bridge::Stepp msg;
    msg.motor = (int)motor::BOTH;
    msg.rpm = motorsys.stepp.rpm.right;
    msg.direction = (int)motorsys.stepp.dir.right;
    stepp_pub.publish(msg);
}

static void send_digger() {
    motor_bridge::Digger msg;
    msg.speed = 0;
    if (motorsys.digger == DIG_FORWARD)
        msg.speed = 100;
    else if (motorsys.digger == DIG_BACKWARD)
        msg.speed = -100;
    excav_pub.publish(msg);
}

static void print_status() {
    std::ostringstream status;
    status << "Pitch target: " << std::endl
           << motorsys.pitch.length << std::endl
           << "Loco target: " << std::endl
           << motorsys.loco.speed << std::endl
           << "Stepper target: " << std::endl
           << motorsys.stepp.rpm << std::endl
           << "Digger target: " << std::endl
           << "  " << motorsys.digger << std::endl
           << "Selected control: " << sel_c << "  Selected motor: " << sel_m
           << std::endl;
    printw("%s", status.str().c_str());
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
