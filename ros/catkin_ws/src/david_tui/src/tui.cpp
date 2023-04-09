#include <iostream>
#include <ncurses.h>
/*
#include <motor_bridge/Digger.h>
#include <motor_bridge/Drive.h>
#include <motor_bridge/Pitch.h>
#include <motor_bridge/Stepp.h>
#include <motor_bridge/Estop.h>
*/
#include <motor_bridge/System.h>
#include <ros/ros.h>
//#include <std_msgs/String.h>
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
//enum control { PITCH = 0, DRIVE = 1, DEPTH = 2, DIGGER = 3 };
enum direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

// Print helpers
/*
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
   }*/

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

/*
// State holders
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
*/
// Selectors
//static control sel_c;
//static motor sel_m = BOTH;

// Keybind functions
/*
   static void sel_pitch() { sel_c = PITCH; }
   static void sel_drive() { sel_c = DRIVE; }
   static void sel_depth() { sel_c = DEPTH; }

   static void sel_both() { sel_m = BOTH; }
   static void sel_left() { sel_m = LEFT; }
   static void sel_right() { sel_m = RIGHT; }
   */

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

//static void stop_selected();
static void stop_all();
static void estop();
static void estart();
static void quit();

// Can handlers
static void send_targets();
/*
   static void send_pitch();
   static void send_loco(motor m);
   static void send_stepp();
   static void send_digger();
   */

// Display functions
static void print_status();
static void print_keybinds();

// Publishers
/*
   static ros::Publisher pitch_pub;
   static ros::Publisher loco_pub;
   static ros::Publisher stepp_pub;
   static ros::Publisher excav_pub;
   static ros::Publisher estop_pub;
   */
// Message
static motor_bridge::System s;
static ros::Publisher pub;


// Keybinds - char k, string description, void function
static std::tuple<char, std::string, std::function<void()>> bindings[] = {
    {'j', "Drive Both Forward", drive_forward},
    {'k', "Drive Both Backward", drive_backward},
    {'h', "Drive Right Motor Forward", drive_right_forward},
    {'l', "Drive Left Motor Forward", drive_left_forward},
    {'H', "Drive Right Motor Backward", drive_right_backward},
    {'L', "Drive Left Motor Backward", drive_left_backward},
    {'[', "Spin Right (Right backward, Left forward)", drive_spin_right},
    {']', "Spin Left (Right forward, Left backward)", drive_spin_left},
    {'J', "Pitch Down", pitch_down},
    {'K', "Pitch Up", pitch_up},
    {',', "Extend depth", depth_extend},
    {'.', "Retract depth", depth_retract},
    {'<', "Digger forward", digger_forward},
    {'>', "Digger backward", digger_backward},
    {'s', "Stop All", stop_all},
    {'q', "Emergency Stop", estop},
    {'r', "Restart", estart},
    {'Q', "Quit", quit}
};

//Max range of input/outputs
const static int max = 1024;
const static int min = -max;

// Loop condition
static bool still_going = true;

int main(int argc, char **argv) {
    // Setup ROS
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;

    /*
       pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
       loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);
       stepp_pub = nh.advertise<motor_bridge::Stepp>("/stepp_control", 5);
       excav_pub = nh.advertise<motor_bridge::Digger>("/excav_control", 5);
       estop_pub = nh.advertise<motor_bridge::Estop>("/estop", 5);
       */
    pub = nh.advertise<motor_bridge::System>("/system", 5);

    // Initialize min and max states
    /*
       motorsys.loco.min = -max;
       motorsys.loco.max = max;
       motorsys.stepp.min = 0;
       motorsys.stepp.max = max;
       motorsys.digger.min = -max;
       motorsys.digger.max = max;
       */

    // Setup ncurses.
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    // Refresh at 0.1-second interval.
    halfdelay(1);

    // input char
    int n;
    // Previous char, allows for 2 character commands
    int pn;

    while (still_going) {
        clear();
        move(0, 0);
        print_status();

        print_keybinds();
        pn = n;
        n = getch();
        for (auto bind : bindings)
            if (n == std::get<0>(bind))
                std::get<2>(bind)();

        pub.publish(s);
    }
    return 0;
}

static void drive_forward() {
    s.left.rpm = max;
    s.right.rpm = max;
}

static void drive_backward() {
    s.left.rpm = min;
    s.right.rpm = min;
}

static void drive_right_forward() {
    s.left.rpm = 0;
    s.right.rpm = max;
}

static void drive_left_forward() {
    s.left.rpm = max;
    s.right.rpm = 0;
}

static void drive_right_backward() {
    s.left.rpm = 0;
    s.right.rpm = min;
}

static void drive_left_backward() {
    s.left.rpm = min;
    s.right.rpm = 0;
}

static void drive_spin_right() {
    s.left.rpm = max;
    s.right.rpm = min;
}

static void drive_spin_left() {
    s.left.rpm = min;
    s.right.rpm = max;
}

static void drive_stop() {
    s.left.rpm = 0;
    s.right.rpm = 0;
}

static void pitch_down() {
    s.pitch.direction = BACKWARD;
    s.pitch.motor = BOTH;
}

static void pitch_up() {
    s.pitch.direction = FORWARD;
    s.pitch.motor = BOTH;
}

static void pitch_stop() {
    s.pitch.direction = STOP;
    s.pitch.motor = BOTH;
}

static void depth_extend() {
    s.extend.rpm = max;
    s.extend.direction = FORWARD;
    s.extend.motor = BOTH;
}

static void depth_retract() {
    s.extend.rpm = max;
    s.extend.direction = BACKWARD;
    s.extend.motor = BOTH;
}

static void depth_stop() {
    s.extend.rpm = 0;
    s.extend.direction = STOP;
    s.extend.motor = BOTH;
}

static void digger_forward() {
    s.digger.rpm = max;
}

static void digger_backward() {
    s.digger.rpm = min;
}

static void digger_stop() {
    s.digger.rpm = 0;
}

static void stop_all() {
    pitch_stop();
    depth_stop();
    drive_stop();
    digger_stop();
}

static void estop() {
    s.estop = true;
}

static void estart() {
    s.estop = false;
}

static void quit() {
    clear();
    estop();
    endwin();
    still_going = false;
}

/*
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

static void print_status() {
    std::stringstream ss;
    ss << "Drive left: " << s.left.rpm
        << ", right: " << s.right.rpm << std::endl;
    ss << "Digger: " << s.digger.rpm << std::endl;
    ss << "Extend dir: " << (direction) s.extend.direction
        << ", rpm: " << s.extend.rpm
        << ", motor: " << (motor) s.extend.motor << std::endl;
    ss << "Pitch dir: " << (direction) s.pitch.direction
        << ", motor: " << (motor) s.pitch.motor << std::endl;
    ss << "Estop" << s.estop << std::endl;
    printw(ss.str().c_str());
    refresh();
}

static void print_keybinds() {
    printw("Keybinds\n");
    for (auto bind : bindings) {
        print_bold("%c", std::get<0>(bind));
        printw(" : ");
        printw("%s", std::get<1>(bind).c_str());
        printw("\n");
    }
    refresh();
}

