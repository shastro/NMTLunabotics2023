#include <iostream>
#include <ncurses.h>
#include <motor_bridge/System.h>
#include <ros/ros.h>
#include <sstream>
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
    {'[', "Spin Left"},
    {'[', "Spin Right"},
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
static struct {
    pitch_target pitch;
    loco_target loco;
    stepp_target stepp;
    digger_target digger;
} motorsys;

static control sel_c;
static motor sel_m = motor::BOTH;

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
static void send_targets();
static void send_pitch();
static void send_loco(motor m);
static void send_stepp();
static void send_digger();
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
        case '[':
            sel_c = control::LOCO;
            motorsys.loco.speed.left = motorsys.loco.speed.max;
            motorsys.loco.speed.right = motorsys.loco.speed.min;
            break;
        case ']':
            sel_c = control::LOCO;
            motorsys.loco.speed.left = motorsys.loco.speed.min;
            motorsys.loco.speed.right = motorsys.loco.speed.max;
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
    for (auto bind : bindings) {
        print_bold("%c", std::get<0>(bind));
        printw(" : ");
        printw("%s", std::get<1>(bind).c_str());
        printw("\n");
    }
    refresh();
}

