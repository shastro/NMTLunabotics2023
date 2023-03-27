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

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };
enum control { PITCH = 0, LOCO = 1};

static void send_targets(control c, motor m, int target);

#define print_bold(...)                                                        \
    do {                                                                       \
        attron(A_BOLD);                                                        \
        printw(__VA_ARGS__);                                                   \
        attroff(A_BOLD);                                                       \
    } while (0)

ros::Publisher pitch_pub;
ros::Publisher loco_pub;

int left_pitch_target = 0;
int left_pitch_actual = 0;
int right_pitch_target = 0;
int right_pitch_actual = 0;
int left_loco_target = 0;
int right_loco_target = 0;
int pitch_min = 0;
int pitch_max = 1024;
int loco_min = -1024;
int loco_max = 1024;

int main(int argc, char **argv) {
    ros::init(argc, argv, "david_tui");
    ros::NodeHandle nh;
    pitch_pub = nh.advertise<motor_bridge::Pitch>("/pitch_control", 5);
    loco_pub = nh.advertise<motor_bridge::Drive>("/loco_control", 5);

    // Setup ncurses.
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    // Refresh at 0.1-second interval.
    halfdelay(1);

    motor selected_motor = BOTH;
    control selected_control = PITCH;

    while (true) {
        clear();
        move(0, 0);

        int print_left_target = 0;
        int print_right_target = 0;
        int print_left_actual = 0;
        int print_right_actual = 0;
        if (selected_control == PITCH) {
            printw("Pitch motors:\n");
            print_left_target = left_pitch_target;
            print_right_target = right_pitch_target;
            print_left_actual = left_pitch_actual;
            print_right_actual = right_pitch_actual;
        } else if (selected_control == LOCO) {
            printw("Drive motors:\n");
            print_left_target = left_loco_target;
            print_right_target = right_loco_target;
            print_left_actual = left_loco_target;
            print_right_actual = right_loco_target;
        }

        if (selected_motor != RIGHT || selected_motor == BOTH)
            print_bold("  > ");
        else
            print_bold("    ");
        print_bold("l");
        printw("eft:  target %d actual %d\n", print_left_target, print_left_actual);

        if (selected_motor != LEFT || selected_motor == BOTH)
            print_bold("  > ");
        else
            print_bold("    ");
        print_bold("r");
        printw("ight: target %d actual %d\n", print_right_target, print_right_actual);

        printw("  Use ");
        print_bold("p");
        printw(" to select pitch, ");
        print_bold("d");
        printw(" to select drive");
        printw("\n");

        printw("  Use ");
        print_bold("b");
        printw(" to select both, ");
        print_bold("r");
        printw(" to select right, ");
        print_bold("l");
        printw(" to select left");
        printw("\n");

        printw("  Change target by 64: ");
        print_bold("[");
        printw(" decrease, ");
        print_bold("]");
        printw(" increase");
        printw("\n");

        printw("  Set target length: ");
        print_bold("{");
        printw(" min, ");
        print_bold("}");
        printw(" max");
        printw("\n");

        printw("  Quit: ");
        print_bold("q");
        printw("\n");

        refresh();

        int n = getch();

        switch (n) {
            case 'p':
                selected_control = PITCH;
                break;
            case 'd':
                selected_control = LOCO;
                break;

            case 'l':
                selected_motor = LEFT;
                break;
            case 'r':
                selected_motor = RIGHT;
                break;
            case 'b':
                selected_motor = BOTH;
                if (selected_control == PITCH) {
                    left_pitch_target = right_pitch_target;
                } else if (selected_control == LOCO) {
                    left_loco_target = right_loco_target;
                }
                break;

            case '[':
                if (selected_control == PITCH) {
                    if (selected_motor != RIGHT) {
                        left_pitch_target -= 64;
                        left_pitch_target = max(left_pitch_target, pitch_min);
                    }
                    if (selected_motor != LEFT) {
                        right_pitch_target -= 64;
                        right_pitch_target = max(right_pitch_target, pitch_min);
                    }
                } else if (selected_control == LOCO) {
                    if (selected_motor != RIGHT) {
                        left_loco_target -= 64;
                        left_loco_target = max(left_loco_target, loco_min);
                    }
                    if (selected_motor != LEFT) {
                        right_loco_target -= 64;
                        right_loco_target = max(right_loco_target, loco_min);
                    }
                }
                break;
            case ']':
                if (selected_control == PITCH) {
                    if (selected_motor != RIGHT) {
                        left_pitch_target += 64;
                        left_pitch_target = min(left_pitch_target, pitch_max);
                    }
                    if (selected_motor != LEFT) {
                        right_pitch_target += 64;
                        right_pitch_target = min(right_pitch_target, pitch_max);
                    }
                } else if (selected_control == LOCO) {
                    if (selected_motor != RIGHT) {
                        left_loco_target += 64;
                        left_loco_target = min(left_loco_target, loco_max);
                    }
                    if (selected_motor != LEFT) {
                        right_loco_target += 64;
                        right_loco_target = min(right_loco_target, loco_max);
                    }
                }
                break;

            case '{':
                if (selected_control == PITCH) {
                    if (selected_motor != RIGHT)
                        left_pitch_target = pitch_min;
                    if (selected_motor != LEFT)
                        right_pitch_target = pitch_min;
                } else if (selected_control == LOCO) {
                    if (selected_motor != RIGHT)
                        left_loco_target = loco_min;
                    if (selected_motor != LEFT)
                        right_loco_target = loco_min;
                }
                break;
            case '}':
                if (selected_control == PITCH) {
                    if (selected_motor != RIGHT)
                        left_pitch_target = pitch_max;
                    if (selected_motor != LEFT)
                        right_pitch_target = pitch_max;
                } else if (selected_control == LOCO) {
                    if (selected_motor != RIGHT)
                        left_loco_target = loco_max;
                    if (selected_motor != LEFT)
                        right_loco_target = loco_max;
                }
                break;

            case 'q':
                send_targets(PITCH, BOTH, 0);
                send_targets(LOCO, BOTH, 0);
                endwin();
                return 0;

            default:
                // do nothing
                break;
        }

        int target;

        if (selected_control == PITCH) {
            if (selected_motor == LEFT)
                target = left_pitch_target;
            else
                target = right_pitch_target;
        } else if (selected_control == LOCO) {
            if (selected_motor == LEFT)
                target = left_loco_target;
            else
                target = right_loco_target;
        }

        send_targets(selected_control, selected_motor, target);
    }
}

static void send_targets(control c, motor m, int target) {
    if (c == PITCH) {
        motor_bridge::Pitch p;
        p.length = target;
        p.motor = (int) m;
        pitch_pub.publish(p);
    } else if (c == LOCO) {
        motor_bridge::Drive d;
        d.speed = target;
        if (m == BOTH) {
            d.motor = (int) LEFT;
            loco_pub.publish(d);
            d.motor = (int) RIGHT;
            loco_pub.publish(d);
        } else {
            d.motor = m;
            loco_pub.publish(d);
        }
    }
}

