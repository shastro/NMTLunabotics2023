#include <iostream>
#include <ncurses.h>

#include <motor_bridge/Pitch.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace std;

enum motor { BOTH = 0, LEFT = 1, RIGHT = 2 };

#define print_bold(...)                                                        \
    do {                                                                       \
        attron(A_BOLD);                                                        \
        printw(__VA_ARGS__);                                                   \
        attroff(A_BOLD);                                                       \
    } while (0)

int main(int argc, char **argv) {
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    // Refresh at 0.1-second interval.
    halfdelay(1);

    motor selected_motor = BOTH;
    int left_target = 0;
    int left_actual = 0;
    int right_target = 0;
    int right_actual = 0;

    while (true) {
        clear();
        move(0, 0);
        printw("Pitch motors:\n");

        if (selected_motor != RIGHT)
            print_bold("  > ");
        else
            print_bold("    ");
        print_bold("l");
        printw("eft:  target %d actual %d\n", left_target, left_actual);

        if (selected_motor != LEFT)
            print_bold("  > ");
        else
            print_bold("    ");
        print_bold("r");
        printw("ight: target %d actual %d\n", right_target, right_actual);

        printw("  Use ");
        print_bold("b");
        printw(" to select both.\n");

        printw("\n");

        printw("  Change pitch by 64: ");
        print_bold("[");
        printw(" decrease, ");
        print_bold("]");
        printw(" increase\n");

        printw("  Set pitch length: ");
        print_bold("{");
        printw(" zero, ");
        print_bold("}");
        printw(" max\n");

        refresh();

        int n = getch();

        switch (n) {
        case 'l':
            selected_motor = LEFT;
            break;
        case 'r':
            selected_motor = RIGHT;
            break;
        case 'b':
            selected_motor = BOTH;
            break;

        case '[':
            if (selected_motor != RIGHT) {
                left_target -= 64;
                left_target = max(left_target, 0);
            }
            if (selected_motor != LEFT) {
                right_target -= 64;
                right_target = max(right_target, 0);
            }
            break;
        case ']':
            if (selected_motor != RIGHT) {
                left_target += 64;
                left_target = min(left_target, 1024);
            }
            if (selected_motor != LEFT) {
                right_target += 64;
                right_target = min(right_target, 1024);
            }
            break;

        case '{':
            if (selected_motor != RIGHT)
                left_target = 0;
            if (selected_motor != LEFT)
                right_target = 0;
            break;
        case '}':
            if (selected_motor != RIGHT)
                left_target = 1024;
            if (selected_motor != LEFT)
                right_target = 1024;
            break;

        case 'q':
            endwin();
            return 0;

        default:
            // do nothing
            break;
        }
    }
}
