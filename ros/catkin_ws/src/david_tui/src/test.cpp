#include <iostream>
#include <ncurses.h>
#include <unistd.h>
#include <vector>

int main(int argc, char *argv[]) {
    initscr();
    cbreak();
    keypad(stdscr, true);
    noecho();

    char n;
    while (true) {
        n = getch();
        if (n == 'f')
            printw("yes");
    }
}
