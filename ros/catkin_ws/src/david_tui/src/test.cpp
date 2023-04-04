#include "joystick/gamepad.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    int max = 1024;
    GamepadHandler gh("/dev/input/js0", max, 100);
    
    while(true) {
        gh.update();
        std::cout << "\033[2J\033[1;1H";
        std::cout << gh.left_stick;
        int left = gh.left_stick.x + gh.left_stick.y;
        int right = gh.left_stick.y - gh.left_stick.x;
        left = (left > max) ? max : left;
        left = (left < -max) ? -max : left;
        right = (right > max) ? max : right;
        right = (right < -max) ? -max : right;
        std::cout << "left: " << left << ", right: " << right << std::endl;
    }
    return 0;
}
