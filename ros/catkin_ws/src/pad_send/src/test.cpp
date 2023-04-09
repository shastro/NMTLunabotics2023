#include <iostream>
#include "joystick/gamepad.hpp"

int main(int argc, char **argv) {
    GamepadHandler g("/dev/input/js0", 1024, 100);
    bool going = true;
    while(going) {
        g.update();
        std::cout << g << std::endl << std::endl;
        if (g.buttons.Y)
            going = false;
    }
    return 0;
}
