#include <iostream>
#include "joystick/gamepad.hpp"
#include <string>

int main(int argc, char *argv[]) {
    GamepadHandler* g;
    g = new GamepadHandler("dev/input/js0", 1024, 100);
    delete g;
}
