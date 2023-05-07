#include <iostream>
#include "joystick/gamepad.hpp"

int main(int argc, char **argv) {
    GamepadHandler g("/dev/input/js0", 1024, 100);
    bool going = true;
    float pitch_max = 100;
    float pitch_min = 0;
    float pitch_point = 50.;
    float delta = 0.001 * pitch_max;
    while(true) {
        g.update();
        std::cout << "Here"  << std::endl;
    }
    return 0;
}
