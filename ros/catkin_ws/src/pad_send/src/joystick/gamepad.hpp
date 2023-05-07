#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include "joystick.hpp"
#include <string>

enum class Button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    left_bumper = 4,
    right_bumper = 5,
    left_extra = 6,
    right_extra = 7,
    xbox = 8,
    left_thumb = 9,
    right_thumb = 10
};

enum class Axis {
    left_thumb_x = 0,
    left_thumb_y = 1,
    left_trigger = 2,
    right_thumb_x = 3,
    right_thumb_y = 4,
    right_trigger = 5,
    dpad_x = 6,
    dpad_y = 7,
};

struct Stick {
    int x;
    int y;

    friend std::ostream &operator<<(std::ostream &s, const Stick &t) {
        s << "stick x: " << t.x << ", y: " << t.y << std::endl;
        return s;
    }
};

struct Dpad {
    bool left;
    bool right;
    bool up;
    bool down;

    friend std::ostream &operator<<(std::ostream &s, const Dpad &t) {
        s << "dpad: ";
        if (t.left)
            s << "left ";
        if (t.right)
            s << "right ";
        if (t.up)
            s << "up ";
        if (t.down)
            s << "down ";
        s << std::endl;
        return s;
    }
};

struct Buttons {
    bool A;
    bool B;
    bool X;
    bool Y;
    bool left_bumper;
    bool right_bumper;
    bool xbox;
    bool left_extra;
    bool right_extra;
    bool left_thumb;
    bool right_thumb;

    friend std::ostream &operator<<(std::ostream &s, const Buttons &t) {
        s << "buttons: ";
        if (t.A)
            s << "A ";
        if (t.B)
            s << "B ";
        if (t.X)
            s << "X ";
        if (t.Y)
            s << "Y ";
        if (t.left_bumper)
            s << "LeftBumper ";
        if (t.right_bumper)
            s << "RightBumper ";
        if (t.xbox)
            s << "xbox ";
        if (t.left_extra)
            s << "Left Extra ";
        if (t.right_extra)
            s << "Right Extra ";
        s << std::endl;
        return s;
    }
};

class GamepadHandler{
    public:
        Stick left_stick;
        Stick right_stick;
        Dpad dpad;
        int left_trigger;
        int right_trigger;
        Buttons buttons;

        GamepadHandler(std::string pad_path, int sig_max, int dead);
        GamepadHandler(std::string pad_path, int sig_max, int dead, bool blocking);
        ~GamepadHandler();

        void update();

        friend std::ostream& operator<< (std::ostream& s, const GamepadHandler& gamepad);

    private:
        Joystick* j;
        int trig_min;
        int trig_max;
        int stick_min;
        int stick_max;
        int dead;

        void getLeftStick(JoystickEvent& event);
        void getRightStick(JoystickEvent& event);
        void getLeftTrigger(JoystickEvent& event);
        void getRightTrigger(JoystickEvent& event);
        void getDpad(JoystickEvent& event);
        void getButtons(JoystickEvent& event);

        int scale(int val, int old_min, int old_max, int new_min, int new_max);
};

#endif
