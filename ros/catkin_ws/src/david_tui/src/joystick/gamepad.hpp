#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include "joystick.hpp"
#include <string>

enum class Button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    leftBumper = 4,
    rightBumper = 5,
    leftExtra = 6,
    rightExtra = 7,
    xbox = 8,
    thumbLeft = 9,
    thumbRight = 10
};

enum class Axis {
    leftThumbX = 0,
    leftThumbY = 1,
    leftTrigger = 2,
    rightThumbX = 3,
    rightThumbY = 4,
    rightTrigger = 5,
    dpadX = 6,
    dpadY = 7,
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
    bool leftBumper;
    bool rightBumper;
    bool xbox;
    bool leftExtra;
    bool rightExtra;
    bool thumbLeft;
    bool thumbRight;

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
        if (t.leftBumper)
            s << "LeftBumper ";
        if (t.rightBumper)
            s << "RightBumper ";
        if (t.xbox)
            s << "xbox ";
        if (t.leftExtra)
            s << "Left Extra ";
        if (t.rightExtra)
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
        ~GamepadHandler();

        void update();

        friend std::ostream& operator<< (std::ostream& s, const GamepadHandler& gamepad);

    private:
        Joystick* j;
        int trig_min;
        int trig_max;
        int stick_min;
        int stick_max;
        int stick_dead;

        void getLeftStick(JoystickEvent& event);
        void getRightStick(JoystickEvent& event);
        void getLeftTrigger(JoystickEvent& event);
        void getRightTrigger(JoystickEvent& event);
        void getDpad(JoystickEvent& event);
        void getButtons(JoystickEvent& event);

        int scale(int val, int old_min, int old_max, int new_min, int new_max);
};

#endif
