#include "gamepad.hpp"
#include <unistd.h>
#include <cmath>
#include <string>

GamepadHandler::GamepadHandler(std::string pad_path, int sig_max, int dead) {
    std::cout << "Connecting to joystick..." << std::endl;
    try {
        j = new Joystick(pad_path, true);
        std::cout << "awww yeahhhhh we good ... for now" << std::endl;
    } catch (std::string err) {
        std::cerr << "Connection timeout. Try plugging it in, dumbass" << std::endl;
    }
    this->stick_min = -1 * sig_max;
    this->stick_max = sig_max;
    this->stick_dead = dead;
    this->trig_min = 0;
    this->trig_max = sig_max;
}

GamepadHandler::~GamepadHandler() {
    delete j;
}

void  GamepadHandler::getLeftStick(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::leftThumbX) {
            this->left_stick.x = this->scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->left_stick.x) < this->stick_dead)
                this->left_stick.x = 0;
            return;
        } else if (axis == Axis::leftThumbY) {
            this->left_stick.y = -1 * this->scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->left_stick.y) < this->stick_dead)
                this->left_stick.y = 0;
            return;
        }
    }
}

void  GamepadHandler::getRightStick(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::rightThumbX) {
            this->right_stick.x = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->right_stick.x) < this->stick_dead)
                this->right_stick.x = 0;
            return;
        } else if (axis == Axis::rightThumbY) {
            this->right_stick.y = -1 * scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->right_stick.y) < this->stick_dead)
                this->right_stick.y = 0;
            return;
        }
    }
}

void GamepadHandler::getLeftTrigger(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::leftTrigger) {
            this->left_trigger = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->trig_min,
                    this->trig_max);
            return;
        }
    }
}

void GamepadHandler::getRightTrigger(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::rightTrigger) {
            this->right_trigger = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->trig_min,
                    this->trig_max);
            return;
        }
    }
}

void GamepadHandler::getDpad(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::dpadX) {
            if (event.value > 0) {
                this->dpad.left = true;
            } else if (event.value < 0) {
                this->dpad.right = true;
            } else if (event.value == 0) {
                this->dpad.left = false;
                this->dpad.right = false;
            }
        } else if (axis == Axis::dpadY) {
            if (event.value > 0) {
                this->dpad.down = true;
            } else if (event.value < 0) {
                this->dpad.up = true;
            } else if (event.value == 0) {
                this->dpad.up = false;
                this->dpad.down = false;
            }
        }
    }
}
void GamepadHandler::getButtons(JoystickEvent& event) {
    if (event.isButton()) {
        Button button = static_cast<Button>(event.number);
        switch(button) {
            case Button::A:
                if (event.value == 1)
                    this->buttons.A = true;
                else
                    this->buttons.A = false;
                break;
            case Button::B:
                if (event.value == 1)
                    this->buttons.B = true;
                else
                    this->buttons.B = false;
                break;
            case Button::X:
                if (event.value == 1)
                    this->buttons.X = true;
                else
                    this->buttons.X = false;
                break;
            case Button::Y:
                if (event.value == 1)
                    this->buttons.Y = true;
                else
                    this->buttons.Y = false;
                break;
            case Button::leftBumper:
                if (event.value == 1)
                    this->buttons.leftBumper = true;
                else
                    this->buttons.leftBumper = false;
                break;
            case Button::rightBumper:
                if (event.value == 1)
                    this->buttons.rightBumper = true;
                else
                    this->buttons.rightBumper = false;
                break;
            case Button::leftExtra:
                if (event.value == 1)
                    this->buttons.leftExtra = true;
                else
                    this->buttons.leftExtra = false;
                break;
            case Button::xbox:
                if (event.value == 1)
                    this->buttons.xbox = true;
                else
                    this->buttons.xbox = false;
                break;
            case Button::thumbLeft:
                if (event.value == 1)
                    this->buttons.thumbLeft = true;
                else
                    this->buttons.thumbLeft = false;
                break;
            case Button::thumbRight:
                if (event.value == 1)
                    this->buttons.thumbRight = true;
                else
                    this->buttons.thumbRight = false;
                break;
            default:
                break;
        }
    }
}

void GamepadHandler::update() {
    JoystickEvent e = this->j->sample();
    this->getLeftStick(e);
    this->getRightStick(e);
    this->getLeftTrigger(e);
    this->getRightTrigger(e);
    this->getDpad(e);
    this->getButtons(e);
}

int GamepadHandler::scale(int val, int old_min, int old_max, int new_min, int new_max) {
    return (((val - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
}

std::ostream& operator<< (std::ostream& s, const GamepadHandler& gamepad) {
    s << "left " << gamepad.left_stick;
    s << "right " << gamepad.right_stick;
    s << "ltrig: " << gamepad.left_trigger << std::endl;
    s << "rtrig: " << gamepad.right_trigger << std::endl;
    s << gamepad.dpad;
    s << gamepad.buttons;
    return s;
}
