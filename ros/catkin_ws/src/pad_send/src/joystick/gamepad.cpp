#include "gamepad.hpp"
#include <unistd.h>
#include <cmath>
#include <string>

GamepadHandler::GamepadHandler(std::string pad_path, int sig_max, int dead) :
    GamepadHandler(pad_path, sig_max, dead, false) {}

GamepadHandler::GamepadHandler(std::string pad_path, int sig_max, int dead, bool blocking) {
    std::cout << "Connecting to joystick..." << std::endl;
    try {
        j = new Joystick(pad_path, blocking);
        std::cout << "awww yeahhhhh we good ... for now" << std::endl;
    } catch (std::string err) {
        std::cerr << "Connection timeout. Try plugging it in, dumbass" << std::endl;
    }
    this->stick_min = -1 * sig_max;
    this->stick_max = sig_max;
    this->trig_min = 0;
    this->trig_max = sig_max;
    this->dead = dead;
}

GamepadHandler::~GamepadHandler() {
    delete j;
}

void  GamepadHandler::getLeftStick(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::left_thumb_x) {
            this->left_stick.x = this->scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->left_stick.x) < this->dead)
                this->left_stick.x = 0;
            return;
        } else if (axis == Axis::left_thumb_y) {
            this->left_stick.y = -1 * this->scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->left_stick.y) < this->dead)
                this->left_stick.y = 0;
            return;
        }
    }
}

void  GamepadHandler::getRightStick(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::right_thumb_x) {
            this->right_stick.x = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->right_stick.x) < this->dead)
                this->right_stick.x = 0;
            return;
        } else if (axis == Axis::right_thumb_y) {
            this->right_stick.y = -1 * scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->stick_min,
                    this->stick_max);
            if (abs(this->right_stick.y) < this->dead)
                this->right_stick.y = 0;
            return;
        }
    }
}

void GamepadHandler::getLeftTrigger(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::left_trigger) {
            this->left_trigger = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->trig_min,
                    this->trig_max);
            this->left_trigger = (this->left_trigger > this->dead) ? this->left_trigger : 0;
            return;
        }
    }
}

void GamepadHandler::getRightTrigger(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::right_trigger) {
            this->right_trigger = scale(event.value,
                    event.MIN_AXES_VALUE,
                    event.MAX_AXES_VALUE,
                    this->trig_min,
                    this->trig_max);
            this->right_trigger = (this->right_trigger > this->dead) ? this->right_trigger : 0;
            return;
        }
    }
}

void GamepadHandler::getDpad(JoystickEvent& event) {
    if (event.isAxis()) {
        Axis axis = static_cast<Axis>(event.number);
        if (axis == Axis::dpad_x) {
            if (event.value > this->dead) {
                this->dpad.left = true;
            } else if (event.value < -this->dead) {
                this->dpad.right = true;
            } else if (event.value > -this->dead && event.value < this->dead) {
                this->dpad.left = false;
                this->dpad.right = false;
            }
        } else if (axis == Axis::dpad_y) {
            if (event.value > this->dead) {
                this->dpad.down = true;
            } else if (event.value < -this->dead) {
                this->dpad.up = true;
            } else if (event.value > -this->dead && event.value < this->dead) {
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
            case Button::left_bumper:
                if (event.value == 1)
                    this->buttons.left_bumper = true;
                else
                    this->buttons.left_bumper = false;
                break;
            case Button::right_bumper:
                if (event.value == 1)
                    this->buttons.right_bumper = true;
                else
                    this->buttons.right_bumper = false;
                break;
            case Button::left_extra:
                if (event.value == 1)
                    this->buttons.left_extra = true;
                else
                    this->buttons.left_extra = false;
                break;
            case Button::xbox:
                if (event.value == 1)
                    this->buttons.xbox = true;
                else
                    this->buttons.xbox = false;
                break;
            case Button::left_thumb:
                if (event.value == 1)
                    this->buttons.left_thumb = true;
                else
                    this->buttons.left_thumb = false;
                break;
            case Button::right_thumb:
                if (event.value == 1)
                    this->buttons.right_thumb = true;
                else
                    this->buttons.right_thumb = false;
                break;
            default:
                break;
        }
    }
}

void GamepadHandler::update() {
    JoystickEvent e;
    if (this->j->sample(&e)) {
        this->getLeftStick(e);
        this->getRightStick(e);
        this->getLeftTrigger(e);
        this->getRightTrigger(e);
        this->getDpad(e);
        this->getButtons(e);
    }
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
