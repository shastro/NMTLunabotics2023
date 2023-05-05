
/*
 * canshit.cpp
 * Can message generation helpers
 * Auto generated from KCD file
 */
#include "canshit.hpp"
#include "david.h"

int pack_msg(const motor_bridge::EStop& msg, uint8_t* buff) {
    david_e_stop_t t = {
        .stop = msg.stop,
    };

    david_e_stop_pack(buff, &t, 8);
    return DAVID_E_STOP_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchCtrl& msg, uint8_t* buff) {
    david_pitch_ctrl_t t = {
        .home = msg.home,
        .direction = msg.direction,
    };

    david_pitch_ctrl_pack(buff, &t, 8);
    return DAVID_PITCH_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchAdjust& msg, uint8_t* buff) {
    david_pitch_adjust_t t = {
        .motor = msg.motor,
        .dir = msg.dir,
        .amount = msg.amount,
    };

    david_pitch_adjust_pack(buff, &t, 8);
    return DAVID_PITCH_ADJUST_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchTelem& msg, uint8_t* buff) {
    david_pitch_telem_t t = {
        .left_count = msg.left_count,
        .left_direction = msg.left_direction,
        .right_count = msg.right_count,
        .right_direction = msg.right_direction,
    };

    david_pitch_telem_pack(buff, &t, 8);
    return DAVID_PITCH_TELEM_FRAME_ID;
}

int pack_msg(const motor_bridge::LocoCtrl& msg, uint8_t* buff) {
    david_loco_ctrl_t t = {
        .left_vel = msg.left_vel,
        .right_vel = msg.right_vel,
    };

    david_loco_ctrl_pack(buff, &t, 8);
    return DAVID_LOCO_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::ExcavCtrl& msg, uint8_t* buff) {
    david_excav_ctrl_t t = {
        .direction = msg.direction,
    };

    david_excav_ctrl_pack(buff, &t, 8);
    return DAVID_EXCAV_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperCtrl& msg, uint8_t* buff) {
    david_stepper_ctrl_t t = {
        .home = msg.home,
        .direction = msg.direction,
        .rpm = msg.rpm,
    };

    david_stepper_ctrl_pack(buff, &t, 8);
    return DAVID_STEPPER_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperCtrlAdjust& msg, uint8_t* buff) {
    david_stepper_ctrl_adjust_t t = {
        .motor = msg.motor,
        .dir = msg.dir,
        .amount = msg.amount,
    };

    david_stepper_ctrl_adjust_pack(buff, &t, 8);
    return DAVID_STEPPER_CTRL_ADJUST_FRAME_ID;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::EStop& msg) {
    s << "EStop" << " - ";
    s << "stop: " << msg.stop << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchCtrl& msg) {
    s << "PitchCtrl" << " - ";
    s << "home: " << msg.home << ", ";
    s << "direction: " << msg.direction << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchAdjust& msg) {
    s << "PitchAdjust" << " - ";
    s << "motor: " << msg.motor << ", ";
    s << "dir: " << msg.dir << ", ";
    s << "amount: " << msg.amount << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchTelem& msg) {
    s << "PitchTelem" << " - ";
    s << "left_count: " << msg.left_count << ", ";
    s << "left_direction: " << msg.left_direction << ", ";
    s << "right_count: " << msg.right_count << ", ";
    s << "right_direction: " << msg.right_direction << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::LocoCtrl& msg) {
    s << "LocoCtrl" << " - ";
    s << "left_vel: " << msg.left_vel << ", ";
    s << "right_vel: " << msg.right_vel << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::ExcavCtrl& msg) {
    s << "ExcavCtrl" << " - ";
    s << "direction: " << msg.direction << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::StepperCtrl& msg) {
    s << "StepperCtrl" << " - ";
    s << "home: " << msg.home << ", ";
    s << "direction: " << msg.direction << ", ";
    s << "rpm: " << msg.rpm << ", ";
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const motor_bridge::StepperCtrlAdjust& msg) {
    s << "StepperCtrlAdjust" << " - ";
    s << "motor: " << msg.motor << ", ";
    s << "dir: " << msg.dir << ", ";
    s << "amount: " << msg.amount << ", ";
    s << std::endl;
    return s;
}

