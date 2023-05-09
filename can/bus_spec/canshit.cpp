
/*
 * canshit.cpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include "canshit.hpp"
#include <ros/ros.h>

int pack_msg(const motor_bridge::EStop& msg, uint8_t* buff) {
    david_e_stop_t t = {
        .stop = msg.stop,
    };

    david_e_stop_pack(buff, &t, 8);
    return DAVID_E_STOP_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchCtrl& msg, uint8_t* buff) {
    david_pitch_ctrl_t t = {
        .set_point = msg.set_point,
        .left_offset = msg.left_offset,
        .right_offset = msg.right_offset,
    };

    david_pitch_ctrl_pack(buff, &t, 8);
    return DAVID_PITCH_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchPositionTelem& msg, uint8_t* buff) {
    david_pitch_position_telem_t t = {
        .left_position = msg.left_position,
        .right_position = msg.right_position,
    };

    david_pitch_position_telem_pack(buff, &t, 8);
    return DAVID_PITCH_POSITION_TELEM_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchDriverTelem& msg, uint8_t* buff) {
    david_pitch_driver_telem_t t = {
        .left_current = msg.left_current,
        .right_current = msg.right_current,
        .left_temperature = msg.left_temperature,
        .right_temperature = msg.right_temperature,
        .direction = msg.direction,
    };

    david_pitch_driver_telem_pack(buff, &t, 8);
    return DAVID_PITCH_DRIVER_TELEM_FRAME_ID;
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
        .vel = msg.vel,
    };

    david_excav_ctrl_pack(buff, &t, 8);
    return DAVID_EXCAV_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperCtrl& msg, uint8_t* buff) {
    david_stepper_ctrl_t t = {
        .home = msg.home,
        .set_point = msg.set_point,
    };

    david_stepper_ctrl_pack(buff, &t, 8);
    return DAVID_STEPPER_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperTelem& msg, uint8_t* buff) {
    david_stepper_telem_t t = {
        .at_min_stop = msg.at_min_stop,
        .at_max_stop = msg.at_max_stop,
        .left_position = msg.left_position,
        .right_position = msg.right_position,
    };

    david_stepper_telem_pack(buff, &t, 8);
    return DAVID_STEPPER_TELEM_FRAME_ID;
}

std::string printable(motor_bridge::EStop& msg) {
    s << "EStop - ";
    s << "stop: " << (int)msg.stop << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::PitchCtrl& msg) {
    s << "PitchCtrl - ";
    s << "set_point: " << (int)msg.set_point << ", ";
    s << "left_offset: " << (int)msg.left_offset << ", ";
    s << "right_offset: " << (int)msg.right_offset << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::PitchPositionTelem& msg) {
    s << "PitchPositionTelem - ";
    s << "left_position: " << (int)msg.left_position << ", ";
    s << "right_position: " << (int)msg.right_position << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::PitchDriverTelem& msg) {
    s << "PitchDriverTelem - ";
    s << "left_current: " << (int)msg.left_current << ", ";
    s << "right_current: " << (int)msg.right_current << ", ";
    s << "left_temperature: " << (int)msg.left_temperature << ", ";
    s << "right_temperature: " << (int)msg.right_temperature << ", ";
    s << "direction: " << (int)msg.direction << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::LocoCtrl& msg) {
    s << "LocoCtrl - ";
    s << "left_vel: " << (int)msg.left_vel << ", ";
    s << "right_vel: " << (int)msg.right_vel << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::ExcavCtrl& msg) {
    s << "ExcavCtrl - ";
    s << "vel: " << (int)msg.vel << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::StepperCtrl& msg) {
    s << "StepperCtrl - ";
    s << "home: " << (int)msg.home << ", ";
    s << "set_point: " << (int)msg.set_point << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(motor_bridge::StepperTelem& msg) {
    s << "StepperTelem - ";
    s << "at_min_stop: " << (int)msg.at_min_stop << ", ";
    s << "at_max_stop: " << (int)msg.at_max_stop << ", ";
    s << "left_position: " << (int)msg.left_position << ", ";
    s << "right_position: " << (int)msg.right_position << ", ";
    s << std::endl;
    return s.str();
}

