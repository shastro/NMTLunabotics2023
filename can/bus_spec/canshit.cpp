
/*
 * canshit.cpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include "canshit.hpp"
#include <ros/ros.h>

int pack_msg(const motor_bridge::EStop& msg, uint8_t* buff) {
    david_e_stop_t t = {
        .stop = david_e_stop_stop_encode(msg.stop),
    };

    david_e_stop_pack(buff, &t, sizeof(t));
    return DAVID_E_STOP_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchCtrl& msg, uint8_t* buff) {
    david_pitch_ctrl_t t = {
        .left = david_pitch_ctrl_left_encode(msg.left),
        .right = david_pitch_ctrl_right_encode(msg.right),
    };

    david_pitch_ctrl_pack(buff, &t, sizeof(t));
    return DAVID_PITCH_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchPositionTelem& msg, uint8_t* buff) {
    david_pitch_position_telem_t t = {
        .left_position = david_pitch_position_telem_left_position_encode(msg.left_position),
        .right_position = david_pitch_position_telem_right_position_encode(msg.right_position),
        .left_direction = david_pitch_position_telem_left_direction_encode(msg.left_direction),
        .right_direction = david_pitch_position_telem_right_direction_encode(msg.right_direction),
    };

    david_pitch_position_telem_pack(buff, &t, sizeof(t));
    return DAVID_PITCH_POSITION_TELEM_FRAME_ID;
}

int pack_msg(const motor_bridge::PitchDriverTelem& msg, uint8_t* buff) {
    david_pitch_driver_telem_t t = {
        .left_current = david_pitch_driver_telem_left_current_encode(msg.left_current),
        .right_current = david_pitch_driver_telem_right_current_encode(msg.right_current),
        .left_temperature = david_pitch_driver_telem_left_temperature_encode(msg.left_temperature),
        .right_temperature = david_pitch_driver_telem_right_temperature_encode(msg.right_temperature),
        .left_direction = david_pitch_driver_telem_left_direction_encode(msg.left_direction),
        .right_direction = david_pitch_driver_telem_right_direction_encode(msg.right_direction),
    };

    david_pitch_driver_telem_pack(buff, &t, sizeof(t));
    return DAVID_PITCH_DRIVER_TELEM_FRAME_ID;
}

int pack_msg(const motor_bridge::LocoCtrl& msg, uint8_t* buff) {
    david_loco_ctrl_t t = {
        .left_vel = david_loco_ctrl_left_vel_encode(msg.left_vel),
        .right_vel = david_loco_ctrl_right_vel_encode(msg.right_vel),
    };

    david_loco_ctrl_pack(buff, &t, sizeof(t));
    return DAVID_LOCO_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::ExcavCtrl& msg, uint8_t* buff) {
    david_excav_ctrl_t t = {
        .vel = david_excav_ctrl_vel_encode(msg.vel),
    };

    david_excav_ctrl_pack(buff, &t, sizeof(t));
    return DAVID_EXCAV_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperCtrl& msg, uint8_t* buff) {
    david_stepper_ctrl_t t = {
        .left = david_stepper_ctrl_left_encode(msg.left),
        .right = david_stepper_ctrl_right_encode(msg.right),
    };

    david_stepper_ctrl_pack(buff, &t, sizeof(t));
    return DAVID_STEPPER_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::StepperTelem& msg, uint8_t* buff) {
    david_stepper_telem_t t = {
        .at_min_stop = david_stepper_telem_at_min_stop_encode(msg.at_min_stop),
        .left_position = david_stepper_telem_left_position_encode(msg.left_position),
        .right_position = david_stepper_telem_right_position_encode(msg.right_position),
    };

    david_stepper_telem_pack(buff, &t, sizeof(t));
    return DAVID_STEPPER_TELEM_FRAME_ID;
}

int pack_msg(const motor_bridge::MastCtrl& msg, uint8_t* buff) {
    david_mast_ctrl_t t = {
        .direction = david_mast_ctrl_direction_encode(msg.direction),
    };

    david_mast_ctrl_pack(buff, &t, sizeof(t));
    return DAVID_MAST_CTRL_FRAME_ID;
}

int pack_msg(const motor_bridge::MastTelem& msg, uint8_t* buff) {
    david_mast_telem_t t = {
        .angle = david_mast_telem_angle_encode(msg.angle),
    };

    david_mast_telem_pack(buff, &t, sizeof(t));
    return DAVID_MAST_TELEM_FRAME_ID;
}

std::string printable(const motor_bridge::EStop& msg) {
    std::stringstream s;    s << "EStop - ";
    s << "stop: " << (int)msg.stop << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::PitchCtrl& msg) {
    std::stringstream s;    s << "PitchCtrl - ";
    s << "left: " << (int)msg.left << ", ";
    s << "right: " << (int)msg.right << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::PitchPositionTelem& msg) {
    std::stringstream s;    s << "PitchPositionTelem - ";
    s << "left_position: " << (int)msg.left_position << ", ";
    s << "right_position: " << (int)msg.right_position << ", ";
    s << "left_direction: " << (int)msg.left_direction << ", ";
    s << "right_direction: " << (int)msg.right_direction << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::PitchDriverTelem& msg) {
    std::stringstream s;    s << "PitchDriverTelem - ";
    s << "left_current: " << (int)msg.left_current << ", ";
    s << "right_current: " << (int)msg.right_current << ", ";
    s << "left_temperature: " << (int)msg.left_temperature << ", ";
    s << "right_temperature: " << (int)msg.right_temperature << ", ";
    s << "left_direction: " << (int)msg.left_direction << ", ";
    s << "right_direction: " << (int)msg.right_direction << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::LocoCtrl& msg) {
    std::stringstream s;    s << "LocoCtrl - ";
    s << "left_vel: " << (int)msg.left_vel << ", ";
    s << "right_vel: " << (int)msg.right_vel << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::ExcavCtrl& msg) {
    std::stringstream s;    s << "ExcavCtrl - ";
    s << "vel: " << (int)msg.vel << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::StepperCtrl& msg) {
    std::stringstream s;    s << "StepperCtrl - ";
    s << "left: " << (int)msg.left << ", ";
    s << "right: " << (int)msg.right << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::StepperTelem& msg) {
    std::stringstream s;    s << "StepperTelem - ";
    s << "at_min_stop: " << (int)msg.at_min_stop << ", ";
    s << "left_position: " << (int)msg.left_position << ", ";
    s << "right_position: " << (int)msg.right_position << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::MastCtrl& msg) {
    std::stringstream s;    s << "MastCtrl - ";
    s << "direction: " << (int)msg.direction << ", ";
    s << std::endl;
    return s.str();
}

std::string printable(const motor_bridge::MastTelem& msg) {
    std::stringstream s;    s << "MastTelem - ";
    s << "angle: " << (int)msg.angle << ", ";
    s << std::endl;
    return s.str();
}

