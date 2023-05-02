
/*
 * canshit.cpp
 * Can message generation helpers
 * Auto generated from KCD file
 */
#include "canshit.hpp"

void to_bytes(int n, uint8_t* buff, int start) {
    if (start + sizeof(n) >= 8)
        throw string("Not enough space in can frame");
    memcpy(buff + start, &n, sizeof(n));
}

void pack_msg(motor_bridge::EStop msg, uint8_t* buff) {
   to_bytes(msg.stop, buff, 0);
}

void pack_msg(motor_bridge::PitchCtrl msg, uint8_t* buff) {
   to_bytes(msg.home, buff, 0);
   to_bytes(msg.direction, buff, 8);
}

void pack_msg(motor_bridge::PitchAdjust msg, uint8_t* buff) {
   to_bytes(msg.motor, buff, 0);
   to_bytes(msg.dir, buff, 8);
   to_bytes(msg.amount, buff, 16);
}

void pack_msg(motor_bridge::PitchTelem msg, uint8_t* buff) {
   to_bytes(msg.left_count, buff, 0);
   to_bytes(msg.left_direction, buff, 16);
   to_bytes(msg.right_count, buff, 32);
   to_bytes(msg.right_direction, buff, 48);
}

void pack_msg(motor_bridge::LocoCtrl msg, uint8_t* buff) {
   to_bytes(msg.left_vel, buff, 0);
   to_bytes(msg.right_vel, buff, 32);
}

void pack_msg(motor_bridge::ExcavCtrl msg, uint8_t* buff) {
   to_bytes(msg.direction, buff, 0);
}

void pack_msg(motor_bridge::StepperCtrl msg, uint8_t* buff) {
   to_bytes(msg.home, buff, 0);
   to_bytes(msg.direction, buff, 8);
   to_bytes(msg.r_p_m, buff, 16);
}

void pack_msg(motor_bridge::StepperCtrlAdjust msg, uint8_t* buff) {
   to_bytes(msg.motor, buff, 0);
   to_bytes(msg.dir, buff, 8);
   to_bytes(msg.amount, buff, 16);
}

