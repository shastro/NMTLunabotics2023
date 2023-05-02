
/*
 * canshit.hpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include <iostream>

void pack_msg(motor_bridge::EStop msg, uint8_t* buff);
void pack_msg(motor_bridge::PitchCtrl msg, uint8_t* buff);
void pack_msg(motor_bridge::PitchAdjust msg, uint8_t* buff);
void pack_msg(motor_bridge::PitchTelem msg, uint8_t* buff);
void pack_msg(motor_bridge::LocoCtrl msg, uint8_t* buff);
void pack_msg(motor_bridge::ExcavCtrl msg, uint8_t* buff);
void pack_msg(motor_bridge::StepperCtrl msg, uint8_t* buff);
void pack_msg(motor_bridge::StepperCtrlAdjust msg, uint8_t* buff);
