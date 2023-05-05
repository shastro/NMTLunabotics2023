
/*
 * canshit.hpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include <iostream>
#include <ros/ros.h>
#include <motor_bridge/EStop.h>
#include <motor_bridge/PitchCtrl.h>
#include <motor_bridge/PitchAdjust.h>
#include <motor_bridge/PitchTelem.h>
#include <motor_bridge/LocoCtrl.h>
#include <motor_bridge/ExcavCtrl.h>
#include <motor_bridge/StepperCtrl.h>
#include <motor_bridge/StepperCtrlAdjust.h>

int pack_msg(const motor_bridge::EStop& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchAdjust& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchTelem& msg, uint8_t* buff);
int pack_msg(const motor_bridge::LocoCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::ExcavCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::StepperCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::StepperCtrlAdjust& msg, uint8_t* buff);

std::ostream& operator<<(std::ostream& s, const motor_bridge::EStop& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchCtrl& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchAdjust& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::PitchTelem& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::LocoCtrl& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::ExcavCtrl& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::StepperCtrl& msg);
std::ostream& operator<<(std::ostream& s, const motor_bridge::StepperCtrlAdjust& msg);
