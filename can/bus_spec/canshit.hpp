
/*
 * canshit.hpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include <iostream>
#include "david.h"

#include <motor_bridge/EStop.h>
#include <motor_bridge/PitchCtrl.h>
#include <motor_bridge/PitchPositionTelem.h>
#include <motor_bridge/PitchDriverTelem.h>
#include <motor_bridge/LocoCtrl.h>
#include <motor_bridge/ExcavCtrl.h>
#include <motor_bridge/StepperCtrl.h>
#include <motor_bridge/StepperTelem.h>
#include <motor_bridge/MastCtrl.h>
#include <motor_bridge/MastTelem.h>

int pack_msg(const motor_bridge::EStop& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchPositionTelem& msg, uint8_t* buff);
int pack_msg(const motor_bridge::PitchDriverTelem& msg, uint8_t* buff);
int pack_msg(const motor_bridge::LocoCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::ExcavCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::StepperCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::StepperTelem& msg, uint8_t* buff);
int pack_msg(const motor_bridge::MastCtrl& msg, uint8_t* buff);
int pack_msg(const motor_bridge::MastTelem& msg, uint8_t* buff);

std::string printable(const motor_bridge::EStop& msg);
std::string printable(const motor_bridge::PitchCtrl& msg);
std::string printable(const motor_bridge::PitchPositionTelem& msg);
std::string printable(const motor_bridge::PitchDriverTelem& msg);
std::string printable(const motor_bridge::LocoCtrl& msg);
std::string printable(const motor_bridge::ExcavCtrl& msg);
std::string printable(const motor_bridge::StepperCtrl& msg);
std::string printable(const motor_bridge::StepperTelem& msg);
std::string printable(const motor_bridge::MastCtrl& msg);
std::string printable(const motor_bridge::MastTelem& msg);
