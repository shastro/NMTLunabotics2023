
/*
 * unpackers.cpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include <unpackers.hpp>

void unpackEStop(int can_id, uint8_t* buff, int8* stop){
    if (can_id == DAVID_EStop_FRAME_ID) {
        david_e_stop_t t;
        david_e_stop_unpack(&t, buff, 8);
        *stop = t.stop;
   }
}

void unpackPitchCtrl(int can_id, uint8_t* buff, int16* set_point, int16* left_offset, int16* right_offset){
    if (can_id == DAVID_PitchCtrl_FRAME_ID) {
        david_pitch_ctrl_t t;
        david_pitch_ctrl_unpack(&t, buff, 8);
        *set_point = t.set_point;
        *left_offset = t.left_offset;
        *right_offset = t.right_offset;
   }
}

void unpackPitchPositionTelem(int can_id, uint8_t* buff, int16* left_position, int16* right_position){
    if (can_id == DAVID_PitchPositionTelem_FRAME_ID) {
        david_pitch_position_telem_t t;
        david_pitch_position_telem_unpack(&t, buff, 8);
        *left_position = t.left_position;
        *right_position = t.right_position;
   }
}

void unpackPitchDriverTelem(int can_id, uint8_t* buff, int8* left_current, int8* right_current, int8* left_temperature, int8* right_temperature){
    if (can_id == DAVID_PitchDriverTelem_FRAME_ID) {
        david_pitch_driver_telem_t t;
        david_pitch_driver_telem_unpack(&t, buff, 8);
        *left_current = t.left_current;
        *right_current = t.right_current;
        *left_temperature = t.left_temperature;
        *right_temperature = t.right_temperature;
   }
}

void unpackLocoCtrl(int can_id, uint8_t* buff, int16* left_vel, int16* right_vel){
    if (can_id == DAVID_LocoCtrl_FRAME_ID) {
        david_loco_ctrl_t t;
        david_loco_ctrl_unpack(&t, buff, 8);
        *left_vel = t.left_vel;
        *right_vel = t.right_vel;
   }
}

void unpackExcavCtrl(int can_id, uint8_t* buff, int16* vel){
    if (can_id == DAVID_ExcavCtrl_FRAME_ID) {
        david_excav_ctrl_t t;
        david_excav_ctrl_unpack(&t, buff, 8);
        *vel = t.vel;
   }
}

void unpackStepperCtrl(int can_id, uint8_t* buff, bool* home, int16* set_point){
    if (can_id == DAVID_StepperCtrl_FRAME_ID) {
        david_stepper_ctrl_t t;
        david_stepper_ctrl_unpack(&t, buff, 8);
        *home = t.home;
        *set_point = t.set_point;
   }
}

void unpackStepperTelem(int can_id, uint8_t* buff, bool* at_min_stop, bool* at_max_stop, int16* left_position, int16* right_position){
    if (can_id == DAVID_StepperTelem_FRAME_ID) {
        david_stepper_telem_t t;
        david_stepper_telem_unpack(&t, buff, 8);
        *at_min_stop = t.at_min_stop;
        *at_max_stop = t.at_max_stop;
        *left_position = t.left_position;
        *right_position = t.right_position;
   }
}

