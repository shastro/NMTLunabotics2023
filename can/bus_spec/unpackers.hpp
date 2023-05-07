#include "david.h"


/*
 * unpackers.hpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

// struct CANPacket {
//     uint32_t len;
//     uint32_t id;
//     unsigned char buf[8];
// };

// template <typename T>
// void david_excav_ctrl_run(const CANPacket &packet, T function) {
//     if (packet.id == DAVID_EXCAV_CTRL_FRAME_ID) {
//         struct david_excav_ctrl_t s;
//         david_excav_ctrl_unpack(&s, packet.buf, packet.len);
//         function(s);
//     }
// }


void unpackEStop(int can_id, uint8_t* buff, int8* stop);
void unpackPitchCtrl(int can_id, uint8_t* buff, int16* set_point, int16* left_offset, int16* right_offset);
void unpackPitchPositionTelem(int can_id, uint8_t* buff, int16* left_position, int16* right_position);
void unpackPitchDriverTelem(int can_id, uint8_t* buff, int8* left_current, int8* right_current, int8* left_temperature, int8* right_temperature);
void unpackLocoCtrl(int can_id, uint8_t* buff, int16* left_vel, int16* right_vel);
void unpackExcavCtrl(int can_id, uint8_t* buff, int16* vel);
void unpackStepperCtrl(int can_id, uint8_t* buff, bool* home, int16* set_point);
void unpackStepperTelem(int can_id, uint8_t* buff, bool* at_min_stop, bool* at_max_stop, int16* left_position, int16* right_position);
