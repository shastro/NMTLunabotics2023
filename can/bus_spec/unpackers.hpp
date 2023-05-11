
/*
 * unpackers.hpp
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include "david.h"

struct CANPacket {
    uint32_t len;
    uint32_t id;
    unsigned char buff[8];
};

template <typename T>
void e_stop_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_EStop_FRAME_ID) {
        struct david_e_stop_t t;
        david_e_stop_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void pitch_ctrl_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_PitchCtrl_FRAME_ID) {
        struct david_pitch_ctrl_t t;
        david_pitch_ctrl_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void pitch_position_telem_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_PitchPositionTelem_FRAME_ID) {
        struct david_pitch_position_telem_t t;
        david_pitch_position_telem_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void pitch_driver_telem_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_PitchDriverTelem_FRAME_ID) {
        struct david_pitch_driver_telem_t t;
        david_pitch_driver_telem_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void loco_ctrl_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_LocoCtrl_FRAME_ID) {
        struct david_loco_ctrl_t t;
        david_loco_ctrl_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void excav_ctrl_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_ExcavCtrl_FRAME_ID) {
        struct david_excav_ctrl_t t;
        david_excav_ctrl_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void stepper_ctrl_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_StepperCtrl_FRAME_ID) {
        struct david_stepper_ctrl_t t;
        david_stepper_ctrl_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void stepper_telem_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_StepperTelem_FRAME_ID) {
        struct david_stepper_telem_t t;
        david_stepper_telem_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void mast_ctrl_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_MastCtrl_FRAME_ID) {
        struct david_mast_ctrl_t t;
        david_mast_ctrl_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

template <typename T>
void mast_telem_dispatch(const CANPacket &packet, T function) {
    if (packet.id == DAVID_MastTelem_FRAME_ID) {
        struct david_mast_telem_t t;
        david_mast_telem_unpack(&t, packet.buff, packet.len);
        function(t);
    }
}

