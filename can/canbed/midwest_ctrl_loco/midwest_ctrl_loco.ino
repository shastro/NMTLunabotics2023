#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "david.h"
#include "pins_arduino.h"
#include "arduino_lib.hpp"

void setup() {
    MCP_CAN can = setup_can();
    
    // TODO: Wire up inhbit pins and PWMs
    #define LINHIBIT 0
    #define LRELAY_CCW 4
    #define LRELAY_CW 5
    #define LRELAY_COMMON 6
    MidwestMotorController left(LINHIBIT, LRELAY_CCW, LRELAY_CW, LRELAY_COMMON);
    #define RINHIBIT 0
    #define RRELAY_CCW 10
    #define RRELAY_CW 9
    #define RRELAY_COMMON 11
    MidwestMotorController right(RINHIBIT, RRELAY_CCW, RRELAY_CW, RRELAY_COMMON);
    
    bool eStopped = false;
    for (;;) {
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
                left.setVel(0.0);
                right.setVel(0.0);
            }
        }

        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_LOCO_CTRL, david_loco_ctrl) {
                left.setVel(david_loco_ctrl_left_vel_decode(frame.left_vel));
                right.setVel(david_loco_ctrl_right_vel_decode(frame.right_vel));
            }
        }
    }
}
