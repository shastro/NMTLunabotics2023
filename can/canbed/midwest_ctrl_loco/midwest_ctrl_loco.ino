#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "david.h"
#include "pins_arduino.h"
#include "arduino_lib.hpp"

void setup() {
    MCP_CAN can = setup_can();

    // PWM pins: 5, 6, 3, 9, 10, 11
    // TODO: Needs correct pins and stuff
    // TODO: Wire up inhbit pins and PWMs
    #define LINHIBIT 6
    #define LRELAY_CCW 12
    #define LRELAY_CW A3
    #define LRELAY_PWM 9
    MidwestMotorController left(LINHIBIT, LRELAY_CCW, LRELAY_CW, LRELAY_PWM);
    #define RINHIBIT 6
    #define RRELAY_CCW A0
    #define RRELAY_CW 9
    #define RRELAY_PWM A2
    MidwestMotorController right(RINHIBIT, RRELAY_CCW, RRELAY_CW, RRELAY_PWM);
    
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

        // TODO(lcf): is this needed?
        delayMicroseconds(1000);
    }
}
