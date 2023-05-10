#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"
#include "pins_arduino.h"

void setup() {
    MCP_CAN can = setup_can();

    // PWM pins: 5, 6, 3, 9, 10, 11
    MidwestMotorController left(11, Relay(A0, A1, A2));
    MidwestMotorController right(D8, Relay(D6, A3, 12));

    bool eStopped = false;
    while (1) {
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
