#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"
#include "pins_arduino.h"

void setup() {
    MCP_CAN can = setup_can();

    // PWM pins for leonardo
    // 3, 5, 6, 9, 10, 11, 13
    MidwestMotorController left(12, 4, 5, 10);
    MidwestMotorController right(A2, A1, 11, 6);

    bool eStopped = false;
    for(;;) {
        CANPacket packet = can_read_blocking(can);
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
                left.setVel(-david_loco_ctrl_left_vel_decode(frame.left_vel));
                right.setVel(david_loco_ctrl_right_vel_decode(frame.right_vel));
                Serial.print(david_loco_ctrl_left_vel_decode(frame.left_vel));
                Serial.print(", ");
                Serial.println(david_loco_ctrl_right_vel_decode(frame.right_vel));
               // left.setVel(10.0);
                //right.setVel(10.0);
            }
        }
    }
}
