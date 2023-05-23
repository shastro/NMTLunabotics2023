#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

void setup() {
    MCP_CAN can = setup_can();

#define INHIBIT A3
#define RELAY_CCW D5
#define RELAY_CW D4
#define RELAY_PWM D6
    MidwestMotorController excav(INHIBIT, Relay(RELAY_CCW, RELAY_CW, RELAY_PWM),
                                 1);

    bool eStopped = false;
    for (;;) {
        CANPacket packet = can_read_blocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
                excav.setVel(0.0);
            }
        }

        // Avoid doing anything else if we're e-stopped.
        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_EXCAV_CTRL, david_excav_ctrl) {
                double vel = david_excav_ctrl_vel_decode(frame.vel);
                excav.setVel(vel);
                Serial.println(vel);
            }
        }
    }
}
