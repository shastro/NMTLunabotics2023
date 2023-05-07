// Locomotion and excavation controllers.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

// The dead-zone of velocity in meters per second; within this region
// the motors are turned off.
#define VEL_DEADZONE 0.01

// Motor controller object.
class MotorController {
    OutPin enable;
    Relay relay;

  public:
    MotorController(int inhibit, int relay_left, int relay_right,
                    int relay_common)
        : enable(inhibit), relay(relay_left, relay_right, relay_common) {
        setVel(0);
    }

    void setVel(double vel) {
        // Experimentally determined that the motors' maximum velocity
        // is somewhere around 0.45-0.5 duty cycle. It's unclear why,
        // but as a result for better control we can just multiply by
        // 0.5;
        vel *= 0.5;

        // The motors are slightly more responsive when going forwards
        // than when going backwards.
        if (vel < 0)
            vel *= 1.5;

        enable.write(abs(vel) > VEL_DEADZONE);
        if (vel > 0)
            relay.output_right().write_pwm(abs(vel));
        else
            relay.output_left().write_pwm(abs(vel));
    }

    // TODO: implement telemetry.
};

void setup() {
    Serial.begin(115200);

    MotorController excav(11, 4, 5, 6);
    MCP_CAN can = setup_can();

    bool eStopped = true;
    while (true) {
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
                excav.setVel(0);
            }
        }

        // Avoid doing anything else if we're e-stopped.
        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_EXCAV_CTRL, david_excav_ctrl) {
                excav.setVel(david_excav_ctrl_vel_decode(frame.vel) / 100);
            }
        }
    }
}

// no loop()

