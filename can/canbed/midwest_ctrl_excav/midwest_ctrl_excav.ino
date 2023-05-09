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

        if (abs(vel) < VEL_DEADZONE) {
            enable.write(false);
            /* relay.common.analogWrite(0.0f); */
            relay.common.write_pwm(0.0f);
        } else {
            enable.write(true);
            ((vel > 0)? relay.output_right() : relay.output_left()).write_pwm(abs(vel));
        }
        
    }

    // TODO: implement telemetry.
};

void setup() {
    MCP_CAN can = setup_can();

    #define INHIBIT 11
    #define RELAY_LEFT 4
    #define RELAY_RIGHT 5
    #define RELAY_COMMON 6
    MotorController excav(INHIBIT, RELAY_LEFT, RELAY_RIGHT, RELAY_COMMON);

    bool eStopped = false;
    for (;;) {
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
                excav.setVel(david_excav_ctrl_vel_decode(frame.vel) / 1000.0f);
            }
        }
    }
}

// no loop()

