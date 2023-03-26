// Locomotion and excavation controllers.

#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "david.h"

#define SPI_CS_PIN 17

// The dead-zone of velocity in meters per second; within this region
// the motors are turned off.
#define VEL_DEADZONE 0.01

// Digital output pins.
class DigOutPin {
    int num;

  public:
    DigOutPin(int num) : num(num) { pinMode(num, OUTPUT); }
    void write(bool value) { digitalWrite(num, value ? HIGH : LOW); }
};

// Motor controller object.
class MotorController {
    DigOutPin relay_left;
    DigOutPin relay_right;
    DigOutPin relay_common;

    void set_direction(bool clockwise) {
        // Create a 5-volt difference between relay_left and _right,
        // which will trigger the coil in the relay.
        relay_left.write(clockwise);
        relay_right.write(!clockwise);
    }

  public:
    MotorController(int relay_left, int relay_right, int relay_common)
        : relay_left(relay_left), relay_right(relay_right),
          relay_common(relay_common) {
        setVel(0);
    }

    void setVel(double vel) {
        // Currently we only support full forward, full backward, and
        // stopped velocity.
        if (abs(vel) < VEL_DEADZONE) {
            // XXX: this *sometimes* turns off the motor, and
            // sometimes doesn't.
            set_direction(false);
            relay_common.write(false);
        } else if (vel > 0) {
            set_direction(true);
            relay_common.write(true);
        } else {
            set_direction(false);
            relay_common.write(true);
        }
    }

    // TODO: implement telemetry.
};

// Set up a CAN connection.
MCP_CAN setup_can() {
    MCP_CAN can(SPI_CS_PIN);
    while (can.begin(CAN_500KBPS) != CAN_OK) {
        Serial.println("CAN bus fail!");
        delay(100);
    }
    Serial.println("CAN bus ok!");
    return can;
}

void setup() {
    Serial.begin(115200);

    MotorController left(8, 9, 10);
    // MotorController right(3, 4, 5);
    // MotorController excav(6, 7, 8);

    MCP_CAN can = setup_can();

    while (1) {
        // left.setVel(0);
        // delay(1000);

        Serial.println("Counter-clockwise");
        left.setVel(-10);
        delay(2000);

        Serial.println("Stop");
        left.setVel(0);
        delay(2000);

        Serial.println("Clockwise");
        left.setVel(10);
        delay(2000);

        Serial.println("Stop");
        left.setVel(0);
        delay(2000);
    }
}
