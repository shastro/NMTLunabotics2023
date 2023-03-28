// Locomotion and excavation controllers.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "david.h"

#define SPI_CS_PIN 17

// The dead-zone of velocity in meters per second; within this region
// the motors are turned off.
#define VEL_DEADZONE 0.01

struct CANPacket {
    uint32_t len;
    uint32_t id;
    unsigned char buf[8];
};

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
            // relay_common is also tied to MMP's pin 9
            // "INHIBIT/ENABLE", so writing 0 to this also turns off
            // the motor.

            // Do you have any idea how fucking long it took to figure
            // out how to turn off this motor? We tried turning off
            // both inputs to the driver. We tried turning on both
            // inputs to the driver. For hours we had this thing
            // turning one direction or the other, but able to stop
            // only about 75% of the time. The relevant data sheet
            // section says "TTL level (+5V) inhibit/enable input.
            // Pull to ground to inhibit drive (SW1-5 ON). Pull to
            // ground to enable drive (SW1-5 OFF)." Ah, so it's not
            // "stop," it's not "turn the fucking motor off," it's not
            // "disable," it's fucking "INHIBIT."
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

// Read a CAN packet. Blocks until one is available.
CANPacket can_read(MCP_CAN &can) {
    while (can.checkReceive() != CAN_MSGAVAIL)
        ;
    CANPacket packet;
    can.readMsgBuf((unsigned char *)&packet.len, packet.buf);
    packet.id = can.getCanId();

    return packet;
}

void setup() {
    Serial.begin(115200);

    // MotorController left(9, 10, 11);
    // MotorController right(A0, A1, A2);
    MotorController excav(9, 10, 11);

    MCP_CAN can = setup_can();

    bool eStopped = false;
    while (true) {
        // excav.setVel(1);
        // delay(1000);
        // excav.setVel(0);
        // delay(1000);
        // excav.setVel(-1);
        // delay(1000);
        // excav.setVel(0);
        // delay(1000);

        CANPacket packet = can_read(can);
        switch (packet.id) {
        case DAVID_E_STOP_FRAME_ID: {
            eStopped = true;
            excav.setVel(0);
            break;
        }

        case DAVID_E_START_FRAME_ID: {
            eStopped = false;
            break;
        }
        }

        // Avoid doing anything else if we're e-stopped.
        if (eStopped)
            continue;

        switch (packet.id) {
        case DAVID_EXCAV_CTRL_FRAME_ID: {
            david_excav_ctrl_t ep;
            david_excav_ctrl_unpack(&ep, packet.buf, packet.len);
            double rpm = (int32_t)ep.rpm * 0.001;
            excav.setVel(rpm);
        }
        }
    }
}

// no loop()
