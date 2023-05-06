// Locomotion and excavation controllers.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "david.h"
#include "pins_arduino.h"

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
    DigOutPin dir_clockwise;
    DigOutPin dir_counter;
    DigOutPin inhibit;

    void set_direction(bool clockwise) {
        dir_clockwise.write(clockwise);
        dir_counter.write(!clockwise);
    }

  public:
    // Build a motor controller with the given pins.
    MotorController(int dir_clockwise, int dir_counter, int inhibit)
        : dir_clockwise(dir_clockwise), dir_counter(dir_counter),
          inhibit(inhibit) {
        setVel(0);
    }

    void setVel(double vel) {
        // Currently we only support full forward, full backward, and
        // stopped velocity.
        if (abs(vel) < VEL_DEADZONE) {
            inhibit.write(false);
        } else if (vel > 0) {
            set_direction(true);
            inhibit.write(true);
        } else {
            set_direction(false);
            inhibit.write(true);
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

    MotorController left(A0, A1, A2);
    MotorController right(10, 9, 11);

    MCP_CAN can = setup_can();

    bool eStopped = false;
    while (true) {
        CANPacket packet = can_read(can);
        switch (packet.id) {
        case DAVID_E_STOP_FRAME_ID: {
            eStopped = true;
            left.setVel(0);
            right.setVel(0);
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
        case DAVID_LOCO_CTRL_LEFT_FRAME_ID: {
            david_loco_ctrl_left_t lp;
            david_loco_ctrl_left_unpack(&lp, packet.buf, packet.len);
            double vel = (int32_t)lp.velocity * 0.001;
            left.setVel(vel);
            break;
        }

        case DAVID_LOCO_CTRL_RIGHT_FRAME_ID: {
            david_loco_ctrl_right_t rp;
            david_loco_ctrl_right_unpack(&rp, packet.buf, packet.len);
            double vel = (int32_t)rp.velocity * 0.001;
            right.setVel(vel);
            break;
        }

        case DAVID_LOCO_CTRL_BOTH_FRAME_ID: {
            david_loco_ctrl_both_t bp;
            david_loco_ctrl_both_unpack(&bp, packet.buf, packet.len);
            double vel = (int32_t)bp.velocity * 0.001;
            left.setVel(vel);
            right.setVel(vel);
            break;
        }
        }
    }
}

// no loop()
