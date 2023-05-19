#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0

Stepper right;
Stepper left;
InPin min_limit;

bool eStopped = false;
bool home = false;
bool at_min = false;
bool at_max = false;

unsigned long pos = 0;
unsigned long point = 0;
unsigned long clock = 0;

CANPacket packet;

void doStep(enum Dirs dir) {
void pack_telemetry(unsigned char buf[8]) {

void setup() {
    MCP_CAN can = setup_can();

    left = Stepper(LPUL, LDIR);
    right = Stepper(RPUL, RDIR);
    min_limit = InPin(MIN_LIMIT);
}

void loop() {
    clock = millis();
    
    packet = can_read_nonblocking(can);
    switch (packet.id) {
        FRAME_CASE(DAVID_E_STOP, david_e_stop) { eStopped = frame.stop; }
    }
    
    if (eStopped)
        return;

    switch (packet.id) {
        FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
            if (frame.home) {
                point = 0;
            } else {
                point = frame.point;
            }
        }
    }
    
    // Send Telemetry
    if (clock % 10 == 0) {
        CANPacket telemetry(DAVID_STEPPER_TELEM_FRAME_ID);
        pack_telemetry(telemetry.buf);
        can_send(can, telemetry);
    }

    // Get min swich
    if (clock % 5 == 0) {
        at_min = min_limit.read();
    }

    if (at_min) {
        pos = 0;
    }

    // Handle point
    if (clock % 50 == 0) {
        if (pos < point) {
            doStep(1);
        } else if (pos > point) {
            doStep(-1);
        }
    }
    
}

void doStep(enum Dirs dir) {
    pos += (int) dir;
    right.doStep(dir);
    left.doStep(dir);
}

void pack_telemetry(unsigned char buf[8]) {
    david_stepper_telem_t data = {0};
    data.at_min_stop = at_min;
    data.at_max_stop = false;
    // TODO(lcf): steppers should always be at same pos, so we can get rid
    // of left/right position and have position and point in kcd to verify
    // commands are updating point correctly
    data.position =
        david_stepper_telem_position_encode(pos);
    data.set_point =
        david_stepper_telem_position_encode(pos);
    david_stepper_telem_pack(buf, &data, 8);
}
