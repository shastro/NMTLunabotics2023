#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>
#include <limits.h>

#include "arduino_lib.hpp"
#include "david.h"

#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0

OutPin rpul(RPUL);
OutPin lpul(LPUL);
OutPin rdir(RDIR);
OutPin ldir(LDIR);
InPin min_limit(MIN_LIMIT);

bool eStopped = false;
bool home = false;
bool at_min = false;

unsigned long pos = 0;
unsigned long point = 0;
unsigned long clock = 0;

CANPacket packet;

void doStep();
void pack_telemetry(unsigned char buf[8]);
MCP_CAN can = setup_can();

void setup() {
}

void loop() {
    clock++;
        
    packet = can_read_nonblocking(can);
    switch (packet.id) {
        FRAME_CASE(DAVID_E_STOP, david_e_stop) { eStopped = frame.stop; }
    }
    
    if (eStopped)
        return;

    switch (packet.id) {
        FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
            if (frame.home) {
                pos = INT_MAX;
                point = 0;
            } else {
                point = frame.set_point;
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
    //if (clock % 5 == 0) {
        at_min = min_limit.read();
    //}

    if (at_min) {
        pos = 0;
    }

    // Handle point
    if (pos < point) {
        doStep(false);
    } else if (pos > point) {
        doStep(true);
    }
    
}

void doStep(bool dir) {
    if (dir) {
        pos--;
    } else {
        pos++;
    }
    ldir.write(dir);
    rdir.write(dir);
    lpul.write(HIGH);
    lpul.write(HIGH);
    delayMicroseconds(100);    
    lpul.write(LOW);
    rpul.write(LOW);
    delayMicroseconds(100);    
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
