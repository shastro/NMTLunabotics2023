#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

struct StepperController {
    Stepper right;
    Stepper left;
    InPin min_limit;
    InPin max_limit;
    enum States {
        MOVE = 0, // Move to point
        HOME = 1,
    };
    enum States state;
    enum Dirs { BACKWARD = -1, STOP = 0, FORWARD = 1 };
    unsigned long int pos;
    unsigned long int point;
    enum Dirs dir;
    unsigned int homing_timer;

    StepperController(int pulse_l, int dir_l, int pulse_r, int dir_r, int min,
                      int max)
        : left(pulse_l, dir_l), right(pulse_r, dir_r), min_limit(min),
          max_limit(max) {
        state = MOVE;
        pos = 0;
        point = 0;
        dir = STOP;
    }

#define STEP_TO_MM (10000)
    void setPoint(double p) {
        // TODO(lcf): double check that this isnt fucky
        // TODO(lcf): THIS IS FUCKY, it moves soo little....
        point = (unsigned long int)(p * (STEP_TO_MM));
    }

    void doStep(enum Dirs dir) {
        if (dir != STOP) {
            pos += dir;
            right.doStep(pos);
            left.doStep(pos);
        }
    }

    const int ticks_per_loop = 2000;
    const int read_limit_frequency = 200;
    const int step_delay_micros = 5;
    void loop() {
        int ticks = ticks_per_loop;
        bool at_min = false;
        bool at_max = false;
        while (ticks > 0) {
            if (ticks-- % read_limit_frequency == 0) {
                at_min = min_limit.read();
                at_max = max_limit.read();
            }

            switch (state) {
            case HOME: {
                if (at_min) {
                    pos = 0;
                    state = MOVE;
                    dir = STOP;
                } else {
                    dir = BACKWARD;
                }
            } break;
            case MOVE: {
                dir = (Dirs)sign((int)point - (int)pos);
                if (dir == BACKWARD && at_min) {
                    point = pos;
                    dir = STOP;
                }
                if (dir == FORWARD && at_max) {
                    point = pos;
                    dir = STOP;
                }
            } break;
            }

            doStep(dir);
            delayMicroseconds(step_delay_micros);
        } // end while(ticks)

        delay(50);
    }

    void pack_telemetry(unsigned char buf[8]) {
        david_stepper_telem_t data = {0};
        data.at_min_stop = min_limit.read();
        data.at_max_stop = max_limit.read();
        // TODO(lcf): steppers should always be at same pos, so we can get rid
        // of left/right position and have position and point in kcd to verify
        // commands are updating point correctly
        data.position =
            david_stepper_telem_position_encode(((double)pos) / STEP_TO_MM);
        data.set_point =
            david_stepper_telem_position_encode(((double)point) / STEP_TO_MM);
        david_stepper_telem_pack(buf, &data, 8);
    }
};

void setup() {
    MCP_CAN can = setup_can();

#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0
#define MAX_LIMIT A1

    int count = 0;

    StepperController control(RPUL, RDIR, LPUL, LDIR, MIN_LIMIT, MAX_LIMIT);

    bool eStopped = false;
    for (;;) {

        count++;

        CANPacket packet = can_read_nonblocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) { eStopped = frame.stop; }
        }

        // Send Telemetry
        if (count % 200 == 0) {
            CANPacket telemetry(DAVID_STEPPER_TELEM_FRAME_ID);
            control.pack_telemetry(telemetry.buf);
            can_send(can, telemetry);
        }

        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
                if (frame.home) {
                    control.state = control.HOME;
                } else {
                    control.setPoint(
                        david_stepper_telem_set_point_decode(frame.set_point));
                }
            }
        }
        control.loop();
    }
}
