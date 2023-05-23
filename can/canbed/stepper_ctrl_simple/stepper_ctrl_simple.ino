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
#define MAX_LIMIT A1

#define MAX_STEPS 370000

struct StepperController {
    InPin min_limit;
    InPin max_limit;

    Stepper right;
    Stepper left;
    StepperController(int pulse_l, int dir_l, int pulse_r, int dir_r, int min, int max) : left(pulse_l, dir_l), right(pulse_r, dir_r), min_limit(min), max_limit(max) {
    }

    const int steps_per_loop = 500;
    const int step_delay_micros = 5; // TODO(lcf): what is the lowest we can make this?
    void loop() {
        int steps = steps_per_loop;
        bool at_min = false;
    
        for (int steps = 0; steps < steps_per_loop; steps++) {
            at_min = min_limit.read();

            if (left.dir == Stepper::RETRACT && at_min) {
                left.count = 0;
            } else if (left.dir == Stepper::EXTEND && left.count >= MAX_STEPS) {
                left.count = MAX_STEPS;
            } else {
                left.doStep();
            }

            if (right.dir == Stepper::RETRACT && at_min) {
                right.count = 0;
            } else if (right.dir == Stepper::EXTEND && right.count >= MAX_STEPS){
                right.count = MAX_STEPS;
            } else {
                right.doStep();
            }
        }
    }

    void pack_telemetry(unsigned char buf[8]) {
        david_stepper_telem_t data = {0};
        data.at_min_stop = min_limit.read();
        data.left_position = david_stepper_telem_left_position_encode((double)left.count);
        data.right_position = david_stepper_telem_right_position_encode((double)right.count);
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

    StepperController control(RPUL, RDIR, LPUL, LDIR, MIN_LIMIT, MAX_LIMIT);

    bool eStopped = false;
    for (int count = 0; ; count++) {
        CANPacket packet = can_read_nonblocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) { eStopped = frame.stop; }
        }

        if (eStopped) {
            continue;
        }

        switch (packet.id) {
            FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
                control.left.setDirection(frame.left);
                control.right.setDirection(frame.right);
            }
        }
        
        control.loop();
    }
}
