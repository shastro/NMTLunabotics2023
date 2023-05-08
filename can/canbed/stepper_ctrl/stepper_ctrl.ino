#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

struct Stepper {
    OutPin pulse;
    OutPin direction;

    Stepper(int pulse_pin, int dir_pin) :
    pulse(pulse_pin), direction(dir_pin) {}

    void doStep(unsigned int step) {
        const bool pulse_sequence[] = {1, 1, 0, 0};
        const bool dir_sequence[] = {0, 1, 1, 0};
        pulse.write(pulse_sequence[step % 4]);
        direction.write(dir_sequence[step % 4]);
    }        
};

struct StepperController {
    Stepper right;
    Stepper left;
    InPin home_int;
    InPin forward_int;
    enum States {
        MOVE = 0, // Move to point
        HOME = 1,
    };
    enum States state;
    enum Dirs {
        BACKWARD = -1,
        STOP = 0,
        FORWARD = 1
    };
    unsigned int step;
    unsigned int point;
    int dir;

    StepperController(int pulse_l, int dir_l, int pulse_r, int dir_r, int home, int forward) :
    left(pulse_l, dir_l), right(pulse_r, dir_r), home_int(home), forward_int(forward) {
        state = HOME;
        step = 0;
        point = 0;
        dir = STOP;
    }

    void setPoint(double p) {
        // TODO: double check that this isnt fucky
        double mm_to_steps = 0.296875;
        point = (unsigned int) (p * mm_to_steps);
    }
    
    void doStep(unsigned int s) {
        right.doStep(step);
        left.doStep(step);
    }

    const int ticks_per_loop = 100;
    const int step_delay_micros = 100;
    const int limit_volt_threshold = 1000;
    void loop() {
        int ticks = ticks_per_loop;
        while (ticks-- > 0) {
            switch (state) {
            case HOME: {
                // TODO(lcf): poll limit-switches
                // or alternatively setup interrupts.
                if (!home_int.read()) {
                    doStep(step--);
                } else {
                    step = point = 0;
                    state = MOVE;
                }
            } break;
            case MOVE: {
                dir = sign((int)point - (int)step);
                if (dir == BACKWARD && home_int.read()) {
                    // TODO: convey that we are at limits in telemetry
                } else if (dir == FORWARD && forward_int.read()) {
                
                } else {
                    doStep(step += dir);            
                }
            } break;
            }
            delayMicroseconds(step_delay_micros);
        } // end while(ticks)
        
        // TODO telemetry
    }
};

const int stepsPerTick = 600;  // change this to fit the number of steps per revolution
const int stepDelayMicros = 10;

void setup() {
    enum {
        RPUL = 10,
        RDIR = 6, 
        LPUL = 4,
        LDIR = 11,
        HOME_INT = 2,
        FORWARD_INT = 3,
    };
    StepperController control(RPUL, RDIR, LPUL, LDIR, HOME_INT, FORWARD_INT);

    MCP_CAN can = setup_can();

    bool eStopped = true;
    for (;;) {
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
            }
        }

        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
                control.state = (frame.home)? control.HOME : control.MOVE;
                control.setPoint(david_stepper_ctrl_set_point_decode(frame.set_point));
            }
        }

        control.loop();
    }
}
