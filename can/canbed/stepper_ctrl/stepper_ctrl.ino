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
        const int pulse_sequence[] = {HIGH, HIGH, LOW, LOW};
        const int dir_sequence[] = {LOW, HIGH, HIGH, LOW};
        pulse.write(pulse_sequence[step % 4]);
        direction.write(dir_sequence[step % 4]);
    }        
};

struct StepperController {
    Stepper right;
    Stepper left;
    InPin home_limit;
    InPin extent_limit;
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
    left(pulse_l, dir_l), right(pulse_r, dir_r), home_limit(home), extent_limit(forward) {
        state = HOME;
        step = 100000;
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
    const int read_limit_frequency = 5;
    void loop() {
        int ticks = ticks_per_loop;
        bool at_home = false;
        bool at_extent = false;
        while (ticks-- > 0) {
            if (ticks % read_limit_frequency == 0) {
                at_home = home_limit.read_threshold();
                at_extent = extent_limit.read_threshold();
            }

            switch (state) {
            case HOME: {
                dir = BACKWARD;
                if (at_home) {
                    /* step = point = 0; */
                    /* state = MOVE; */
                } else {
                    doStep(step--);
                }
            } break;
            case MOVE: {
                dir = sign((int)point - (int)step);
                if (dir == BACKWARD && at_home) {
                    point = step; dir = STOP;
                }
                if (dir == FORWARD && at_extent) {
                    point = step; dir = STOP;
                }
                doStep(step += dir);            
            } break;
            }
        } // end while(ticks)
        
        // TODO telemetry
    }
};

void setup() {
    #define RPUL 10
    #define RDIR 6
    #define LPUL 4
    #define LDIR 11
    #define HOME_LIMIT A0
    #define EXTENT_LIMIT A1
    StepperController control(RPUL, RDIR, LPUL, LDIR, HOME_LIMIT, EXTENT_LIMIT);

    MCP_CAN can = setup_can();

    bool eStopped = false;
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

        /* int ticks = control.ticks_per_loop; */
        /* while (ticks-- > 0) { */
            /* control.doStep(control.step++); */
        /* } */
        control.loop();
    }
}
