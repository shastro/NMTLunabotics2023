#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

struct StepperController {
    enum Dirs {
        CCW = -1,
        STOP = 0,
        CW = 1
    };
    enum Dirs dir;

    Stepper cam;

    unsigned int step;

StepperController(int pulse_c, int dir_c) :
    cam(pulse_c, dir_c) {
        dir = STOP;
        step = 0;
    }
    void doStep(enum Dirs dir) {
        if (dir != STOP) {
            step += dir;
            cam.doStep(step);
        }
    }

    void loop() {
        doStep(dir);
        delayMicroseconds(50);
    }
};

void setup() {
    MCP_CAN can = setup_can();
    
#define PUL 5 // For camera mast
#define DIR 4
    StepperController control(PUL, DIR);

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
            FRAME_CASE(DAVID_MAST_CTRL, david_mast_ctrl) {
                control.dir = frame.direction;
            }
        }
        control.loop();
    }
}


