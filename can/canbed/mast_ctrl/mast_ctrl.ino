#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

// TODO(cad): measure steps to rotation, so that
// TODO(cad): enforce a limit of rotations so that the cable does not get wound around
// to much.

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
        step = 26000;
    }
    void doStep(enum Dirs dir) {
        if (dir != STOP) {
          if ((step >= 52000 && dir < 0) || (step <= 0 && dir > 0) || (step < 52000 && step > 0)) {
            step += dir;
            cam.doStep(step);
            Serial.println(step);
          }   
        }
      }

    const int ticks_per_loop = 50;
    const int step_delay_micros = 200;
    void loop() {
        for (int i = 0; i < ticks_per_loop; i++) {
            doStep(dir);
            delayMicroseconds(step_delay_micros);
        }
    }

        void pack_telemetry(unsigned char buf[8]) {
        david_mast_telem_t data = {0};
        data.angle = david_mast_telem_angle_encode(((double)step));
        david_mast_telem_pack(buf, &data, 8);
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

        // Send Telemetry
        CANPacket telemetry = {DAVID_MAST_TELEM_FRAME_ID, 0};
        control.pack_telemetry(telemetry.buf);
        can_send(can, telemetry);

        if (eStopped)
            continue;
        
        switch (packet.id) {
            FRAME_CASE(DAVID_MAST_CTRL, david_mast_ctrl) {
                control.dir = frame.direction-1;
            }
        }
        control.loop();
    }
}


