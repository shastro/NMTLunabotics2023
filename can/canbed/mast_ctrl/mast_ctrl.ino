#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

int spin_voltages[3] = {0, 0, 1};
struct StepperController {
    enum Dirs {
        CCW = -1,
        STOP = 0,
        CW = 1
    };
    enum Dirs dir;

    OutPin pulse;
    OutPin dirpin;
    long count;

    #define STEP_TO_DEGREE (0.0533333333333)

    StepperController(int pulse_c, int dir_c) :
    pulse(pulse_c), dirpin(dir_c) {
        dir = STOP;
        dirpin.write(spin_voltages[dir]);
        count = 6750;
    }
    
    void doStep() {
        if ((count >= 13500 && dir < 0) || (count <= 0 && dir > 0) || (count < 13500 && count > 0)) {
            if (dir != STOP) {
                count += dir;
                pulse.write(0);
                pulse.write(1);
                delayMicroseconds(500);
                //Serial.println(count);
            }
        }   
    }

    const int ticks_per_loop = 50;
    const int step_delay_micros = 200;
    void loop() {
        for (int i = 0; i < ticks_per_loop; i++) {
            doStep();
            delayMicroseconds(step_delay_micros);
        }
    }

    void pack_telemetry(unsigned char buf[8]) {
        double stepDegree = count*STEP_TO_DEGREE-360;
        constrain(stepDegree, -360, 359);
        david_mast_telem_t data = {0};
        data.angle = david_mast_telem_angle_encode(((double)stepDegree));
        david_mast_telem_pack(buf, &data, 8);
        //Serial.println(stepDegree);
    }
};

void setup() {
    MCP_CAN can = setup_can();
    #define PUL 5 // For camera mast
    #define DIR 4
    StepperController control(PUL, DIR);

    bool eStopped = false;
    for (int countup = 0; ; countup++) {
        CANPacket packet = can_read_nonblocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
            }
        }

        // Send Telemetry
        if (countup % 30 == 0) {
            CANPacket telemetry = CANPacket(DAVID_MAST_TELEM_FRAME_ID);
            control.pack_telemetry(telemetry.buf);
            can_send(can, telemetry);
            //Serial.print(countup);
        }
        //Serial.println(countup);
        if (eStopped)
            continue;
        
        switch (packet.id) {
            FRAME_CASE(DAVID_MAST_CTRL, david_mast_ctrl) {
                control.dir = frame.direction-1;
                control.dirpin.write(spin_voltages[frame.direction]);
            }
        }
        control.loop();
        delayMicroseconds(500);
    }
}


