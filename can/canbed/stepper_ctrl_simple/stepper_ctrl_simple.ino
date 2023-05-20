#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

enum Dirs { BACKWARD = 0, STOP = 1, FORWARD = 2};

#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0
#define MAX_LIMIT A1

const int direction_voltages = {HIGH, LOW, LOW};
struct StepperController {
    InPin min_limit;
    InPin max_limit;

    Stepper right;
    Stepper left;
    enum Dirs right_dir;
    enum Dirs left_dir;

    StepperController(int pulse_l, int dir_l, int pulse_r, int dir_r, int min, int max) : left(pulse_l, dir_l), right(pulse_r, dir_r), min_limit(min), max_limit(max) {
            left_dir = STOP;
            left.direction.write(direction_voltages[left_dir]);
            right_dir = STOP;
            right.direction.write(direcetion_voltages[right_dir]);
    }

    const int steps_per_loop = 500;
    const int read_limit_frequency = 10;
    const int step_delay_micros = 50;
    void loop() {
        int steps = steps_per_loop;
        bool at_min = false;
        int dir_table[] = {-1, 0, 1};

        for (int steps = 0; steps < steps_per_loop; steps++) {
            if ((steps % read_limit_frequency) == 0) {
                at_min = min_limit.read();
            }

            if (left_dir == BACKWARD && at_min) {
                left.count = 0;
            } else {
                left.doStep(dir_table[left_dir]); // TODO(lcf) why no increment
            }

            if (right_dir == BACKWARD && at_min) {
                right.count = 0;
            } else {
                right.doStep(dir_table[right_dir]);
            }
            
            delayMicroseconds(step_delay_micros);
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

        // Send Telemetry
        #define TELEMETRY_FREQUENCY 2
        if (count % TELEMETRY_FREQUENCY == 0) {
            CANPacket telemetry(DAVID_STEPPER_TELEM_FRAME_ID);
            control.pack_telemetry(telemetry.buf);
            can_send(can, telemetry);
        }

        if (eStopped) {
            continue;
        }

        switch (packet.id) {
            FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
                control.left_dir = frame.left;
                control.left.direction.write(direction_voltages(frame.left));
                control.right_dir = frame.right;
                control.right.direction.write(direction_voltages(frame.right));
            }
        }
        
        control.loop();
    }
}
