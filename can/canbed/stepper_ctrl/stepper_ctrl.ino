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
    InPin min_limit;
    InPin max_limit;
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
    unsigned int pos;
    unsigned int point;
    enum Dirs dir;
    unsigned int homing_timer;

    StepperController(int pulse_l, int dir_l, int pulse_r, int dir_r, int min, int max) :
    left(pulse_l, dir_l), right(pulse_r, dir_r), min_limit(min), max_limit(max) {
        state = HOME;
        pos = 1000;
        point = 0;
        dir = STOP;
    }

    #define STEP_TO_MM 0.296875
    void setPoint(unsigned int p) {
        // TODO: double check that this isnt fucky
        point = p;
    }
    
    void doStep(enum Dirs dir) {
        if (dir != STOP) {
            pos += dir;
            right.doStep(pos);
            left.doStep(pos);
        }
    }

    const int ticks_per_loop = 100; 
    const int read_limit_frequency = 5;
    const int step_delay_micros = 10;
    const int homing_time_ticks = 10;
    void loop() {
        int ticks = ticks_per_loop;
        bool at_min = false;
        bool at_max = false;
        while (ticks-- > 0) {
            if (ticks % read_limit_frequency == 0) {
                at_min = min_limit.read_threshold();
                at_max = max_limit.read_threshold();
            }

            switch (state) {
            case HOME: {
                if (at_min) {
                    homing_timer++;
                    if (homing_timer > homing_time_ticks) {
                        pos = point = 0;
                        state = MOVE;
                        dir = STOP;
                        homing_timer = 0;
                    }
                } else {
                    dir = BACKWARD;
                    homing_timer = 0;
                }
            } break;
            case MOVE: {
                dir = sign((int)point - (int)pos);
                if (dir == BACKWARD && at_min) {
                    point = pos; dir = STOP;
                }
                if (dir == FORWARD && at_max) {
                    point = pos; dir = STOP;
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
        data.at_min_stop = min_limit.read_threshold();
        /* data.at_max_stop = max_limit.read_threshold(); */
        data.at_max_stop = state;
        // TODO(lcf): steppers should always be at same pos, so we can get rid of
        // left/right position and have position and point in kcd to verify commands
        // are updating point correctly
        data.left_position = david_stepper_telem_left_position_encode(((double)pos)*STEP_TO_MM);
        data.right_position = david_stepper_telem_left_position_encode(((double)point)*STEP_TO_MM);
        david_stepper_telem_pack(buf, &data, 8);
    }
};

void setup() {
    MCP_CAN can(SPI_CS_PIN);
    Serial.begin(9600);
    while (CAN_OK != can.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");
    
    /* #define RPUL 5 */ // For camera mast
    /* #define RDIR 4 */
    #define RPUL 10
    #define RDIR 6
    #define LPUL 4
    #define LDIR 11
    #define MIN_LIMIT A0
    #define MAX_LIMIT A1
    StepperController control(RPUL, RDIR, LPUL, LDIR, MIN_LIMIT, MAX_LIMIT);

    bool eStopped = false;
    for (;;) {

        // TODO: investigate if a non-blocking can_read would allow us to reduce
        // ticks_per_loop to reduce command latency.
        // So don't have `while (can.checkReceive)`, just do the call and then continue
        // if !CAN_MSGAVAIL.
        /* CANPacket packet = can_read(can); */
        CANPacket packet = {0xFFFFFFFF, 0};
        for (int i = 0; i < 64; i++) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned char len;
                if (can.readMsgBuf(&len, packet.buf) == CAN_OK) {
                    packet.len = len;
                    packet.id = can.getCanId();
                    break;
                }
            }
        }
        
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
            }
        }

        // Send Telemetry
        CANPacket telemetry = {DAVID_STEPPER_TELEM_FRAME_ID, 0};
        control.pack_telemetry(telemetry.buf);
        can_send(can, telemetry);

        if (eStopped)
            continue;

        /* if (packet.id == DAVID_STEPPER_CTRL_FRAME_ID) { */
        /*     david_stepper_ctrl_t frame; */
        /*     david_stepper_ctrl_unpack(&frame, packet.buf, packet.len); */
        /*     control.state = (frame.home)? control.HOME : control.MOVE; */
        /*     control.setPoint(frame.set_point); */
        /* } */
        
        switch (packet.id) {
            FRAME_CASE(DAVID_STEPPER_CTRL, david_stepper_ctrl) {
                /* david_stepper_ctrl_t frame; */
                /* david_stepper_ctrl_unpack(&frame, packet.buf, packet.len); */
                control.state = (frame.home)? control.HOME : control.MOVE;
                control.setPoint(frame.set_point);
            }
        }


        /* int ticks = control.ticks_per_loop; */
        /* while (ticks-- > 0) { */
            /* control.doStep(control.step++); */
        /* } */
        control.loop();

        /* for (int i = 0; i < 100; i++) { */
        /*     control.doStep(control.step++); */
        /*     delayMicroseconds(50); */
        /* } */
        /* for (int i = 0; i < 100; i++) { */
        /*     control.doStep(control.step--); */
        /*     delayMicroseconds(50); */
        /* } */

    }
}
