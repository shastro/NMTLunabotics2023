// receive a frame from can bus

#include "arduino_lib.hpp"

// Import CAN message constants
#include "david.h"
// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)

// Pins
#define HALL_PIN_L 2
#define HALL_PIN_R 3

// static const int64_t trig_delay  = 22500; // Microsecond delay
static const int64_t trig_delay  = 16000; // Microsecond delay
// Period is 15k mus so we wait 1.5 times the period
static const int64_t DEFAULT_HOMING_DELAY = 300;
static const double MM_PER_COUNT = 0.17896;


enum class Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};


struct ControlCommand {
    double set_point = 0.0;
    double left_offset = 0.0;
    double right_offset = 0.0;
    bool home = false;
};

struct MotorState {
    volatile Dir direction = Dir::Stop;
    volatile int count = 0;
    volatile int64_t last_trigger = 0;

};

ControlCommand current_command;
MotorState left_motor;
MotorState right_motor;

void pack_telemetry(unsigned char buf[8], bool home_done) {
    david_pitch_position_telem_t data = {0};

    data.left_position = david_pitch_position_telem_left_position_encode((double)left_motor.count * MM_PER_COUNT);
    data.right_position = david_pitch_position_telem_right_position_encode((double)right_motor.count * MM_PER_COUNT);
    data.left_direction = (int)left_motor.direction;
    data.right_direction = (int)right_motor.direction;
    data.home_done = david_pitch_position_telem_home_done_encode(home_done);

    david_pitch_position_telem_pack(buf, &data, 8);
}




bool home_done = false;

const int required_num_duplicates = 10;
void home(MCP_CAN &can) {
    delay(1500);
    int prev_count_l = 0xFFFFFFFF;
    int prev_count_r = 0xFFFFFFFF;
    int left_duplicates = 0;
    int right_duplicates = 0;

    home_done = false;

    int loop_count = 0;
    int telem_freq = 3;
    for (;;) {
        if (left_motor.count == prev_count_l) {
            left_duplicates++;
        } else {
            left_duplicates = 0;
        }
        if (right_motor.count == prev_count_l) {
            right_duplicates++;
        } else {
            right_duplicates = 0;
        }
        // Serial.print("Left dups: ");
        // Serial.println(left_duplicates);
        // Serial.println(left_motor.count);
        prev_count_l = left_motor.count;
        prev_count_r = right_motor.count;

        if (loop_count % telem_freq == 0){
            CANPacket position_telemetry(DAVID_PITCH_POSITION_TELEM_FRAME_ID);
            pack_telemetry(position_telemetry.buf, home_done);
            can_send(can, position_telemetry);
        }

        delay(30);
        if ((left_duplicates > required_num_duplicates) && (right_duplicates > required_num_duplicates)) {
            current_command.home = false;
            home_done = true;
            left_motor.count = 151.5 / MM_PER_COUNT;
            right_motor.count = 151.5 / MM_PER_COUNT;
            break;
        }
        loop_count++;
    }

}

void setup()
{
    MCP_CAN can = setup_can();
    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), left_hall_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), right_hall_handler, FALLING);

    bool has_recieved_home_command = false;
    bool e_stopped = false;
    long loop_count = 0;
    int telem_freq = 2;

    for(;;){
        CANPacket packet = can_read_blocking(can);
        if (packet.id == 0xFFFFFFFF) {
            delay(5);
            continue;
        }
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                e_stopped = frame.stop;
            }
        }


        if (e_stopped) {
            continue;
        }

        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_DRIVER_TELEM, david_pitch_driver_telem) {
                left_motor.direction =  (Dir)frame.left_direction;
                right_motor.direction = (Dir)frame.right_direction;
            }
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {
                current_command.set_point = david_pitch_ctrl_set_point_decode(frame.set_point);
                current_command.left_offset = david_pitch_ctrl_left_offset_decode(frame.left_offset);
                current_command.right_offset = david_pitch_ctrl_right_offset_decode(frame.right_offset);
                current_command.home = david_pitch_ctrl_home_decode(frame.home);

            }
            
        }

        if ((loop_count % telem_freq) == 0) {
            CANPacket position_telemetry(DAVID_PITCH_POSITION_TELEM_FRAME_ID);
            pack_telemetry(position_telemetry.buf, home_done);
            can_send(can, position_telemetry);
        }

        if (current_command.home) {
            has_recieved_home_command = true;
            left_motor.direction = Dir::Extend;
            right_motor.direction = Dir::Extend;
            home(can);
        } 

        // Clear home_done flag after awhile
        if (has_recieved_home_command && home_done && (loop_count % 50 == 0)){
            has_recieved_home_command = false;
            home_done = false;
        }

      
        loop_count++;
    }
}

void left_hall_handler(){
    int64_t timestamp = micros();
    if ((timestamp - left_motor.last_trigger) > trig_delay) {
        left_motor.last_trigger = timestamp;
        switch (left_motor.direction) {
        case Dir::Retract:
            left_motor.count -= 1;
            break;
        case Dir::Extend: 
            left_motor.count += 1;
            break;
        }
    } 
}

void right_hall_handler(){
    int64_t timestamp = micros();
    if ((timestamp - right_motor.last_trigger) > trig_delay) {
        right_motor.last_trigger = timestamp;
        switch (right_motor.direction) {
        case Dir::Retract:
            right_motor.count -= 1;
            break;
        case Dir::Extend: 
            right_motor.count += 1;
            break;
        }
    } 
}



