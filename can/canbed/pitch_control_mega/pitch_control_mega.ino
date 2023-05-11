// receive a frame from can bus
#include "arduino_lib.hpp"
#include "Longan_I2C_CAN_Arduino.h"

// Import CAN message constants
#include "david.h"
// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)

// Pins
#define HALL_PIN_L 18
#define HALL_PIN_R 19

static const int64_t trig_delay = 10000; // Microsecond delay
static const int64_t DEFAULT_HOMING_DELAY = 300;

#define I2C_CAN_ADDR 0x25

static const int MM_PER_COUNT = 0.17896;

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
    Dir direction = Dir::Stop;
    volatile int count = 0.0;
    double position = 0.0;
    double last_trigger = 0.0;

    void update_pos() {
        position = count * MM_PER_COUNT;
    }
};

inline I2C_CAN setup_can_i2c() {
    I2C_CAN can(I2C_CAN_ADDR);
    Serial.begin(9600);
    while (can.begin(CAN_500KBPS) != CAN_OK) {
        Serial.println("CAN bus fail!");
        delay(100);
    }
    Serial.println("CAN bus ok!");
    return can;
}

inline CANPacket can_read_i2c(I2C_CAN &can) {
    if (can.checkReceive() == CAN_MSGAVAIL) {
        CANPacket packet;
        packet.len = 0;
        packet.id = 0xFFFFFFFF;
        can.readMsgBuf((unsigned char *)&packet.len, packet.buf);
        packet.id = can.getCanId();
        return packet;
    } else {
        CANPacket packet;
        packet.len = 8;
        packet.id = 0xFFFFFFFF;
        return packet;
    }

}
inline void can_send_i2c(I2C_CAN &can, CANPacket packet) {
    can.sendMsgBuf(packet.id, CAN_STDID, 8, packet.buf);
}

inline Dir choose_direction(double position, double set_point, double offset) {
    if (position < (set_point + offset)) {
        return Dir::Retract;
    } else if (position > (set_point + offset)){
        return Dir::Extend;
    } else {
        return Dir::Stop;
    }
}

void pack_telemetry(unsigned char buf[8], MotorState l_state, MotorState r_state, bool home_done) {
    david_pitch_position_telem_t data = {0};

    data.left_position = david_pitch_position_telem_left_position_encode(l_state.position);
    data.right_position = david_pitch_position_telem_right_position_encode(r_state.position);
    data.left_direction = (unsigned char)l_state.direction;
    data.right_direction = (unsigned char)r_state.direction;
    data.home_done = david_pitch_position_telem_home_done_encode(home_done);

    david_pitch_position_telem_pack(buf, &data, 8);
}



ControlCommand current_command;
MotorState left_motor;
MotorState right_motor;

bool home_done = false;

const int required_num_duplicates = 1000;
void home(I2C_CAN can) {
    int prev_count_l = 0xFFFFFFFF;
    int prev_count_r = 0xFFFFFFFF;
    int left_duplicates = 0;
    int right_duplicates = 0;

    home_done = false;
    left_motor.direction = Dir::Extend;
    right_motor.direction = Dir::Extend;
    for (;;) {
        if (left_motor.count == prev_count_l) {
            left_duplicates++;
        }
        if (right_motor.count == prev_count_l) {
            right_duplicates++;
        }
        prev_count_l = left_motor.count;
        prev_count_r = right_motor.count;
        left_motor.update_pos();
        right_motor.update_pos();

        CANPacket position_telemetry = {DAVID_PITCH_POSITION_TELEM_FRAME_ID, 0};
        pack_telemetry(position_telemetry.buf, left_motor, right_motor, home_done);
        can_send_i2c(can, position_telemetry);

        if ((left_duplicates > required_num_duplicates) || (right_duplicates > required_num_duplicates)) {
            left_motor.direction = Dir::Stop;
            right_motor.direction = Dir::Stop;
            current_command.home = false;
            home_done = true;
            break;
        }
    }
}

void setup()
{
    I2C_CAN can = setup_can_i2c();
    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), left_hall_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), right_hall_handler, FALLING);

    double current_set_point = 0.0;
    bool home_state = false;
    bool e_stopped = false;
    // controller.loop();
    for(;;){
        CANPacket packet = can_read_i2c(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                e_stopped = frame.stop;
            }
        }

        // Send Telemetry
        CANPacket position_telemetry = {DAVID_PITCH_POSITION_TELEM_FRAME_ID, 0};
        pack_telemetry(position_telemetry.buf, left_motor, right_motor, home_done);
        can_send_i2c(can, position_telemetry);

        if (e_stopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {

                current_command.set_point = david_pitch_ctrl_set_point_decode(frame.set_point);
                current_command.left_offset = david_pitch_ctrl_left_offset_decode(frame.left_offset);
                current_command.right_offset = david_pitch_ctrl_right_offset_decode(frame.right_offset);
                current_command.home = david_pitch_ctrl_home_decode(frame.home);

                if (frame.home) {
                    break;
                }
            }
        }

        // Update direction
        left_motor.direction = choose_direction(left_motor.position, current_command.set_point, current_command.left_offset);

        right_motor.direction = choose_direction(right_motor.position, current_command.set_point, current_command.right_offset);

        left_motor.update_pos();
        right_motor.update_pos();

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



