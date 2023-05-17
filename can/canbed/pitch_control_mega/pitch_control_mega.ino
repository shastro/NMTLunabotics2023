// receive a frame from can bus

#include "arduino_lib.hpp"

#include "david.h"

// Pins
#define HALL_PIN_L 2
#define HALL_PIN_R 3

static const int64_t trig_delay_retract = 20000; // Microsecond delay
static const int64_t trig_delay_extend = 7000;   // Microsecond delay
// Period is 15k mus so we wait 1.5 times the period
static const double MM_PER_COUNT = 0.17896;

enum class Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};

struct MotorState {
    volatile Dir direction = Dir::Stop;
    volatile int count = 0;
    volatile int64_t last_trigger = 0;
};

CANPacket pack_telemetry(const MotorState &left, const MotorState &right) {
    david_pitch_position_telem_t data = {
        .left_position = david_pitch_position_telem_left_position_encode(
            (double)left.count * MM_PER_COUNT),
        .right_position = david_pitch_position_telem_right_position_encode(
            (double)right.count * MM_PER_COUNT),
        .left_direction = (uint8_t)left.direction,
        .right_direction = (uint8_t)right.direction,
    };

    CANPacket pkt(DAVID_PITCH_POSITION_TELEM_FRAME_ID);
    pkt.len = 8;
    david_pitch_position_telem_pack(pkt.buf, &data, 8);

    return pkt;
}

void hall_handler(MotorState &state) {
    int64_t timestamp = micros();
    switch (state.direction) {
    case Dir::Stop:
        break;
    case Dir::Extend:
        if ((timestamp - state.last_trigger) > trig_delay_extend) {
            state.last_trigger = timestamp;
            state.count++;
        }
        break;
    case Dir::Retract:
        if ((timestamp - state.last_trigger) > trig_delay_retract) {
            state.last_trigger = timestamp;
            state.count--;
        }
        break;
    }
}

void setup() {
    MCP_CAN can = setup_can();
    MotorState left_motor;
    MotorState right_motor;

    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    // Interrupts
    auto left_hall_handler = [&]() { hall_handler(left_motor); };
    auto right_hall_handler = [&]() { hall_handler(right_motor); };

    attach_interrupt_lambda<HALL_PIN_L>(left_hall_handler, FALLING);
    attach_interrupt_lambda<HALL_PIN_R>(right_hall_handler, FALLING);

    // Intentionally ignoring E-stops here, because this is just
    // telemetry and shouldn't care about software e-stop.
    scheduler_dense([&]() {
        CANPacket packet = can_read_nonblocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_DRIVER_TELEM, david_pitch_driver_telem) {
                left_motor.direction = (Dir)frame.left_direction;
                right_motor.direction = (Dir)frame.right_direction;
            }
        }
    })
        .schedule(
            100,
            [&]() { can_send(can, pack_telemetry(left_motor, right_motor)); })
        .run();
}
