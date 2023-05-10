// Shared library code for Arduinos.

#ifndef ARDUINO_LIB_H
#define ARDUINO_LIB_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#define sign(i) (((i) > 0) - ((i) < 0))

// Pinout reference:
// D4  RX
// D5* TX
// D6* SDA
// A3  SCL
// 12  D8
// A0  D9*
// A1  10*
// A2  11*
// 5V  GND
// (* = PWM)

// Pin CAN runs on.
#define SPI_CS_PIN 17

// Labels for pins.
#define D4 4
#define D5 5
#define D6 6
#define D8 8
#define D9 9
#define D13 13

// Chase tells me this is correct ~~Alex
#define RX 0

// Pin that has the LED attached.
#define BOARD_LED D13

// Lock up the board, send an error message over serial, and flash the
// LED forever.
//
// This should *only* be used for hardware configuration errors that
// are completely unrecoverable.
inline void panic(const char *error_message);

// Generate a case label for unpacking a particular frame kind from
// `packet`. `frame_upper` and `frame_lower` should be the name of the
// frame, in SCREAMING_SNAKE_CASE and snake_case respectively.
//
// FRAME_CASE(FOO, foo) {
//     ...
// }
// is equivalent to
// case FOO_FRAME_ID: {
//     foo_t frame;
//     foo_unpack(&frame, packet.buf, packet.len);
//     ...
// }
#define FRAME_CASE(frame_upper, frame_lower)                                   \
    break;                                                                     \
    case frame_upper##_FRAME_ID:                                               \
        for (bool once = true; once; once = false)                             \
            for (frame_lower##_t frame;                                        \
                 frame_lower##_unpack(&frame, packet.buf, packet.len), once;   \
                 once = false)

struct CANPacket {
    uint32_t id;
    uint32_t len;
    unsigned char buf[8];
};

// Set up a CAN connection.
inline MCP_CAN setup_can() {
    MCP_CAN can(SPI_CS_PIN);
    Serial.begin(9600);
    while (can.begin(CAN_500KBPS) != CAN_OK) {
        Serial.println("CAN bus fail!");
        delay(100);
    }
    Serial.println("CAN bus ok!");
    return can;
}

// Read a CAN packet. Blocks until one is available.
inline CANPacket can_read(MCP_CAN &can) {
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

inline void can_send(MCP_CAN &can, CANPacket packet) {
    can.sendMsgBuf(packet.id, CAN_STDID, 8, packet.buf);
}

// Output pins.
class OutPin {
    int num;
    bool allow_pwm;

  public:
    OutPin(int num) : num(num) {
        pinMode(num, OUTPUT);

        // Only some pins are allowed to do PWM output. This only runs
        // once at init, so isn't much of a speed concern.
        switch (num) {
        case 3:
        case 5:
        case 6:
        case 9:
        case 10:
        case 11:
            allow_pwm = true;
            break;

        default:
            allow_pwm = false;
        }
    }

    void write(bool value) { digitalWrite(num, value ? HIGH : LOW); }
    void write_pwm(double duty_cycle) {
        if (!allow_pwm)
            panic("Attempted PWM on invalid PWM pin");

        analogWrite(num, min(duty_cycle, 1) * 255);
    }
};

// Input pins.
class InPin {
    int num;
    float threshold;

  public:
    InPin(int num, float threshold = 0.8f) : num(num), threshold(threshold) {
        pinMode(num, INPUT);
    }
    bool read() { return digitalRead(num) == HIGH; }
    float read_analog() { return (analogRead(num) / 1023.0); }
    bool read_threshold() { return read_analog() >= threshold; }
};

// Lock up the board, send an error message over serial, and flash the
// LED forever.
inline void panic(const char *error_message) {
    // Panic and make the LED blink forever.
    OutPin led(BOARD_LED);
    while (true) {
        Serial.println(error_message);
        led.write(false);
        delay(500);
        led.write(true);
        delay(500);
    }
}

// Relay controller.
struct Relay {
    OutPin left;
    OutPin right;
    OutPin common;

    Relay(int left, int right, int common)
        : left(left), right(right), common(common) {}

    // Sets the relay to output on its left pin, and returns a digital
    // output pin that is now connected to the left side of the relay.
    OutPin &output_left() {
        left.write(false);
        right.write(true);
        return common;
    }

    // Sets the relay to output on its right pin, and returns a digital
    // output pin that is now connected to the right side of the relay.
    OutPin &output_right() {
        left.write(true);
        right.write(false);
        return common;
    }
};

// Single Stepper Motor
struct Stepper {
    OutPin pulse;
    OutPin direction;

    Stepper(int pulse_pin, int dir_pin)
        : pulse(pulse_pin), direction(dir_pin) {}

    void doStep(unsigned int step) {
        const int pulse_sequence[] = {HIGH, HIGH, LOW, LOW};
        const int dir_sequence[] = {LOW, HIGH, HIGH, LOW};
        pulse.write(pulse_sequence[step % 4]);
        direction.write(dir_sequence[step % 4]);
    }
};

// Motor controller object.
class MidwestMotorController {
    OutPin enable;
    Relay relay;

  public:
    MidwestMotorController(int inhibit, int relay_ccw, int relay_cw,
                           int relay_pwm)
        : enable(inhibit), relay(relay_ccw, relay_cw, relay_pwm) {
        setVel(0);
    }

    MidwestMotorController(int inhibit, Relay relay)
        : enable(inhibit), relay(relay) {
        setVel(0);
    }

    const double vel_scale = 100.0;
    const double vel_deadzone = 0.01;
    void setVel(double vel) {
        vel /= vel_scale;
        // Experimentally determined that the motors' maximum velocity
        // is somewhere around 0.45-0.5 duty cycle. It's unclear why,
        // but as a result for better control we can just multiply by
        // 0.5;
        vel *= 0.5;

        // The motors are slightly more responsive when going forwards
        // than when going backwards.
        if (vel < 0)
            vel *= 1.5;

        enable.write(abs(vel) > vel_deadzone);
        if (vel > 0)
            relay.output_right().write_pwm(abs(vel));
        else
            relay.output_left().write_pwm(abs(vel));
    }
};

#endif // ARDUINO_LIB_H
