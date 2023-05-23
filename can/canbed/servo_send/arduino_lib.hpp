// Shared library code for Arduinos.
#ifndef ARDUINO_LIB_H
#define ARDUINO_LIB_H

#include "pins_arduino.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include <wiring_private.h>

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

// Sentinel ID value to indicate that a CAN packet is empty.
#define CAN_PACKET_EMPTY 0xffffffff

struct CANPacket {
    uint32_t id = CAN_PACKET_EMPTY;
    uint32_t len = 0;
    unsigned char buf[8];

    CANPacket() {}
    CANPacket(uint32_t id) : id(id) {}

    // This allows code like the following:
    // CANPacket pkt = can_read_nonblocking(can);
    // if (pkt) {
    //     // this is only entered if `can_read_nonblocking` succeeded.
    // }
    operator bool() const { return id != CAN_PACKET_EMPTY; }
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

// Read a CAN packet. Does not block and immediately returns a
// CANPacket that coerces to `false` if none is available.
inline CANPacket can_read_nonblocking(MCP_CAN &can) {
    CANPacket packet;
    if (can.checkReceive() == CAN_MSGAVAIL) {
        can.readMsgBuf((unsigned char *)&packet.len, packet.buf);
        packet.id = can.getCanId();
        return packet;
    } else {
        return packet;
    }
}

// Read a CAN packet. Blocks until one is available.
inline CANPacket can_read_blocking(MCP_CAN &can) {
    while (can.checkReceive() != CAN_MSGAVAIL)
        ;

    return can_read_nonblocking(can);
}

// Use can_read_nonblocking or can_read_blocking instead.
// [[deprecated]] inline CANPacket can_read(MCP_CAN &can) {
//     return can_read_nonblocking(can);
// }

// arduino_cli doesn't emit warnings for deprecated functions; I'm not
// bothering deprecating it and I'm just gonna delete the function
// instead. ~~Alex

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
int direction_voltages[3] = {1, 0, 1};
struct Stepper {
    OutPin pulse;
    OutPin dirpin;
    long count;
    enum Dirs { STOP = 0, EXTEND = 1, RETRACT = 2 };
    enum Dirs dir; 

    Stepper(int pulse_pin, int dir_pin)
        : pulse(pulse_pin), dirpin(dir_pin) {
        count = 0;
        dir = STOP;
    }

    void setDirection(unsigned int d) {
        dir = d;
        dirpin.write(direction_voltages[dir]);
    }

    inline void doStep() {
        int dir_table[] = {0, 1, -1};
        count += dir_table[dir];
        if (dir != STOP) {
            pulse.write(0);
            pulse.write(1);
        }
    }
};

// Motor controller object.
struct MidwestMotorController {
    OutPin enable;
    Relay relay;

    // Experimentally determined that the motors' maximum velocity is
    // somewhere around 0.45-0.5 duty cycle. It's unclear why, but as
    // a result for better control we can just multiply by 0.5; make
    // this adjustable though.
    double post_scale = 0.5;

  public:
    MidwestMotorController(int inhibit, int relay_ccw, int relay_cw,
                           int relay_pwm)
        : enable(inhibit), relay(relay_ccw, relay_cw, relay_pwm) {
        setVel(0.0);
    }

    MidwestMotorController(int inhibit, Relay relay)
        : enable(inhibit), relay(relay) {
        setVel(0.0);
    }

    MidwestMotorController(int inhibit, Relay relay, double post_scale)
        : enable(inhibit), relay(relay), post_scale(post_scale) {
        setVel(0.0);
    }

    const double vel_scale = 100.0;
    const double vel_deadzone = 0.01;
    void setVel(double vel) {
        vel /= vel_scale;
        vel *= post_scale;

        // The motors are slightly more responsive when going forwards
        // than when going backwards.
        if (vel > 0)
            vel *= 1.5;

        enable.write(abs(vel) > vel_deadzone);
        if (vel > 0) {
            relay.output_right().write_pwm(abs(vel));
        } else {
            relay.output_left().write_pwm(abs(vel));
        }
    }
};

// Experimental scheduler type for Arduino. The idea is that you
// should be able to chain together functions like so:
// scheduler()
//     .schedule(100, [&]() {
//         // Every 100ms print "hello"
//         Serial.println("hello");
//     })
//     .schedule(150, [&]() {
//         // Every 150ms print "world"
//         Serial.println("world");
//     })
//     .run();
// The lambda functions will be run periodically on simultaneous
// timers, without dynamic memory allocation occurring.
template <typename Fn, typename Rest> class Schedule {
    // Delay between now and the next occurrence of the event.
    double delay;

    // Delay between occurrences of the event.
    double period;

    // Function to run when the `delay` reaches zero.
    Fn function;

    // The rest of the events.
    Rest rest;

  public:
    Schedule(Fn function, Rest rest, double period)
        : delay(period), period(period), function(function), rest(rest) {}

    // Add a new function to the schedule.
    template <typename NextFn>
    Schedule<NextFn, Schedule<Fn, Rest>> schedule(double period,
                                                  NextFn function) {
        return Schedule<NextFn, Schedule<Fn, Rest>>(function, *this, period);
    }

    // Calculate the delay between now and the next event that will
    // occur.
    double min_delay() { return min(delay, rest.min_delay()); }

    // Run any events whose delays have reached zero.
    void run_event() {
        if (delay < 0.000001) {
            function();
            delay = period;
        }

        rest.run_event();
    }

    // Subtracts `amount` from the delays on the schedule.
    void tick(double amount) {
        delay -= amount;
        rest.tick(amount);
    }

    // Waits for some number of milliseconds.
    void sleep(int ms) { rest.sleep(ms); }

    // Runs the schedule forever.
    void run() {
        while (true) {
            double m_delay = min_delay();
            sleep(m_delay);
            tick(m_delay);
            run_event();
        }
    }
};

// A schedule that does nothing ever.
class NullSchedule {
  public:
    NullSchedule();

    template <typename NextFn>
    Schedule<NextFn, NullSchedule> schedule(double period, NextFn function) {
        return Schedule<NextFn, NullSchedule>(function, *this, period);
    }

    double min_delay() { return 1000000; }

    void run_event() {}

    void sleep(int ms) { delay(ms); }

    void tick(double amount) {}
};

// A schedule that does something as often as possible between other
// tasks.
template <typename Fn> class DenseSchedule {
    Fn function;

  public:
    DenseSchedule(Fn function) : function(function) {}

    template <typename NextFn>
    Schedule<NextFn, DenseSchedule<Fn>> schedule(double period,
                                                 NextFn function) {
        return Schedule<NextFn, DenseSchedule<Fn>>(function, *this, period);
    }

    double min_delay() { return 1000000; }

    void run_event() {}

    void sleep(int ms) {
        uint32_t start = micros();

        while (ms > 0) {
            function();
            while (ms > 0 && (micros() - start) >= 1000) {
                ms--;
                start += 1000;
            }
        }
    }

    void tick(double amount) {}
};

// Creates a new empty schedule.
inline NullSchedule scheduler(void) { return NullSchedule(); }

// Creates a new dense schedule.
template <typename Fn> inline DenseSchedule<Fn> scheduler_dense(Fn function) {
    return DenseSchedule<Fn>(function);
}

// Captured data to be sent to interrupt handlers.
static void *interrupt_data[EXTERNAL_NUM_INTERRUPTS];

// Interrupt callback for a particular interrupt number and handler
// function.
template <uint8_t num, typename Func> inline void callback_interrupt() {
    Func &f = *(Func *)interrupt_data[digitalPinToInterrupt(num)];
    f();
}

// Attach an interrupt function to a particular pin. Use e.g.
// attach_interrupt<8>([&]() { count++; }, FALLING) to count how many
// times digital pin 8 falls.
template <uint8_t num, typename Func>
inline void attach_interrupt_lambda(Func &func, int mode) {
    if (digitalPinToInterrupt(num) == NOT_AN_INTERRUPT)
        panic("attempt to set up interrupts on a non-interrupt pin");

    interrupt_data[digitalPinToInterrupt(num)] = (void *)&func;
    attachInterrupt(digitalPinToInterrupt(num), callback_interrupt<num, Func>,
                    mode);
}

#endif // ARDUINO_LIB_H
