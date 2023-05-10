// receive a frame from can bus

#include <Wire.h>
#include "arduino_lib.hpp"
#include "Longan_I2C_CAN_Arduino.h"

// Import CAN message constants
#include "david.h"
// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)

// Pins
#define HALL_PIN_L 18
#define HALL_PIN_R 19

// I2C Registers
#define SOFTREG             0x07                    // Byte to read software
#define CMDREG              0x00                    // Command byte
#define SPEEDREG            0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current
#define STATUSREG           0x01
#define ACCREG              0x03

static const int64_t DEFAULT_TRIG_DELAY = 10000; // Microsecond delay
static const int64_t DEFAULT_MAX_COUNT = 875-20;
static const int64_t DEFAULT_HOMING_DELAY = 300;
static const int64_t DEFAULT_TOLERANCE = 3;


#define I2C_CAN_ADDR 0x25

enum Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};

#define ACC 0x0A
#define SPEED 150

struct MDO4Driver {
    int addr;  

    MDO4Driver(unsigned addr_in) {
        addr = addr_in;
    }

    byte getTemp() {
        return getData(TEMPREG);
    }

    byte getCurrent() {
        return getData(CURRENTREG);
    }

    void setDirection(Dir dir){
        sendData(ACCREG, ACC);
        sendData(SPEEDREG, SPEED);
        sendData(CMDREG, dir);
    }

    byte getData(byte reg){                 // function for getting data from MD03
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom(addr, 1);         // Requests byte from MD03
        while(Wire.available() < 1);          // Waits for byte to become available
        byte data = Wire.read();

        return(data);
    }

    void sendData(byte reg, byte val){      // Function for sending data to MD03
        Wire.beginTransmission(addr);      // Send data to MD03
        Wire.write(reg);                    // Command like Direction, Speed
        Wire.write(val);                    // Value for the command
        int error = Wire.endTransmission();
        if(error) {
            Serial.print("I2C ERROR:");
            Serial.println(error);
        }
        delay(10);
    }

};


struct PitchController {
    double tolerance;
    enum States {
        Move = 0,
        Home = 1,
    };

};


void setup()
{
    I2C_CAN can = setup_can();
    Wire.begin();
    MDO4Driver left_m(0x59);
    MDO4Driver right_m(0x58);
    delay(100);

    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    PitchController control(left_m, right_m);
    // Interrupts
    // attachInterrupt(digitalPinToInterrupt(HALL_PIN_L, left_hall_handler, FALLING);
    // attachInterrupt(digitalPinToInterrupt(HALL_PIN_R, right_hall_handle, FALLING);

    double current_set_point = 0.0;
    // controller.loop();
    for(;;){
        int CMD_State = Dir::Stop;
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
                CMD_State = Dir::Stop;
            }
        }

        CANPacket driver_telemetry = {DAVID_PITCH_POSITION_TELEM_FRAME_ID, 0};
        CANPacket driver_telemetry = {DAVID_PITCH_DRIVER_TELEM_FRAME_ID, 0};
        if (eStopped)
            continue;

        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {
                
                if (frame.setpoint) {
                    control.state = control.HOME;
                } else {
                    control.setPoint(david_stepper_ctrl_set_point_decode(frame.set_point));
                }
            }
        }

        control.loop();
        left_m.setDirection(Dir::Retract);
        right_m.setDirection(Dir::Retract);
        // left_m.setDirection(Dir::Stop);
        // right_m.setDirection(Dir::Stop);
        // delay(2000);
        // left_m.setDirection(Dir::Extend);
        // right_m.setDirection(Dir::Extend);
        // delay(2000);
        // left_m.setDirection(Dir::Stop);
        // right_m.setDirection(Dir::Stop);
        // delay(2000);
    }
}
