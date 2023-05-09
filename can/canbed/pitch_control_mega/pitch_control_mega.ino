// receive a frame from can bus

#include <Wire.h>
#include "mcp_can.h"

// Import CAN message constants
#include "david.h"

// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)

// Pins
#define HALL_PIN_L 18
#define HALL_PIN_R 19

// I2C Registers
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current
#define STATUSREG           0x01
#define ACCREG              0x03

static const int64_t DEFAULT_TRIG_DELAY = 10000; // Microsecond delay
static const int64_t DEFAULT_MAX_COUNT = 875-20;
static const int64_t DEFAULT_HOMING_DELAY = 300;
static const int64_t DEFAULT_TOLERANCE = 3;


I2C CAN(SPI_CS_PIN); // Set CS pin

struct MDO4Driver {
    unsigned addr;  

    MDO4Driver(unsigned addr_in) {
        Wire.begin();
        addr = addr_in;
    }

    byte getTemp() {
        return getData(TEMPREG);
    }

    byte getCurrent() {
        return getCurrent(CURRENTREG);
    }

    void setSpeed(byte speed) {
        sendData(SPEEDBYTE, speed);
    }

    void setAcceleration(byte speed) {
        sendData(ACCREG, speed);
    }
        
    void setDirection(Dir dir){
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
        delay(10);
    }

};


struct PitchController {
    double tolerance;

    enum Dir {
        Extend,
        Retract,
        Stop,
    };
};


void setup()
{
    MDO4Driver left_m();
    delay(100);

    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_L, left_hall_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN_R, right_hall_handle, FALLING);

    controller.loop();
}

void loop()
{
    // Check for new messages
    // Standard movement towards target counts


    const static int MOTOR_ADDRESS[] = {0x59, 0x58};
    sendData(MOTOR_ADDRESS[m],ACCREG, 0x0A);
    sendData(MOTOR_ADDRESS[m],SPEEDBYTE, 0xAA);
    sendData(MOTOR_ADDRESS[m],CMDBYTE, Motors[m].dir);
    
}


