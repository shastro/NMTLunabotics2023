// locomotion controllers
// left or right rotation, speed parameter?
// feedback is 1 pin, # of rotations??

// We probably want an api where we set a speed for each motor,
// not a spin count. 

#include <SPI.H>
#include <Wire.h>
#include "mcp_can.h"

#include "david.h"

enum PINS {
    PWM_1 = 0, // \_ Differential signals
    PWM_2 = 1, // /
    PIN_COUNT,
};

enum DIR {
    STOP = 0,
    RIGHT = 1,
    LEFT = 2,
};

enum MOTOR {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_BUCKET = 2,
    MOTOR_COUNT
};

struct MotorState {
    enum DIR dir; // maybe dont need? speeds should be signed
    int target_speed;
    int actual_speed;

    // Telemetry
    int spin_counts;
    int last_hall_trigger;
};

static const int MOTOR_PIN_TABLE[MOTOR_COUNT][8] = {
    // PWM_1, PWM_2 
    {05, 06, 0},
    {03, 09, 0},
    {10, 11, 0},
};

// State
static MotorState MOTOR_STATES[MOTOR_COUNT] = {
    {0},
    {0},
    {0}
};

static const bool e_stopped = false;

void setup() {
    Wire.begin();
    delay(100);

    // Pins
    for (int m = 0; m < MOTOR_COUNT; m++) {
        pinMode(PIN_TABLE[m][P1]); // ? what pin modes / pins
    }
    
    // Interrupts

    // Setup Can + Serial
    Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
        {
            Serial.println("CAN BUS FAIL!");
            delay(100);
        }
    Serial.println("CAN BUS OK!");
}

void loop() {
    // Receive CAN Messages
    if(CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8] = {0};

        // read data
        CAN.readMsgBuf(&len, buf);
        uint32_t canId = CAN.getCanId();

        if (canId == DAVID_E_START_FRAME_ID) {
            estopped = false;
        }
    
        // IF ESTOP stop other behaviors
        if (canId == DAVID_E_STOP_FRAME_ID) {
            estopped = true;
        }


        // Print CAN message
        Serial.println("-----------------------------");
        Serial.println(canId, HEX);
    }

    // Standard rotation
    if (!estopped) {
        
    }

    // Set Control states
    for (int m = 0; m < MOTOR_COUNT; m++) {
        // for sure we are gonna have to map speed to 0-255,
        // however we dont know how to do that for left/right yet.
        int voltage = speed; // what voltage to write ? (0-255)
        analogWrite(MOTOR_PIN_TABLE[m][?], val) // what pins to write to?
    }
    

    // Telemetry

    
}
