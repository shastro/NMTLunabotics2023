// receive a frame from can bus

#include <Wire.h>
#include "mcp_can.h"

// Import CAN message constants
#include "david.h"

enum MOTORS {
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_COUNT
};

enum PINS {
    HALL_PIN = 0,
    PIN_COUNT,
};
#define SPI_CS_PIN 17

enum DIRS {
    STOP = 0,
    EXTEND = 1,
    RETRACT = 2,
    HOMING = 3,
};

// Pins
#define HALL_PIN_L 5
#define HALL_PIN_R 6

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
static const int64_t DEFAULT_HOMING_DELAY = 100;
static const int64_t DEFAULT_TOLERANCE = 3;

static const int PIN_TABLE[MOTOR_COUNT][3] = {
    { HALL_PIN_L, }, // P1_L, P2_L  },
    { HALL_PIN_R, }, // P1_R, P2_R },
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

struct MotorState {
    int64_t tolerance;
    int64_t last_trigger;
    volatile int64_t true_count;
    int64_t prev_count;
    int64_t target_count;
    int dir;
    int last_dir;
    int stationary_timer;
};
struct MotorState Motors[MOTOR_COUNT];
#define ForEachMotor for(int m = 0; m < MOTOR_COUNT; m++)

static int64_t max_count = DEFAULT_MAX_COUNT;
static int64_t trig_delay = DEFAULT_TRIG_DELAY;
static int64_t homing_delay = DEFAULT_HOMING_DELAY;

bool estopped = false;
bool homing = true;

void setup()
{
    Wire.begin();
    delay(100);

    // PinModes
    pinMode(PIN_TABLE[MOTOR_LEFT][HALL_PIN], INPUT_PULLUP);
    pinMode(PIN_TABLE[MOTOR_RIGHT][HALL_PIN], INPUT_PULLUP);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_LEFT][HALL_PIN]), left_hall_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_RIGHT][HALL_PIN]), right_hall_handler, FALLING);

    Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

}

void loop()
{
    unsigned char len = 0;
    unsigned char buf[8] = {0};

    // Check for new messages
    if(CAN_MSGAVAIL == CAN.checkReceive()) {
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

        if (canId == DAVID_SET_TRIG_DELAY_FRAME_ID) {
            trig_delay = extract_value(buf, 0, 6);
            max_count = extract_value(buf, 6, 2);
        }

        // Enter homing state
        if (!estopped && !homing) {    
            if (canId == DAVID_PITCH_CTRL_HOME_FRAME_ID) {
                homing = true;
                Motors[MOTOR_LEFT].prev_count = 0;
                Motors[MOTOR_RIGHT].prev_count = 0;
                Motors[MOTOR_LEFT].target_count = 0;
                Motors[MOTOR_RIGHT].target_count = 0;
            }
        }

        // Set lengths
        if (!estopped && !homing) {    
            // If PITCH_CTRL_BOTH both if statements will fire.
            if (canId == DAVID_PITCH_CTRL_BOTH_FRAME_ID) {
                Motors[MOTOR_LEFT].target_count = extract_value(buf, 0, 6);
                Motors[MOTOR_RIGHT].target_count = extract_value(buf, 0, 6);
                Motors[MOTOR_LEFT].tolerance = extract_value(buf, 6, 2);
                Motors[MOTOR_RIGHT].tolerance = extract_value(buf, 6, 2);
            }
            if (canId == DAVID_PITCH_CTRL_LEFT_FRAME_ID) {
                Motors[MOTOR_LEFT].target_count = extract_value(buf, 0, 6);
                Motors[MOTOR_LEFT].tolerance = extract_value(buf, 6, 2);
            }
            if (canId == DAVID_PITCH_CTRL_RIGHT_FRAME_ID) {
                Motors[MOTOR_RIGHT].target_count = extract_value(buf, 0, 6);
                Motors[MOTOR_RIGHT].tolerance = extract_value(buf, 6, 2);
            }

            ForEachMotor {
                Motors[m].target_count = constrain(Motors[m].target_count, 0, max_count);
            }
        }

        // Print CAN message
        Serial.println("-----------------------------");
        Serial.println(canId, HEX);

    } // finish CAN message

    // Do homing
    if (!estopped && homing) {
        ForEachMotor {
            Motors[m].dir = HOMING;

            if (Motors[m].true_count == Motors[m].prev_count) {
                Motors[m].stationary_timer++;
            } else {
                Motors[m].stationary_timer = 0;
            }

            Motors[m].prev_count = Motors[m].true_count;
        
            if (Motors[m].stationary_timer > homing_delay) {
                Motors[m].dir = STOP;
                Motors[m].true_count = 0;
                Motors[m].target_count = 0;
            }
        }

        if ((Motors[MOTOR_LEFT].dir != HOMING) && (Motors[MOTOR_RIGHT].dir != HOMING)) {
            homing = false;
        }
    }

    // Standard movement towards target counts
    if (!estopped && !homing) {
        ForEachMotor {
            int newdir = STOP; // Should be stop
            if (abs(Motors[m].true_count - Motors[m].target_count) >= DEFAULT_TOLERANCE) {
                if (Motors[m].true_count > Motors[m].target_count) {
                    newdir = RETRACT;  
                } else {
                    newdir = EXTEND;  
                }
            }
            Motors[m].dir = newdir;
        }
    }
    
    ForEachMotor {
        const static int MOTOR_ADDRESS[] = {0x59, 0x58};
        sendData(MOTOR_ADDRESS[m],ACCREG, 0x03);
        sendData(MOTOR_ADDRESS[m],SPEEDBYTE, 0xAA);
        int dir = (Motors[m].dir == HOMING)? RETRACT : Motors[m].dir;
        sendData(MOTOR_ADDRESS[m],CMDBYTE, dir);
    }
    
    // Send telemetry
    send_telemetry(DAVID_PITCH_TELEM_LEFT_FRAME_ID, Motors[MOTOR_LEFT].true_count, Motors[MOTOR_LEFT].dir);
    send_telemetry(DAVID_PITCH_TELEM_RIGHT_FRAME_ID, Motors[MOTOR_RIGHT].true_count, Motors[MOTOR_RIGHT].dir);

}

void send_telemetry(uint32_t canId, int64_t true_count, int64_t dir){
    unsigned char buf[8] = {0};
    buf[0] = true_count >> 8*0;
    buf[1] = true_count >> 8*1;
    buf[2] = true_count >> 8*2;
    buf[3] = true_count >> 8*3;
    buf[4] = true_count >> 8*4;
    buf[5] = true_count >> 8*5;
    buf[6] = dir >> 8*0;
    buf[7] = dir >> 8*1;
    CAN.sendMsgBuf(canId, 0, 8, buf);
}

void left_hall_handler(){
    int64_t timestamp = micros();
    if ((timestamp - Motors[MOTOR_LEFT].last_trigger) > trig_delay) {
        Motors[MOTOR_LEFT].last_trigger = timestamp;
        switch (Motors[MOTOR_LEFT].dir) {
        case RETRACT:
        case HOMING:
            Motors[MOTOR_LEFT].true_count -= 1;
            break;
        case EXTEND: 
            Motors[MOTOR_LEFT].true_count += 1;
            break;
        case STOP: 
            break;
        }
    } 
}

void right_hall_handler(){
    int64_t timestamp = micros();
    if ((timestamp - Motors[MOTOR_RIGHT].last_trigger) > trig_delay) {
        Motors[MOTOR_RIGHT].last_trigger = timestamp;
        switch (Motors[MOTOR_RIGHT].dir) {
        case RETRACT: 
        case HOMING:
            Motors[MOTOR_RIGHT].true_count -= 1;
            break;
        case EXTEND: 
            Motors[MOTOR_RIGHT].true_count += 1;
            break;
        case STOP: 
            break;
        }
    } 
}

int64_t extract_value(char buf[], int first_byte, int bytes) {
    int64_t count = 0;
    int64_t temp = 0;
    for (int i = first_byte; i< first_byte+bytes; i++) {
        temp = (int64_t)buf[i];
        count += temp << (8*i);
    }
    return count;  
}

void sendData(byte addr, byte reg, byte val){      // Function for sending data to MD03
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

byte getData(byte addr, byte reg){                 // function for getting data from MD03
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    byte mdo3 = 1;
    Wire.requestFrom(addr, mdo3);         // Requests byte from MD03
    while(Wire.available() < 1);          // Waits for byte to become available
    byte data = Wire.read();

    return(data);
}
