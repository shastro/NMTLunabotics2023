// receive a frame from can bus

#include <SPI.h>
#include <Wire.h>
#include "mcp_can.h"

// Import CAN message constants
#include "david.h"

#define FOREACH_MOTOR for (int m = 0; m < MOTOR_COUNT; m++)

enum MOTOR {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT
};
const static int MOTOR_ADDRESS[] = {0x59, 0x58};

// Define pins
// P1 is direction
// P2 is power
enum PINS {
    HALL = 0, // See below tables for right/left motors
    P1 = 1,
    P2 = 2,
    SPI_CS_PIN = 17
};

enum VOLTZ {
    V0 = 0,
    V2_5 = 127,
    V5 = 255
};

enum DIR {
    EXTEND = 1,
    RETRACT = 2,
    STOP = 0,
};

// Pins
#define P1_L 4
#define P2_L 5
#define P1_R 6
#define P2_R 12
#define HALL_L 0
#define HALL_R 1

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
static const int64_t DEFAULT_HOMING_DELAY = 200;
static const int64_t DEFAULT_TOLERANCE = 10;

static const int PIN_TABLE[2][3] = {
    { HALL_L, P1_L, P2_L  },
    { HALL_R, P1_R, P2_R },
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

static int64_t max_count = DEFAULT_MAX_COUNT;
static int64_t trig_delay = DEFAULT_TRIG_DELAY;
static int64_t homing_delay = DEFAULT_HOMING_DELAY;

struct MotorState {
    enum DIR dir;
    enum DIR last_dir;
    int64_t true_count;
    int64_t target_count;
    int64_t prev_count;
    int64_t tolerance;
    int64_t last_trigger;
    int stationary_timer;
};

static MotorState States[MOTOR_COUNT] = {
    {0},
    {0}
};

bool estopped = false;
bool homing = true;

void setup()
{
    Wire.begin();
	delay(100);
    
    // PinModes
    FOREACH_MOTOR {
        pinMode(PIN_TABLE[m][HALL], INPUT_PULLUP);
        pinMode(PIN_TABLE[m][P1], OUTPUT);
        pinMode(PIN_TABLE[m][P2], OUTPUT);
    }
    
    // Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_LEFT][HALL]), left_hall_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_RIGHT][HALL]), right_hall_handler, FALLING);

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

        // Set lengths
        if (!estopped && !homing) {
            // Enter homing state
            if (canId == DAVID_PITCH_CTRL_HOME_FRAME_ID) {
                homing = true;
                States[MOTOR_LEFT].target_count = 0;
                States[MOTOR_RIGHT].target_count = 0;
            }
            
            // If PITCH_CTRL_BOTH both if statements will fire.
            if ((canId == DAVID_PITCH_CTRL_LEFT_FRAME_ID) ||
                (canId == DAVID_PITCH_CTRL_BOTH_FRAME_ID)) {
                States[MOTOR_LEFT].target_count = extract_value(buf, 0, 6);
                States[MOTOR_LEFT].tolerance = extract_value(buf, 6, 2);
            }
            if ((canId == DAVID_PITCH_CTRL_RIGHT_FRAME_ID) ||
                (canId == DAVID_PITCH_CTRL_BOTH_FRAME_ID)) {
                States[MOTOR_RIGHT].target_count = extract_value(buf, 0, 6);
                States[MOTOR_RIGHT].tolerance = extract_value(buf, 6, 2);
            }

            FOREACH_MOTOR {
                if (States[m].target_count > max_count) {
                    States[m].target_count = max_count;
                }
                if (States[m].target_count < 0) {
                    States[m].target_count = 0;
                }
            }
        }

        // Print CAN message
        Serial.println("-----------------------------");
        Serial.println(canId, HEX);

    } // finish CAN message


    // Update stationary timers for left and right
    FOREACH_MOTOR {
        if (States[m].true_count == States[m].prev_count) {
            States[m].stationary_timer++;
        } else {
            States[m].stationary_timer = 0;
        }
        States[m].prev_count = States[m].true_count;
    }
    
    // Standard movement towards target counts
    if (!estopped && !homing) {
        FOREACH_MOTOR {
            States[m].tolerance = DEFAULT_TOLERANCE;

            // Set directions
            if (abs(States[m].true_count - States[m].target_count) > States[m].tolerance) {
                if (States[m].true_count < States[m].target_count) {
                    States[m].dir = EXTEND;
                } else {
                    States[m].dir = RETRACT;
                }
            } else {
                States[m].dir = STOP;
            }

            // Set to stop if stationary
            if (States[m].stationary_timer > homing_delay) {
                States[m].dir = STOP;
            }
        }
    }

    // Do homing
    if (!estopped && homing) {
        FOREACH_MOTOR {
            States[m].dir = RETRACT;
        }

        // Finish homing if stationary
        if ((States[MOTOR_LEFT].stationary_timer > homing_delay) &&
            (States[MOTOR_RIGHT].stationary_timer > homing_delay)){
            homing = false;

            FOREACH_MOTOR {
                States[m].true_count = 0;
                States[m].target_count = 0;
            }
        }
    }

    // Set each motors direction
    FOREACH_MOTOR {
        sendData(MOTOR_ADDRESS[m],ACCREG, 0x0D);
        sendData(MOTOR_ADDRESS[m],SPEEDBYTE, 0xFF);
        sendData(MOTOR_ADDRESS[m],CMDBYTE, States[m].dir);
    }
        
    // Send telemetry
    send_telemetry(DAVID_PITCH_TELEM_LEFT_FRAME_ID,
                   States[MOTOR_LEFT].true_count, States[MOTOR_LEFT].dir);
    send_telemetry(DAVID_PITCH_TELEM_RIGHT_FRAME_ID, States[MOTOR_RIGHT].true_count, States[MOTOR_RIGHT].dir);

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
    int new_time = micros();
    if ((new_time - States[MOTOR_LEFT].last_trigger) > trig_delay) {
        States[MOTOR_LEFT].last_trigger = new_time;
        switch (States[MOTOR_LEFT].dir) {
	    case RETRACT: 
            States[MOTOR_LEFT].true_count -= 1;
            break;
	    case EXTEND:
            States[MOTOR_LEFT].true_count += 1;
            break;
	    case STOP: 
            break;
        }
    }
}

void right_hall_handler(){
    int new_time = micros();
    if ((new_time - States[MOTOR_RIGHT].last_trigger) > trig_delay) {
        States[MOTOR_RIGHT].last_trigger = new_time;
        switch (States[MOTOR_RIGHT].dir) {
	    case RETRACT: 
            States[MOTOR_RIGHT].true_count -= 1;
            break;
	    case EXTEND:
            States[MOTOR_RIGHT].true_count += 1;
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

void sendData(uint8_t addr, uint8_t reg, uint8_t val){      // Function for sending data to MD03
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

byte getData(uint8_t addr, uint8_t reg){                 // function for getting data from MD03
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
  
    Wire.requestFrom(addr, (uint8_t) 1);         // Requests byte from MD03
    while(Wire.available() < 1);          // Waits for byte to become available
    byte data = Wire.read();

    return(data);
}
