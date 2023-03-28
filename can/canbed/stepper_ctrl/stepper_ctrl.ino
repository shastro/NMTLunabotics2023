#include <Stepper.h>
#include <Wire.h>

#include "mcp_can.h"
#include "david.h"

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
const int DEFAULT_RPM = 1;
const int MAX_RPX = 128;
const int stepsPerTick = 10;

enum PINS {
    PUL1 = 4,
    DIR1 = 10,

    PUL2 = 5,
    DIR2 = 9,

    SPI_CS_PIN = 17
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

Stepper stepper1(stepsPerRevolution, PUL1, DIR1);
Stepper stepper2(stepsPerRevolution, PUL2, DIR2);

bool estopped = false;

enum DIR {
    BACKWARD = -1,
    STOP = 0,
    FORWARD = 1,
};
int move_dir = STOP;
int rpm = DEFAULT_RPM;

void setup() {
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("WHEELS ON BUS NOT GO ROUND");
        delay(100);
    }
    Serial.println("THE WHEELS ON THE BUS GO ROUND AND ROUND");
}

void loop() {
    stepper1.setSpeed(rpm);
    stepper1.setSpeed(rpm);

    // Check for new messages 
    if(CAN_MSGAVAIL == CAN.checkReceive()) { 
        unsigned char len = 0; 
        unsigned char buf[8] = {0}; 
        // read data 
        CAN.readMsgBuf(&len, buf); 
        uint32_t canId = CAN.getCanId(); 

        // IF ESTOP stop other behaviors 
        if (canId == DAVID_E_STOP_FRAME_ID) { 
            estopped = true; 
        } 

        if (canId == DAVID_STEPPER_CTRL_FRAME_ID) { 
            rpm = map(extract_value(0, 4), 0, 1024, 0, MAX_RPM);
            move_dir = extract_value(buf, 4, 4);
        } 
    } 

    // step one step: 
    if (!estopped) { 
        if (move_dir == FORWARD) { 
            stepper1.step(-stepsPerTick); 
            stepper2.step(stepsPerTick); 
        } else if (move_dir == BACKWARD) { 
            stepper1.step(stepsPerTick); 
            stepper2.step(-stepsPerTick); 
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
