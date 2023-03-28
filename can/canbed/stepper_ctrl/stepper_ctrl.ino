#include <Stepper.h>
#include <Wire.h>

#include "mcp_can.h"
#include "david.h"

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
const int RPM = 1;
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
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
};
int move_dir = STOP;

void setup() {
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
        {
            /* Serial.println("CAN BUS FAIL!"); */
        }

    /* stepper1.setSpeed(1); */
    /* stepper2.setSpeed(1); */
}

void loop() {

    /* // Check for new messages */
    /* if(CAN_MSGAVAIL == CAN.checkReceive()) { */
    /*     unsigned char len = 0; */
    /*     unsigned char buf[8] = {0}; */
    /*     // read data */
    /*     CAN.readMsgBuf(&len, buf); */
    /*     uint32_t canId = CAN.getCanId(); */

    /*     // IF ESTOP stop other behaviors */
    /*     if (canId == DAVID_E_STOP_FRAME_ID) { */
    /*         estopped = true; */
    /*     } */

    /*     if (canId == DAVID_STEPPER_CTRL_FRAME_ID) { */
    /*         move_dir = extract_value(buf, 0, 1);               */
    /*     } */
    /* } */

    /* // step one step: */
    /* if (!estopped) { */
    /*     if (move_dir == FORWARD) { */
    /*         stepper1.step(-stepsPerTick); */
    /*         stepper2.step(stepsPerTick); */
    /*     } else if (move_dir == BACKWARD) { */
    /*         stepper1.step(stepsPerTick); */
    /*         stepper2.step(-stepsPerTick); */
    /*     } */
    /* } */

    /* for (int i = 0; i < 100; i++) { */
    stepper1.step(stepsPerTick);
    stepper2.step(-stepsPerTick);
    /* } */

    /* delay(1000); */

    
    /* for (int i = 0; i < 100; i++) { */
    /*     stepper1.step(-stepsPerTick); */
    /*     stepper2.step(-stepsPerTick); */
    /* } */

    /* delay(1000); */
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
