#include <Wire.h>

#include "mcp_can.h"
#include "david.h"

const int stepsPerTick = 600;  // change this to fit the number of steps per revolution
const int stepDelayMicros = 10;

enum PINS {
    PUL1 = 10,
    DIR1 = 6, 
    
    PUL2 = 4,
    DIR2 = 11,
    
    SPI_CS_PIN = 17
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

bool estopped = false;

enum DIR {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
};
int move_dir = STOP;

static int step = 0;

void setup() {
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
        {
        }

    pinMode(PUL1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PUL2, OUTPUT);
    pinMode(DIR2, OUTPUT);
}

void loop() {

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

        if (canId == DAVID_E_START_FRAME_ID) {
            estopped = false;
        }

        if ((canId == DAVID_STEPPER_CTRL_LEFT_FRAME_ID) ||
            (canId == DAVID_STEPPER_CTRL_RIGHT_FRAME_ID) ||
            (canId == DAVID_STEPPER_CTRL_BOTH_FRAME_ID))
            {
                move_dir = extract_value(buf, 0, 1);
        }
    }

    if (!estopped && !(move_dir == STOP)) {
        for (int i = 0; i < stepsPerTick; i++) {
            stepMotors(step);
            if (move_dir == FORWARD) {
                step++;
            } else if (move_dir == BACKWARD) {
                step--;
            }
            delayMicroseconds(stepDelayMicros);
        }
    }
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


int64_t extract_value(char buf[], int first_byte, int bytes) {
    int64_t count = 0;
    int64_t temp = 0;
    for (int i = first_byte; i< first_byte+bytes; i++) {
        temp = (int64_t)buf[i];
        count += temp << (8*i);
    }
    return count;  
}

void stepMotors(int step)
{
    switch (step % 4) {
    case 0:  // 01
        digitalWrite(DIR1, LOW);
        digitalWrite(PUL1, HIGH);
        digitalWrite(DIR2, LOW);
        digitalWrite(PUL2, HIGH);
        break;
    case 1:  // 11
        digitalWrite(DIR1, HIGH);
        digitalWrite(PUL1, HIGH);
        digitalWrite(DIR2, HIGH);
        digitalWrite(PUL2, HIGH);
        break;
    case 2:  // 10
        digitalWrite(DIR1, HIGH);
        digitalWrite(PUL1, LOW);
        digitalWrite(DIR2, HIGH);
        digitalWrite(PUL2, LOW);
        break;
    case 3:  // 00
        digitalWrite(DIR1, LOW);
        digitalWrite(PUL1, LOW);
        digitalWrite(DIR2, LOW);
        digitalWrite(PUL2, LOW);
        break;
    }
}

