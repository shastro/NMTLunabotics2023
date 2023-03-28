#include <Stepper.h>
#include <Wire.h>

#include "mcp_can.h"
#include "david.h"

const int stepsPerRevolution = 12800;  // change this to fit the number of steps per revolution
const int RPM = 1;
const int stepsPerTick = 10;

enum PINS {
    PUL1 = 10,
    DIR1 = 6, 
    
    PUL2 = 4,
    DIR2 = 11,
    
    SPI_CS_PIN = 17
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

Stepper stepper1(stepsPerRevolution, DIR1, PUL1);
Stepper stepper2(stepsPerRevolution, DIR2, PUL2);

bool estopped = false;

enum DIR {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
};
int move_dir = FORWARD;

void setup() {
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
        {
        }

    // Declare pins as output:
  /* pinMode(PUL1, OUTPUT); */
  /* pinMode(DIR1, OUTPUT); */
  /* pinMode(PUL2, OUTPUT); */
  /* pinMode(DIR2, OUTPUT); */

  // Set the spinning direction CW/CCW:
  /* digitalWrite(DIR1, HIGH); */
  /* digitalWrite(DIR2, HIGH); */
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

        if ((canId == DAVID_STEPPER_CTRL_LEFT_FRAME_ID) ||
            (canId == DAVID_STEPPER_CTRL_RIGHT_FRAME_ID) ||
            (canId == DAVID_STEPPER_CTRL_BOTH_FRAME_ID))
            {
                move_dir = extract_value(buf, 0, 1);
        }
    }

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

    /* /\* for (int i = 0; i < 100; i++) { *\/ */
    /* digitalWrite(PUL1, HIGH); */
    /* digitalWrite(PUL2, HIGH); */
    /* digitalWrite(DIR1, LOW); */
    /* digitalWrite(DIR2, LOW); */
    stepper1.step(-stepsPerRevolution);
    stepper2.step(-stepsPerRevolution);

    delay(1000);
    /* /\* } *\/ */


    
    /* /\* for (int i = 0; i < 100; i++) { *\/ */
    /* digitalWrite(PUL1, HIGH); */
    /* digitalWrite(PUL2, HIGH); */
        /* digitalWrite(DIR1, HIGH); */
        /* digitalWrite(DIR2, HIGH); */
    stepper1.step(+stepsPerRevolution);
    stepper2.step(+stepsPerRevolution);
    delay(1000);

  /*   digitalWrite(DIR1, HIGH); // Enables the motor to move in a particular direction */
  /*       digitalWrite(DIR2, HIGH); // Enables the motor to move in a particular direction */
  /* for (int x = 0; x < 100; x++) */
  /* { */
  /*   digitalWrite(PUL1, HIGH); */
  /*   digitalWrite(PUL2, HIGH); */
  /*   delay(50); */
  /*   digitalWrite(PUL1, LOW); */
  /*   digitalWrite(PUL2, LOW); */
  /*   delay(50); */
  /* } */
  /* delay(1000); // One second delay */
  
  /* digitalWrite(DIR1, LOW); // Enables the motor to move in a particular direction */
  /* digitalWrite(DIR2, LOW); // Enables the motor to move in a particular direction */
  /* for (int x = 0; x < 100; x++) */
  /* { */
  /*       digitalWrite(PUL1, HIGH); */
  /*   digitalWrite(PUL2, HIGH); */
  /*   delay(50); */
  /*   digitalWrite(PUL1, LOW); */
  /*   digitalWrite(PUL2, LOW); */
  /*       delay(50); */
  /* } */
  /* delay(1000); */


    /* digitalWrite(DIR1, HIGH); */
    /* delayMicroseconds(500); */
    /* digitalWrite(DIR2, HIGH); */
    /* delayMicroseconds(500); */

    
    /* for (int i = 0; i < 5*stepsPerRevolution; i++) { */
    /*     digitalWrite(PUL1, HIGH); */
    /*     delayMicroseconds(500); */
    /*     digitalWrite(PUL1, LOW); */
    /*     delayMicroseconds(500); */

    /*     digitalWrite(PUL2, HIGH); */
    /*     delayMicroseconds(500); */
    /*     digitalWrite(PUL2, LOW); */
    /*     delayMicroseconds(500); */
    /* } */

    /* digitalWrite(DIR1, LOW); */
    /* delayMicroseconds(500); */
    /* digitalWrite(DIR2, LOW); */
    /* delayMicroseconds(500); */

    
    /* for (int i = 0; i < 5*stepsPerRevolution; i++) { */
    /*     digitalWrite(PUL1, HIGH); */
    /*     delayMicroseconds(500); */
    /*     digitalWrite(PUL1, LOW); */
    /*     delayMicroseconds(500); */

    /*     digitalWrite(PUL2, HIGH); */
    /*     delayMicroseconds(500); */
    /*     digitalWrite(PUL2, LOW); */
    /*     delayMicroseconds(500); */
    /* } */

    
    /* /\* } *\/ */

    /* delay(1000); */
    send_telemetry(DAVID_PITCH_TELEM_LEFT_FRAME_ID, 0x255, 0x255);    
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
