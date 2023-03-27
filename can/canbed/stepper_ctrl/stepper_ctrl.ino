#include <Stepper.h>
#include <Wire.h>

const int stepsPerRevolution = 700;  // change this to fit the number of steps per revolution
const int RPM = 1;
const int stepsPerTick = 10;


// initialize the stepper library on pins 8 through 11:
#define PUL1 4
#define DIR1 11
#define PUL2 10
#define DIR2 5
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
            Serial.println("CAN BUS FAIL!");
        }

    stepper1.setSpeed(1);
    stepper2.setSpeed(1);
}

void loop() {

  // Check for new messages
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
      // read data
      CAN.readMsgBuf(&len, buf);
      uint32_t canId = CAN.getCanId();

      // IF ESTOP stop other behaviors
      if (canId == DAVID_E_STOP_FRAME_ID) {
          estopped = true;
      }

      if (canId == DAVID_STEPPER_CTRL_FRAME_ID) {
          move_dir = extract_value(buf, 0, 1);              
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
