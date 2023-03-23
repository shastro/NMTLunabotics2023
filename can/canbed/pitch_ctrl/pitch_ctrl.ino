// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 17;              // CANBed V1

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

// Define pins
#define P1_L  4
#define P2_L  5
#define P1_R  6
#define P2_R  12
#define HALL_L  0
#define HALL_R  1

// P1 is direction
// P2 is power

#define ESTOP  0x0
#define PITCH_CTRL_LEFT  0x100
#define PITCH_TELEM_LEFT  0x101
#define PITCH_CTRL_RIGHT  0x102
#define PITCH_TELEM_RIGHT  0x103

// Left State
uint64_t left_true_count;

// Right state
uint64_t right_true_count;


enum DIR {
  EXTEND,
  RETRACT,
  STOP,
};

enum MOTOR {
  MOTOR_LEFT,
  MOTOR_RIGHT,
};

enum DIR left_dir;
enum DIR right_dir;

bool has_homed = false;

void setup()
{
  // PinModes
  pinMode(P1_L, OUTPUT);
  pinMode(P2_L, OUTPUT);
  pinMode(P1_R, OUTPUT);
  pinMode(P2_R, OUTPUT);
  pinMode(HALL_L, INPUT_PULLUP);
  pinMode(HALL_R, INPUT_PULLUP);

  // Interrupts

  attachInterrupt(digitalPinToInterrupt(HALL_L), left_hall_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_R), right_hall_handler, RISING);

  Serial.begin(115200);
  // while(!Serial);
  while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
      Serial.println("CAN BUS FAIL!");
      delay(100);
    }
  Serial.println("CAN BUS OK!");
}

void home(){
  // Left State
  left_true_count = 100000000;
  // Right state
  right_true_count = 100000000;
  // Set to retract
  set_control(MOTOR_LEFT,RETRACT);
  set_control(MOTOR_RIGHT,RETRACT);
  // Homing loop
  uint64_t p_left_count = left_true_count + 1;
  uint64_t p_right_count = right_true_count + 1;

  // Done homing
  bool done = false;
  while (!done) {
    delay(100);
    p_left_count = left_true_count;
    p_right_count = right_true_count;
    // During the delay we expect constantly changing pulses due to the interrupt
    // Once a motor is set it will keep retracting
    // So we just delay and expect interrupts to keep coming updating the value
    // of right_true_count until it does not match p_right_count
    delay(100);

    // Have stayed in the same position for 100 ms
    if (p_left_count == left_true_count) {
     if (p_right_count == right_true_count) {
        done = true;
      } 
    }
  }

  // Set true_count to zero
  left_true_count = 0;
  right_true_count = 0;

  // Set to STOP after home
  set_control(MOTOR_LEFT,STOP);
  set_control(MOTOR_RIGHT,STOP);
    
}

void loop()
{

  if (!has_homed) {
    home();
    has_homed = true;
  }
  
  unsigned char len = 0;
  unsigned char buf[8];

  // check if data coming
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    // read data
    CAN.readMsgBuf(&len, buf);

    uint32_t canId = CAN.getCanId();

    // IF ESTOP freeze forever
    if (canId == ESTOP) {
      while(true){}
    }

    if (canId == PITCH_CTRL_LEFT) {
      uint64_t target_count = extract_count(buf);
      uint64_t tolerance = extract_tolerance(buf);
      left_dir = get_direction(left_true_count, target_count, tolerance);
    }

    if (canId == PITCH_CTRL_RIGHT) {
      uint64_t target_count = extract_count(buf);
      uint64_t tolerance = extract_tolerance(buf);
      right_dir = get_direction(right_true_count, target_count, tolerance);
    }

    // Printing
    Serial.println("-----------------------------");
    Serial.print("Get data from ID: ");
    Serial.println(canId, HEX);

    // print the data
    for(int i = 0; i<len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print("\t");
    }
    Serial.println();

    // Send back the data, mirrored and flipped.
    for (int i = 0; i < len; i++)
      buf[i] = ~buf[i];
    CAN.sendMsgBuf(0x00, 0, 8, buf);
  } // finish CAN message

  // Now set the controls

  bool left_done = true;
  if (!(left_dir == STOP)){
    left_done = false;
  }
  set_control(MOTOR_LEFT,left_dir);

  bool right_done = true;
  if (!(right_dir == STOP)){
    right_done = false;
  }
  set_control(MOTOR_RIGHT,right_dir);

  // Send telemetry
  send_telemetry(PITCH_TELEM_LEFT, left_true_count, left_done);
  send_telemetry(PITCH_TELEM_RIGHT, right_true_count, right_done);
}

void send_telemetry(uint32_t canId, uint64_t true_count, bool done){

  unsigned char buf[8] = {0};
  buf[0] = true_count >> 8*0;
  buf[1] = true_count >> 8*1;
  buf[2] = true_count >> 8*2;
  buf[3] = true_count >> 8*3;
  buf[4] = true_count >> 8*4;
  buf[5] = true_count >> 8*5;

  if (done) {
    buf[6] = 0xff;
  } else {
    buf[6] = 0x0;
  }

  CAN.sendMsgBuf(canId, 0, 8, buf);
}

void set_control(enum MOTOR motor, enum DIR dir) {
    switch (dir) {
        case EXTEND:
            if (motor == MOTOR_LEFT) {
              digitalWrite(P1_L, HIGH);
              digitalWrite(P2_L, HIGH);
            } else {
              digitalWrite(P1_R, HIGH);
              digitalWrite(P2_R, HIGH);
            }
            break;
        case RETRACT:
            if (motor == MOTOR_LEFT) {
              digitalWrite(P1_L, LOW);
              digitalWrite(P2_L, HIGH);
            } else {
              digitalWrite(P1_R, LOW);
              digitalWrite(P2_R, HIGH);
            }
            break;
        case STOP:
            if (motor == MOTOR_LEFT) {
              digitalWrite(P1_L, LOW);
              digitalWrite(P2_L, LOW);
            } else {
              digitalWrite(P1_R, LOW);
              digitalWrite(P2_R, LOW);
            }
            break;
    }
}


enum DIR get_direction(uint64_t true_count, uint64_t target_count, uint64_t tolerance) {
  enum DIR retdir;
  if (abs(true_count - target_count) > tolerance) {
    if (true_count > target_count) {
      retdir = RETRACT;  
    } else {
      retdir = EXTEND;  
    }
  } else {
    retdir = STOP;
  }

  return retdir;
}

void left_hall_handler(){
  if (left_true_count == 0) {
    // Should never happen ideally
    // just going to clamp at zero
    return;
  }
  switch (left_dir) {
    case RETRACT: 
      left_true_count -= 1;
      break;
    case EXTEND: 
      left_true_count += 1;
      break;
    case STOP: 
      // Should never happen
      break;
  }
}
void right_hall_handler(){
  if (right_true_count == 0) {
    return;
  }
  switch (right_dir) {
    case RETRACT: 
      right_true_count -= 1;
      break;
    case EXTEND: 
      right_true_count += 1;
      break;
    case STOP: 
      break;
  }
}

uint64_t extract_count(char buf[]) {
  uint64_t count = 0;
  for (int i=0; i<6; i++) {
    count += buf[i] << 8*i;
  }
  return count;
}

uint64_t extract_tolerance(char buf[]) {
  uint64_t tolerance = 0;
  for (int i=6; i<8; i++) {
    tolerance += buf[i] << 8*i;
  }
  return tolerance;
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
