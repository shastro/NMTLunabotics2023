// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"

// Import can message constants
#include "david.h"

enum MOTOR {
  MOTOR_LEFT = 0,
  MOTOR_RIGHT = 1,
};

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
    V2_5 = 128,
    V5 = 255
};

enum DIR {
  EXTEND,
  RETRACT,
  STOP,
};


static const int PIN_TABLE[2][3] = {
    { 0, 4, 5  },
    { 1, 6, 12 },
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// Left State
volatile uint64_t left_true_count;
uint64_t prev_left_count;

// Right state
volatile uint64_t right_true_count;
uint64_t prev_right_count;

static int count_stationary_timer;

enum DIR left_dir;
enum DIR right_dir;

bool stopped = false;
bool homing = true;

void set_control(enum MOTOR motor, enum DIR dir) {
    switch (dir) {
    case EXTEND:
        analogWrite(PIN_TABLE[motor][P1], V5);
        analogWrite(PIN_TABLE[motor][P2], V5);
        break;
    case RETRACT:
        analogWrite(PIN_TABLE[motor][P1], V0);
        analogWrite(PIN_TABLE[motor][P2], V0);
        break;
    case STOP:
        analogWrite(PIN_TABLE[motor][P1], V2_5);
        analogWrite(PIN_TABLE[motor][P2], V2_5);
        break;
    }
}

void setup()
{
  // PinModes
  pinMode(PIN_TABLE[MOTOR_LEFT][HALL], INPUT_PULLUP);
  pinMode(PIN_TABLE[MOTOR_LEFT][P1], OUTPUT);
  pinMode(PIN_TABLE[MOTOR_LEFT][P2], OUTPUT);
  pinMode(PIN_TABLE[MOTOR_RIGHT][HALL], INPUT_PULLUP);
  pinMode(PIN_TABLE[MOTOR_RIGHT][P1], OUTPUT);
  pinMode(PIN_TABLE[MOTOR_RIGHT][P2], OUTPUT);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_LEFT][HALL]), left_hall_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_TABLE[MOTOR_RIGHT][HALL]), right_hall_handler, RISING);

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
  unsigned char buf[8];
  bool left_done = true;
  bool right_done = true;

  // do homing
  if (!stopped && homing) {
      Serial.println(count_stationary_timer);
      if ((left_true_count == prev_left_count)
          && (right_true_count == prev_right_count)) {
          count_stationary_timer++;
      } else {
          count_stationary_timer = 0;
      }

      prev_left_count = left_true_count;
      prev_right_count = right_true_count;

      if (count_stationary_timer > 100) {
          homing = false;
          left_true_count = right_true_count = 0;
          set_control(MOTOR_LEFT, STOP);
          set_control(MOTOR_RIGHT, STOP);
      }
  }
  
  // check if data coming
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    // read data
    CAN.readMsgBuf(&len, buf);
    uint32_t canId = CAN.getCanId();

    if (canId == DAVID_E_START_FRAME_ID) {
        stopped = false;
        goto end_message;
    }
    
    // IF ESTOP stop other behaviors
    if (canId == DAVID_E_STOP_FRAME_ID) {
        stopped = true;
        goto end_message;
    }

    // enter homing
    if (!stopped && !homing) {    
        if (canId == DAVID_PITCH_CTRL_HOME_FRAME_ID) {
            homing = true;
            left_true_count = right_true_count = 100000000;
            prev_left_count = prev_right_count = left_true_count+1;
            set_control(MOTOR_LEFT, RETRACT);
            set_control(MOTOR_RIGHT, RETRACT);
            goto end_message;
        }
    }
    
    // standard loop
    if (!stopped && !homing) {    
        // If PITCH_CTRL_BOTH both if statements will fire.
        if (canId & DAVID_PITCH_CTRL_LEFT_FRAME_ID > 0) {
            uint64_t target_count = extract_count(buf);
            uint64_t tolerance = extract_tolerance(buf);
            left_dir = get_direction(left_true_count, target_count, tolerance);
        }
        if (canId & DAVID_PITCH_CTRL_RIGHT_FRAME_ID > 0) {
            uint64_t target_count = extract_count(buf);
            uint64_t tolerance = extract_tolerance(buf);
            right_dir = get_direction(right_true_count, target_count, tolerance);
        }

        // Set controls
        if (!(left_dir == STOP)){
            left_done = false;
        }
        set_control(MOTOR_LEFT,left_dir);

        if (!(right_dir == STOP)){
            right_done = false;
        }
        set_control(MOTOR_RIGHT,right_dir);
    }

    end_message: 
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

  } // finish CAN message
  
  // Send telemetry
  send_telemetry(DAVID_PITCH_TELEM_LEFT_FRAME_ID, left_true_count, left_done);
  send_telemetry(DAVID_PITCH_TELEM_RIGHT_FRAME_ID, right_true_count, right_done);
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
  switch (left_dir) {
    case RETRACT: 
      if (left_true_count == 0) {
        // Should never happen ideally
        // just going to clamp at zero
        return;
      }
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
  switch (right_dir) {
    case RETRACT: 
      if (right_true_count == 0) {
        return;
      }
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
