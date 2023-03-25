// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"

// Import CAN message constants
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
    V2_5 = 127,
    V5 = 255
};

enum DIR {
  EXTEND,
  RETRACT,
  STOP,
};

#define P1_L 4
#define P2_L 5
#define P1_R 6
#define P2_R 12
#define HALL_L 0
#define HALL_R 3

static const int64_t DEFAULT_TRIG_DELAY = 10000; // Microsecond delay
static const int64_t DEFAULT_MAX_COUNT = 875-10;
static const int64_t DEFAULT_HOMING_DELAY = 10000;
static const int64_t DEFAULT_TOLERANCE = 1000;
// static const float mm_per_count = 0.17896; 
// static const int64_t MAX_COUNT = (int64_t)((152./mm_per_count) - 30);

static const int PIN_TABLE[2][3] = {
    { HALL_L, P1_L, P2_L  },
    { HALL_R, P1_R, P2_R },
};

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// Left State
static volatile int64_t left_true_count = 0;
static int64_t left_prev_count = 0;
static int64_t left_target_count = 0;
static int64_t left_tolerance = DEFAULT_TOLERANCE;
static int64_t left_last_trigger = 0;

// Right state
static volatile int64_t right_true_count = 0;
static int64_t right_prev_count = 0;
static int64_t right_target_count = 0;
static int64_t right_tolerance = DEFAULT_TOLERANCE;
static int64_t right_last_trigger = 0;

static int64_t max_count = DEFAULT_MAX_COUNT;
static int64_t trig_delay = DEFAULT_TRIG_DELAY;
static int64_t homing_delay = DEFAULT_HOMING_DELAY;

static int count_stationary_timer = 0;

enum DIR left_dir = STOP;
enum DIR right_dir = STOP;

bool estopped = false;
bool homing = true;

void set_control(enum MOTOR motor, enum DIR dir) {
    switch (dir) {
    case EXTEND:
        analogWrite(PIN_TABLE[motor][P1], V0);
        analogWrite(PIN_TABLE[motor][P2], V5);
        break;
    case RETRACT:
        analogWrite(PIN_TABLE[motor][P1], V0);
        analogWrite(PIN_TABLE[motor][P2], V0);
        break;
    case STOP:
        analogWrite(PIN_TABLE[motor][P1], V0);
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
            left_target_count = 0;
            right_target_count = 0;
            left_prev_count = left_true_count+1;
            right_prev_count = right_true_count+1;
        }
    }
    
    // Set lengths
    if (!estopped && !homing) {    
        // If PITCH_CTRL_BOTH both if statements will fire.
        if (canId == DAVID_PITCH_CTRL_BOTH_FRAME_ID) {
            left_target_count = extract_value(buf, 0, 6);
            right_target_count = left_target_count;
            left_tolerance = extract_value(buf, 6, 2);
            right_tolerance = left_tolerance;
        }
        if (canId == DAVID_PITCH_CTRL_LEFT_FRAME_ID) {
            left_target_count = extract_value(buf, 0, 6);
            left_tolerance = extract_value(buf, 6, 2);
        }
        if (canId == DAVID_PITCH_CTRL_RIGHT_FRAME_ID) {
            right_target_count = extract_value(buf, 0, 6);
            right_tolerance = extract_value(buf, 6, 2);
        }
        if (left_target_count > MAX_COUNT) {
          left_target_count = MAX_COUNT;
        }
        if (right_target_count > MAX_COUNT) {
          right_target_count = MAX_COUNT;
        }
    }

    // Print CAN message
    Serial.println("-----------------------------");
    Serial.println(canId, HEX);

  } // finish CAN message

  // Do homing
  if (!estopped && homing) {
      left_dir = RETRACT;
      right_dir = RETRACT;
      if ((left_true_count == left_prev_count)
          && (right_true_count == right_prev_count)) {
          count_stationary_timer++;
      } else {
          count_stationary_timer = 0;
      }

      left_prev_count = left_true_count;
      right_prev_count = right_true_count;

      if (count_stationary_timer > 10000) {
          homing = false;
          left_true_count = 0; 
          right_true_count = 0;
          left_target_count = 0;
          right_target_count = 0;
      }
 }
  
  bool left_done = false;
  bool right_done = false;
  
  // Standard movement towards target counts
  if (!estopped && !homing) {
      left_dir = get_direction(left_true_count, left_target_count, left_tolerance);
      right_dir = get_direction(right_true_count, right_target_count, right_tolerance);
      // Set controls

      if (left_dir == STOP) {
          left_done = true;
      }
      if (right_dir == STOP) {
          right_done = true;
      }
  }
  
  set_control(MOTOR_LEFT, left_dir);
  set_control(MOTOR_RIGHT, right_dir);

  // Send telemetry
  send_telemetry(DAVID_PITCH_TELEM_LEFT_FRAME_ID, left_true_count, left_done);
  send_telemetry(DAVID_PITCH_TELEM_RIGHT_FRAME_ID, right_true_count, right_done);

}

void send_telemetry(uint32_t canId, int64_t true_count, bool done){
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

enum DIR get_direction(int64_t true_count, int64_t target_count, int64_t tolerance) {
  enum DIR retdir = STOP;
  if (abs((int64_t)true_count - (int64_t)target_count) > tolerance) {
    if (true_count > target_count) {
      retdir = RETRACT;  
    } else {
      retdir = EXTEND;  
    }
  }
  
  return retdir;
}

void left_hall_handler(){
  if ((micros() - left_last_trigger) > trig_delay) {
	  left_last_trigger = micros();
	  switch (left_dir) {
	    case RETRACT: 
	      left_true_count -= 1;
	      break;
	    case EXTEND: 
	      left_true_count += 1;
	      break;
	    case STOP: 
	      break;
	  }
  } 

}
void right_hall_handler(){
  if ((micros() - right_last_trigger) > trig_delay) {
	  right_last_trigger = micros();
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
