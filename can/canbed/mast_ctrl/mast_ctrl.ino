#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#include "arduino_lib.hpp"
#include "david.h"

// #define pul 5
// #define dir 4

struct Stepper {
  OutPin pulse;
  OutPin direction;

  Stepper(int pul, int dir) :
  pulse(pul), direction(dir) {}

  void doStep(unsigned int step) {
      const int pulse_sequence[] = {HIGH, HIGH, LOW, LOW};
      const int dir_sequence[] = {LOW, HIGH, HIGH, LOW};
      pulse.write(pulse_sequence[step % 4]);
      direction.write(dir_sequence[step % 4]);
  }
};

struct StepperController {
  enum Dirs {
    COUNTER = -1,
    STOP = 0,
    CLOCk = 1
  };
  enum Dirs dir;

  Stepper cam;

  unsigned int step;

  StepperController(int pulse_c, int dir_c) :
  cam(pulse_c, dir_c) {
    dir = STOP;
    step = 0;
  }
  void doStep(enum Dirs dir) {
    if (dir != STOP) {
      step += dir;
      cam.doStep(step);
    }
  }


  void loop() {
    doStep(dir);
    delayMicroseconds(50);
  }
};

double Mic_delay = 1800;
double Mic_cur = 0;
double Mic_prev = 0;



void setup() {
    MCP_CAN can = setup_can();
    
    #define PUL 5 // For camera mast
    #define DIR 4
    StepperController control(PUL, DIR);

    bool eStopped = false;
    for (;;) {
  
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                eStopped = frame.stop;
            }
        }

        if (eStopped)
            continue;
        
        switch (packet.id) {
            FRAME_CASE(DAVID_MAST_CTRL, david_mast_ctrl) {
              control.dir=(frame.direction);
              Serial.print("Im tired");
            }
        }
        control.loop();
    }
}


