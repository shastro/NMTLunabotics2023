// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"

// Import CAN message constants
#include "david.h"

#define P1_L 4
#define P2_L 5
#define P1_R 6
#define P2_R 12

void setup()
{

  // PinModes
  pinMode(P1_L, OUTPUT);
  pinMode(P2_L, OUTPUT);
  pinMode(P1_R, OUTPUT);
  pinMode(P1_R, OUTPUT);

  
}

void loop()
{
	digitalWrite(P1_L, HIGH);
	digitalWrite(P2_L, HIGH);

	digitalWrite(P1_L, HIGH);
	digitalWrite(P2_L, HIGH);
}

