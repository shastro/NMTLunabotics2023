// receive a frame from can bus

#include <Wire.h>
// #include "mcp_can.h"



void setup()
{
	pinMode(9, OUTPUT);
}

void loop()
{

	for (int i = 0; i<=255; i++){
		analogWrite(9, i);
		delay(1000);
	}
    
}

