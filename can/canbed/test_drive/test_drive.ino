// receive a frame from can bus

#include <SPI.h>
#include <Wire.h>
#include "mcp_can.h"

// Import CAN message constants
#include "david.h"
#define ADDRESS             0xB0                    // Address of MD03
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current


void setup()
{
    Wire.begin();
    delay(100);
    
    // PinModes
    /* pinMode(P1_L, OUTPUT); */
    /* pinMode(P2_L, OUTPUT); */
    /* pinMode(P1_R, OUTPUT); */
    /* pinMode(P1_R, OUTPUT); */
}

void loop()
{
    sendData(SPEEDBYTE, 0xff);             // Sets speed to i
    sendData(CMDBYTE, 0);          // Sets motor to direct, a value of 1 runs the motor forward and 2 runs backward
    delay(100);  
    
}

byte getData(byte reg){                 // function for getting data from MD03
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(ADDRESS, 1);         // Requests byte from MD03
  while(Wire.available() < 1);          // Waits for byte to become available
  byte data = Wire.read();

  return(data);
}

void sendData(byte reg, byte val){      // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS);      // Send data to MD03
    Wire.write(reg);                    // Command like Direction, Speed
    Wire.write(val);                    // Value for the command
  Wire.endTransmission();
}