// receive a frame from can bus

#include <Wire.h>
// #include "mcp_can.h"

// Import CAN message constants
// #include "david.h"
#define ADDRESS             0x58                    // Address of MD03
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current
#define STATUSREG           0x01


void setup()
{
    // while(Serial.available() == 0);
    // Serial.read();
    // Serial.println("fuck");

    Wire.begin();
    delay(100);

    Serial.print("Wire:");
    Serial.begin(19200);                              // Start Serial connection
    delay(100);    

}

void loop()
{
    // Serial.println("Bruh");
    sendData(0x3, 0x3);
    sendData(SPEEDBYTE, 255);             // Sets speed to i
    sendData(CMDBYTE, 1);          // Sets motor to direct, a value of 1 runs the motor forward and 2 runs backward
    // showData();
    delay(100);  
    // Serial.println("Bruh2");
    
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
  int error = Wire.endTransmission();
    Serial.println(error);
}

void showData() {
   
    byte status= getData(STATUSREG); // Get current value (186 = 20 Amp) to Serial
    delay(10);
    // Serial.print(" - Status: ");       
    // Serial.print(status);              
    
    byte current = getData(CURRENTREG); // Get current value (186 = 20 Amp) to Serial
    delay(10);
    // Serial.print(" - Current: ");       
    // Serial.print(current);              
    
    byte temp = getData(TEMPREG);       // Get surface temperature of the PCB in degrees centigrade.
    delay(10);
    // Serial.print(" - Temperature: ");
    // Serial.print(temp);                
    
    // Serial.println(".");                // Point at the end of the line and an Enter
}