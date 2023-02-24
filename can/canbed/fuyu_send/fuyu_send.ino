// send a frame from can bus

#include <SPI.h>
#include "mcp_can.h"

#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 17;              // CANBed V1

MCP_CAN CAN(SPI_CS_PIN);                // Set CS pin

void setup()
{
    Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");
}

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float t = 0;
void loop()
{
    // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf

    int id = 0x10;
    
    unsigned char sval = char(255*(0.5*(sin(t)+1)));
    stmp[0] = sval;

    Serial.print("Sending data on ");
    Serial.println(id, HEX);

    for(int i = 0; i<8; i++)    // print the data
    {
        Serial.print(stmp[i], HEX);
        Serial.print("\t");
    }
    Serial.println();

    CAN.sendMsgBuf(id, 0, 8, stmp);
    delay(100);                       // send data per 100ms
    t += 0.005;
}

// END FILE
