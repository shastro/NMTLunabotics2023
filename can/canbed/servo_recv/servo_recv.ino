// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"
#include <Servo.h>

const int SPI_CS_PIN = 17;              // CANBed V1

MCP_CAN CAN(SPI_CS_PIN);                // Set CS pin

Servo my_servo;

const int SERVO_PIN = 6;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

    my_servo.attach(SERVO_PIN, 900, 2100);
}

unsigned char len = 0;
unsigned char buf[8];

void loop()
{

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned long canId = CAN.getCanId();

        if (canId == 0x10) {
            
            Serial.println("-----------------------------");
            Serial.print("Get data from ID: ");
            Serial.println(canId, HEX);

            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.print(buf[i], HEX);
                Serial.print("\t");
            }
            Serial.println();
            my_servo.write(buf[0]);
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
