// receive a frame from can bus

#include <SPI.h>
#include "mcp_can.h"
#include <Stepper.h>

const int SPI_CS_PIN = 17;              // CANBed V1

MCP_CAN CAN(SPI_CS_PIN);                // Set CS pin


const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 8;
                 
const int STEPS = 200;

const Stepper stepper(STEPS, IN1,IN2,IN3,IN4);

void setup()
{
    Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

    // pinMode(IN1, OUTPUT);
    // pinMode(IN2, OUTPUT);
    // pinMode(IN3, OUTPUT);
    // pinMode(IN4, OUTPUT);
    stepper.setSpeed(10);
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
        }
    }
    unsigned char sin_data = buf[0];
    stepper.setSpeed(sin_data);
    stepper.step(200);
    delay(100);
    // TODO Write correct control signals on the pins we identified above
    // First just make it spin ignoring all CAN stuff
    // Then try using the (sin_data) value to do something
    // such as set RPM
    // analogWrite(LED_PIN, buf[0]);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
