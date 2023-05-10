// receive a frame from can bus
#include "arduino_lib.hpp"
#include "Longan_I2C_CAN_Arduino.h"

// Import CAN message constants
#include "david.h"
// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)

// Pins
#define HALL_PIN_L 18
#define HALL_PIN_R 19

// I2C Registers
#define SOFTREG             0x07                    // Byte to read software
#define CMDREG              0x00                    // Command byte
#define SPEEDREG            0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current
#define STATUSREG           0x01
#define ACCREG              0x03

static const int64_t DEFAULT_TRIG_DELAY = 10000; // Microsecond delay
static const int64_t DEFAULT_MAX_COUNT = 875-20;
static const int64_t DEFAULT_HOMING_DELAY = 300;
static const int64_t DEFAULT_TOLERANCE = 3;


#define I2C_CAN_ADDR 0x25

enum Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};

#define ACC 0x0A
#define SPEED 150

struct MD04Driver {
    int addr;  

    MD04Driver(unsigned addr_in) {
        addr = addr_in;
    }

    byte getTemperature() {
        return getData(TEMPREG);
    }

    byte getCurrent() {
        return getData(CURRENTREG);
    }

    void setDirection(Dir dir){
        sendData(ACCREG, ACC);
        sendData(SPEEDREG, SPEED);
        sendData(CMDREG, dir);
    }

    byte getData(byte reg){                 // function for getting data from MD03
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom(addr, 1);         // Requests byte from MD03
        while(Wire.available() < 1);          // Waits for byte to become available
        byte data = Wire.read();

        return(data);
    }

    void sendData(byte reg, byte val){      // Function for sending data to MD03
        Wire.beginTransmission(addr);      // Send data to MD03
        Wire.write(reg);                    // Command like Direction, Speed
        Wire.write(val);                    // Value for the command
        int error = Wire.endTransmission();
        if(error) {
            Serial.print("I2C ERROR:");
            Serial.println(error);
        }
        delay(10);
    }

};


struct PitchController {

    MD04Driver left_m;
    MD04Driver right_m;
    double tolerance;

    double left_pos;
    double right_pos;

    int left_count;
    int right_count;

    double set_point;
    double left_offset;
    double right_offset;

    enum States {
        Move = 0,
        Home = 1,
    };

    PitchController(double tolerance_) : left_m(0x59), right_m(0x58) {
        tolerance = tolerance_;
        left_pos = 0.0;
        right_pos = 0.0;

        left_count = 0;
        right_count = 0;

        left_offset = 0.0;
        right_offset = 0.0;

    }

    void setPoint(double set_point_) {
        set_point = set_point_;
    }

    void setLeftOffset(double left_offset_){
        left_offset = left_offset_;
    
    }

    void setRightOffset(double right_offset_){
        right_offset = right_offset_;
    }

    void pack_telemetry(unsigned char buf[8]){
        
        david_pitch_driver_telem_t data = {0};
        byte left_current = left_m.getCurrent();
        byte right_current = right_m.getCurrent();
        byte left_temperature = left_m.getTemperature();
        byte right_temperature = right_m.getTemperature();
        // Temperature
        data.left_temperature = david_pitch_driver_telem_left_temperature_encode((double)left_temperature);
        data.right_temperature = david_pitch_driver_telem_right_temperature_encode((double)right_temperature);

        double conversion_factor = 20.0/186.0;
        // Current
        data.left_current = david_pitch_driver_telem_left_current_encode((double)left_current*conversion_factor);
        data.right_current = david_pitch_driver_telem_right_current_encode((double)right_current*conversion_factor);
        david_pitch_driver_telem_pack(buf, &data, 8);
    }
    void loop(){
        int ticks = 1000;
        const int read_limit_frequency = 200;
        while(ticks > 0){
            if (ticks-- % read_limit_frequency == 0) {
                
            }
            
            // Left
            if (left_pos > set_point) {
                left_m.setDirection(Dir::Retract);
            }
            if (left_pos < set_point) {
                left_m.setDirection(Dir::Extend);
            }
            // Right
            if (right_pos > set_point) {
                right_m.setDirection(Dir::Retract);
            }
            if (right_pos < set_point) {
                left_m.setDirection(Dir::Extend);
            }
        }
        delay(50);
    }
};

inline I2C_CAN setup_can_i2c() {
    I2C_CAN can(I2C_CAN_ADDR);
    Serial.begin(9600);
    while (can.begin(CAN_500KBPS) != CAN_OK) {
        Serial.println("CAN bus fail!");
        delay(100);
    }
    Serial.println("CAN bus ok!");
    return can;
}

inline CANPacket can_read_i2c(I2C_CAN &can) {
    if (can.checkReceive() == CAN_MSGAVAIL) {
        CANPacket packet;
        packet.len = 0;
        packet.id = 0xFFFFFFFFFFF;
        can.readMsgBuf((unsigned char *)&packet.len, packet.buf);
        packet.id = can.getCanId();
        return packet;
    } else {
        CANPacket packet;
        packet.len = 8;
        packet.id = 0xFFFFFFFFFFF;
        return packet;
    }

}
inline void can_send_i2c(I2C_CAN &can, CANPacket packet) {
    can.sendMsgBuf(packet.id, CAN_STDID, 8, packet.buf);
}

void setup()
{
    I2C_CAN can = setup_can_i2c();
    // PinModes
    pinMode(HALL_PIN_L, INPUT_PULLUP);
    pinMode(HALL_PIN_R, INPUT_PULLUP);

    double tolerance = 1.0; // 1 mm
    PitchController control(tolerance);
    // Interrupts
    // attachInterrupt(digitalPinToInterrupt(HALL_PIN_L, left_hall_handler, FALLING);
    // attachInterrupt(digitalPinToInterrupt(HALL_PIN_R, right_hall_handle, FALLING);

    double current_set_point = 0.0;
    bool home_state = false;
    bool e_stopped = false;
    // controller.loop();
    for(;;){
        int CMD_State = Dir::Stop;
        CANPacket packet = can_read_i2c(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                e_stopped = frame.stop;
                CMD_State = Dir::Stop;
            }
        }

        CANPacket driver_telemetry = {DAVID_PITCH_DRIVER_TELEM_FRAME_ID, 0};
        control.pack_telemetry(driver_telemetry.buf);
        can_send_i2c(can, driver_telemetry);


        delay(300);
        control.left_m.setDirection(Dir::Extend);
        control.right_m.setDirection(Dir::Extend);
        delay(300);
        control.left_m.setDirection(Dir::Retract);
        control.right_m.setDirection(Dir::Retract);
        delay(300);
        // CANPacket driver_telemetry = {DAVID_PITCH_DRIVER_TELEM_FRAME_ID, 0};
        // can_send(can, driver_telemetry)
        if (e_stopped)
            continue;

        // switch (packet.id) {
        //     FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {

        //         home_state = frame.home;
        //         if (frame.home) {
        //             break;
        //         }
        //         control.setPoint(david_pitch_ctrl_set_point_decode(frame.set_point));
        //         control.setLeftOffset(david_pitch_ctrl_left_offset_decode(frame.left_offset));
        //         control.setRightOffset(david_pitch_ctrl_right_offset_decode(frame.right_offset));

        //     }
        // }


        // control.loop();
        // left_m.setDirection(Dir::Stop);
        // right_m.setDirection(Dir::Stop);
        // delay(2000);
        // left_m.setDirection(Dir::Extend);
        // right_m.setDirection(Dir::Extend);
        // delay(2000);
        // left_m.setDirection(Dir::Stop);
        // right_m.setDirection(Dir::Stop);
        // delay(2000);
    }
}
