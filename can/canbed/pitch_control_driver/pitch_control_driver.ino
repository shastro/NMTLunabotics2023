// receive a frame from can bus
#include "arduino_lib.hpp"

// Import CAN message constants
#include "david.h"
// Interrupt pins for the MEGA
// 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)


// I2C Registers
#define SOFTREG             0x07                    // Byte to read software
#define CMDREG              0x00                    // Command byte
#define SPEEDREG            0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temperature
#define CURRENTREG          0x05                    // Byte to read motor current
#define STATUSREG           0x01
#define ACCREG              0x03


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
        // while(Wire.available() < 1) {
        //     Serial.println("Waiting for i2c");
        // };       // Waits for byte to become available
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


struct ControlCommand {
    double set_point = 0.0;
    double left_offset = 0.0;
    double right_offset = 0.0;
    bool home = false;

};

struct PitchController {

    MD04Driver left_m;
    MD04Driver right_m;
    double tolerance;

    double left_pos;
    double right_pos;

    ControlCommand command;

    PitchController(double tolerance_) : left_m(0x59), right_m(0x58) {
        tolerance = tolerance_;
        left_pos = 0.0;
        right_pos = 0.0;

        left_dir = Dir::Stop;
        right_dir = Dir::Stop;

        left_count = 0;
        right_count = 0;

        left_offset = 0.0;
        right_offset = 0.0;

    }

    void setPoint(double set_point_) {
        set_point = set_point_;
    }

    void setLeftDirection(double left_direction_){
        left_direction = left_direction_;
    }
    
    void setRightDirection(double right_direction_){
        right_direction = right_direction_;
    }

    void setLeftPosition(double left_position_){
        left_position = left_position_;
    }

    void setRightPosition(double right_position_){
        right_position = right_position_;
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
        data.left_current = david_pitch_driver_telem_left_current_encode((double)left_current);
        data.right_current = david_pitch_driver_telem_right_current_encode((double)right_current);
        david_pitch_driver_telem_pack(buf, &data, 8);
    }
    void loop(){
        int ticks = 100;
        const int read_limit_frequency = 200;
        while(ticks > 0){
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
            ticks--;
            delay(10);
        }
    }
};


void setup()
{
    MCP_CAN can = setup_can();
    // PinModes

    Wire.begin();
    delay(100);

    double tolerance = 1.0; // 1 mm
    PitchController control(tolerance);

    bool home_state = false;
    bool e_stopped = false;

    int command_interval = 100;
    int tick = 0;
    for(;;){
        int CMD_State = Dir::Stop;
        CANPacket packet = can_read(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                e_stopped = frame.stop;
                CMD_State = Dir::Stop;
            }
        }

        CANPacket driver_telemetry = {DAVID_PITCH_DRIVER_TELEM_FRAME_ID, 0};
        control.pack_telemetry(driver_telemetry.buf);
        can_send(can, driver_telemetry);

        if (e_stopped)
            continue;

        // if (tick % command_interval < command_interval/2) {
        //     control.left_m.setDirection(Dir::Extend);
        //     control.right_m.setDirection(Dir::Extend);
        // } else {
        //     control.left_m.setDirection(Dir::Retract);
        //     control.right_m.setDirection(Dir::Retract);
        // }
        // tick += 1;

        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {

                home_state = frame.home;
                if (frame.home) {
                    break;
                }
                control.setPoint(david_pitch_ctrl_set_point_decode(frame.set_point));
                control.setLeftOffset(david_pitch_ctrl_left_offset_decode(frame.left_offset));
                control.setRightOffset(david_pitch_ctrl_right_offset_decode(frame.right_offset));

            }
            FRAME_CASE(DAVID_PITCH_POSITION_TELEM, david_pitch_position_telem) { 
                 control.setLeftPosition(david_pitch_position_telem_left_position_decode(frame.left_position));
                 control.setRightPosition(david_pitch_position_telem_right_position_decode(frame.right_position));
                 control.setLeftDirection(david_pitch_position_telem_left_direction_decode(frame.left_direction));
                 control.setRightDirection(david_pitch_position_telem_right_direction_decode(frame.right_direction));
            }
        }


        control.loop();
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
