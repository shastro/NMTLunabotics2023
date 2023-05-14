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


enum class Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};

#define ACC 0x0A
#define SPEED 100

struct MD04Driver {
    int addr;
    Dir direction;  

    MD04Driver(unsigned addr_in) {
        addr = addr_in;
        direction = Dir::Stop;
    }

    byte getTemperature() {
        return getData(TEMPREG);
    }

    byte getCurrent() {
        return getData(CURRENTREG);
    }

    void setDirection(Dir dir){
        direction = dir;
        sendData(ACCREG, ACC);
        sendData(SPEEDREG, SPEED);
        sendData(CMDREG, (byte)dir);
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

    double left_position;
    double right_position;

    double tolerance;

    bool home_done;

    ControlCommand command;

    PitchController(double tolerance_) : left_m(0x59), right_m(0x58) {
        left_position = 0.0;
        right_position = 0.0;
        home_done = false;
        tolerance = tolerance_;
    }

    inline bool in_range(double x, double set_point, double offset) {
        return ((x > (set_point + offset - tolerance)) && (x < (set_point + offset + tolerance)));
    }

    inline Dir choose_direction(double position, double set_point, double offset) {
        if (in_range(position, set_point, offset)) {
            return Dir::Stop;
        } else if (position < (set_point + offset - tolerance)) {
            return Dir::Extend;
        } else {
            return Dir::Retract;
        }
    }

    void setCommand(ControlCommand command_) {
        command = command_;
    }

    void setLeftPosition(double left_position_){
        left_position = left_position_;
    }

    void setRightPosition(double right_position_){
        right_position = right_position_;
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
        // Direction
        data.left_direction = (uint8_t)left_m.direction;
        data.right_direction = (uint8_t)right_m.direction;
        david_pitch_driver_telem_pack(buf, &data, 8);
    }
    bool home() {
        if (!home_done) {
            left_m.setDirection(Dir::Extend);
            right_m.setDirection(Dir::Extend);
            return true;
        } else {
            stop();
            command.home = false;
            return false;
        }
    }

    // Cycle function for testing
    void cycle(int tick) {
        int tickmod = tick % 1000;
        if(tickmod < 500) {
            left_m.setDirection(Dir::Retract);
            right_m.setDirection(Dir::Retract);
        } else {
         left_m.setDirection(Dir::Extend);
         right_m.setDirection(Dir::Extend);
        }
        delay(10);
    }

    void stop() {
        left_m.setDirection(Dir::Stop);
        right_m.setDirection(Dir::Stop);
        command.home = false;
    }

    void loop(MCP_CAN can){
        int ticks = 0;
        // Left
        left_m.setDirection(choose_direction(left_position, command.set_point, command.left_offset));
        right_m.setDirection(choose_direction(right_position, command.set_point, command.right_offset));

        // Serial.println((int)left_m.direction);
    }

};



void setup()
{
    MCP_CAN can = setup_can();
    // PinModes

    Wire.begin();
    delay(100);

    double tolerance = 5.0; // 1 mm
    PitchController control(tolerance);

    bool have_gotten_a_home_done = false;
    bool e_stopped = false;

    int command_interval = 100;
    int telem_freq = 5;
    long tick = 0;
    for(;;){
        CANPacket packet = can_read_nonblocking(can);
        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) {
                e_stopped = frame.stop;
            }
        }
        // if (packet.id == 0xFFFFFFFF) {
        //     delay(5);
        //     continue;
        // }

        if (tick % telem_freq == 0) {
            CANPacket driver_telemetry(DAVID_PITCH_DRIVER_TELEM_FRAME_ID);
            control.pack_telemetry(driver_telemetry.buf);
            can_send(can, driver_telemetry);
        }


        if (e_stopped) {
            control.stop();
            continue;
        }


        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {
                ControlCommand command;
                command.set_point = david_pitch_ctrl_set_point_decode(frame.set_point);
                command.left_offset = david_pitch_ctrl_left_offset_decode(frame.left_offset);
                command.right_offset = david_pitch_ctrl_right_offset_decode(frame.right_offset);
                command.home = frame.home;
                control.setCommand(command);

            }
            FRAME_CASE(DAVID_PITCH_POSITION_TELEM, david_pitch_position_telem) { 
                 control.setLeftPosition(david_pitch_position_telem_left_position_decode(frame.left_position));
                 control.setRightPosition(david_pitch_position_telem_right_position_decode(frame.right_position));
                 control.home_done = david_pitch_position_telem_home_done_decode(frame.home_done);
            }
        }


        if (control.home_done) {
            have_gotten_a_home_done = true;
        }
        
        if (control.command.home) {
            // Serial.println("Homing");
            control.home();
            control.command.set_point = 151.5;
        } else {
            // Serial.print("Control CMD: ");
            Serial.println(control.command.set_point);
            if (have_gotten_a_home_done) {
                control.loop(can);
            }
        }

        tick++;
    }
}
