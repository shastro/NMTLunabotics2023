#include "arduino_lib.hpp"

#include "david.h"

// I2C Registers
#define SOFTREG 0x07    // Byte to read software
#define CMDREG 0x00     // Command byte
#define SPEEDREG 0x02   // Byte to write to speed register
#define TEMPREG 0x04    // Byte to read temperature
#define CURRENTREG 0x05 // Byte to read motor current
#define STATUSREG 0x01
#define ACCREG 0x03

enum class Dir {
    Stop = 0,
    Extend = 1,
    Retract = 2,
};

#define ACC 0x03
#define SPEED 100

struct MD04Driver {
    int addr;
    Dir direction;

    MD04Driver(unsigned addr_in) {
        addr = addr_in;
        direction = Dir::Stop;
    }

    // I strongly suspect these functions crash the driver ~~Alex
    byte getTemperature() { return getData(TEMPREG); }

    byte getCurrent() { return getData(CURRENTREG); }

    void setDirection(Dir dir) {
        direction = dir;
        sendData(ACCREG, ACC);
        sendData(SPEEDREG, SPEED);
        sendData(CMDREG, (byte)dir);
    }

    byte getData(byte reg) { // Function for getting data from MD04
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom(addr, 1); // Requests byte from MD04
        byte data = Wire.read();

        return (data);
    }

    void sendData(byte reg, byte val) { // Function for sending data to MD04
        Wire.beginTransmission(addr);   // Send data to MD04
        Wire.write(reg);                // Command like Direction, Speed
        Wire.write(val);                // Value for the command
        int error = Wire.endTransmission();
        if (error) {
            Serial.print("I2C ERROR:");
            Serial.println(error);
        }
        delay(10);
    }
};

class PitchController {
  public:
    MD04Driver left_m;
    MD04Driver right_m;

    PitchController(MD04Driver left_m, MD04Driver right_m)
        : left_m(left_m), right_m(right_m) {}

    CANPacket pack_telemetry() {
        david_pitch_driver_telem_t data = {
            .left_current = david_pitch_driver_telem_left_current_encode(
                left_m.getCurrent()),
            .right_current = david_pitch_driver_telem_right_current_encode(
                right_m.getCurrent()),

            .left_temperature =
                david_pitch_driver_telem_left_temperature_encode(
                    left_m.getTemperature()),
            .right_temperature =
                david_pitch_driver_telem_right_temperature_encode(
                    right_m.getTemperature()),

            .left_direction = (uint8_t)left_m.direction,
            .right_direction = (uint8_t)right_m.direction,
        };

        CANPacket pkt(DAVID_PITCH_DRIVER_TELEM_FRAME_ID);
        pkt.len = 8;
        david_pitch_driver_telem_pack(pkt.buf, &data, 8);

        return pkt;
    }

    void set_directions(Dir left, Dir right) {
        left_m.setDirection(left);
        right_m.setDirection(right);
    }
};

void setup() {
    MCP_CAN can = setup_can();

    Wire.begin();
    delay(100);

    PitchController control(0x58, 0x59);
    bool e_stopped = false;

    scheduler_dense([&]() {
        CANPacket packet = can_read_nonblocking(can);
        if (!packet)
            return;

        Serial.print("got packet with ID ");
        Serial.println(packet.id);

        switch (packet.id) {
            FRAME_CASE(DAVID_E_STOP, david_e_stop) { e_stopped = frame.stop; }
        }

        if (e_stopped) {
            control.set_directions(Dir::Stop, Dir::Stop);
            return;
        }

        switch (packet.id) {
            FRAME_CASE(DAVID_PITCH_CTRL, david_pitch_ctrl) {
                Serial.println("setting directions");
                control.set_directions((Dir)frame.left, (Dir)frame.right);
            }
        }
    })
        .schedule(100,
                  [&]() {
                      // Sent telemetry every 100ms.
                      can_send(can, control.pack_telemetry());
                  })
        .run();
}
