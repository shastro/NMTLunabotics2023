#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <linux/can.h>

// Basic file descriptor wrapper. Not really intended for use outside
// SocketCAN.
#include <string>
class fd {
    int id;

  public:
    fd();
    fd(int n);
    fd(fd &&other);
    ~fd();

    void operator=(fd &&other);
    int operator*();

    int into_raw();
};

// CAN socket.
class SocketCAN {
    fd sock;

  public:
    SocketCAN(const std::string &interface);

    void transmit(const can_frame &cf);
    void transmit(const uint8_t data[8]);
};

#endif /* CAN_INTERFACE_H */
