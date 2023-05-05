#include <iostream>

void to_bytes(int n, uint8_t* buff, int start) {
    if (start + sizeof(n) >= 8)
        throw string("Not enough space in can frame");
    memcpy(buff + start, &n, sizeof(n));
}
