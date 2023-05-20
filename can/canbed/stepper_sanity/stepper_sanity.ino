#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

void setup() {
#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0
#define MAX_LIMIT A1

    digitalWrite(RDIR, HIGH);
    digitalWrite(LDIR, HIGH);

    for (;;) {
        digitalWrite(RPUL, LOW);
        digitalWrite(LPUL, LOW);
        digitalWrite(RPUL, HIGH);
        digitalWrite(LPUL, HIGH);
        delayMicroseconds(50);
    }
}
