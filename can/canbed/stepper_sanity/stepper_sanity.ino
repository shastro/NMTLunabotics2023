#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "arduino_lib.hpp"

void setup() {
#define RPUL 10
#define RDIR 6
#define LPUL 4
#define LDIR 11
#define MIN_LIMIT A0
#define MAX_LIMIT A1

#define DIR_BACKWARD HIGH
#define DIR_FORWARD LOW

    //InPin min_limit(MIN_LIMIT);
    
    digitalWrite(RDIR, DIR_FORWARD);
    digitalWrite(LDIR, DIR_FORWARD);

    for (int i = 0; i < 40000; i++) {
        digitalWrite(RPUL, LOW);
        digitalWrite(LPUL, LOW);
        digitalWrite(RPUL, HIGH);
        digitalWrite(LPUL, HIGH);
        delayMicroseconds(50);
    }

    digitalWrite(RDIR, DIR_BACKWARD);
    digitalWrite(LDIR, DIR_BACKWARD);

   // while (!min_limit.read()) {
        digitalWrite(RPUL, LOW);
        digitalWrite(LPUL, LOW);
        digitalWrite(RPUL, HIGH);
        digitalWrite(LPUL, HIGH);
        delayMicroseconds(50);
  //  }


    for (;;) {}
}

