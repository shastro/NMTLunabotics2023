#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "arduino_lib.hpp"

void setup() {
#define BACKWARD 1
#define FORWARD 0
#define DELAY 50

    OutPin rpul(10);
    OutPin rdir(6);
    OutPin lpul(4);
    OutPin ldir(11);
    
    InPin min_limit(A0);

    /* rdir.write(FORWARD); */
    /* ldir.write(FORWARD); */
    /* for (int i = 0; i < 100000; i++) { */
    /*     rpul.write(0); */
    /*     lpul.write(0); */
    /*     rpul.write(1); */
    /*     lpul.write(1); */
    /*     delayMicroseconds(DELAY); */
    /* } */

    /* rdir.write(BACKWARD); */
    /* ldir.write(BACKWARD); */
    /* while (!min_limit.read()) { */
        /* rpul.write(0);  */
        /* lpul.write(0); */
        /* rpul.write(1); */
        /* lpul.write(1); */
        /* delayMicroseconds(DELAY); */
    /* } */

    for (;;) {}
}

