volatile int count = 0;//if the interrupt will change this value, it must be volatile

volatile double last_time = 0.0;
static const double MM_PER_COUNT = 0.17896;

double trig_delay = 12000;

void setup() {
    pinMode(3, INPUT_PULLUP); //set as input
    // EIFR = bit(INTF0);
    attachInterrupt(digitalPinToInterrupt(3), interruptName, FALLING); //Interrupt initialization
    Serial.begin(9600);
}//end setup

void loop() {

  // Serial.println(digitalRead(3));
  Serial.println(count*MM_PER_COUNT);//see the counts advance
  delay(10);//Delays usually can't be interfered with, here we will see the interrupt work
}//end loop

void interruptName()
{
    int64_t timestamp = micros();
    if ((timestamp - last_time) > trig_delay) {
        last_time = timestamp;
        count += 1;
    } 
}