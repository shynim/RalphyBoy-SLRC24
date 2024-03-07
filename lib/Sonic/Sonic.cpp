#include <Sonic.h>

long mm,duration;

int Sonic::readMili(){
    
    digitalWrite(t, LOW);
    delayMicroseconds(5);
    digitalWrite(t, HIGH);
    delayMicroseconds(10);
    digitalWrite(t, LOW);

    pinMode(e, INPUT);
    duration = pulseIn(e, HIGH,2000);

    mm = (duration/2) * 0.343;    

    return mm;
}

int Sonic::readCenti(){
    delay(5);
    return sonic.ping_cm();
}