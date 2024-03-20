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
    int reading = sonic.ping_cm();
    if(reading < 1000 && reading != 0){
        return reading;
    }else{
        return 80;
    }
}
