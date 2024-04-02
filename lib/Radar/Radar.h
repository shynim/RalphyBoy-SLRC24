#include <Arduino.h>
#include <Servo.h>
#include <Tof.h>

const int radarPin = 5;
const int railPin = 6;

const int xshut = 44;

Servo radarServo;
Servo railServo;
Tof radarLox(const_cast<int *>((const int[]) {xshut, 1, 0}));

int gemB = 0;

void initRadar(){
    pinMode(xshut, OUTPUT);
    radarServo.attach(radarPin);
    //railServo.attach(railPin);
}

void centerRadar(){
    radarServo.attach(radarPin);
    radarServo.write(78);
    delay(100);
    radarServo.detach();
}

void turnRadar(int ang, int del){
    radarServo.attach(radarPin);
    radarServo.write(ang);
    delay(del);
}

void detachRadar(){
    radarServo.detach();
}

void railUp(int del){
    railServo.attach(railPin);
    railServo.writeMicroseconds(1445); //1480 down // 
    delay(del);
    railServo.detach();
}

void railDown(int del){
    railServo.attach(railPin);
    railServo.writeMicroseconds(1480); //1480 down // 
    delay(del);
    railServo.detach();
}

int readRadar(){
    //return readRadarLox();
    //Serial.print(radarLox.reading);
}