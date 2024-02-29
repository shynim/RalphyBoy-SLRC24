#include <Arduino.h>
#include <SensorPanel.h>
#include <MotorDriver.h>

SensorPanel frontQtr(const_cast<uint8_t *>((const uint8_t[]) {}));
SensorPanel rearQtr(const_cast<uint8_t *>((const uint8_t[]) {}));

MotorDriver driver;

const int leftPins[] = {6, 9, 8};
const int rightPins[] = {7, 11, 10};

void botSetup(){
  Serial.begin(9600);
  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

}

void botLoop(){


}

void setup(){


}

void loop(){


}