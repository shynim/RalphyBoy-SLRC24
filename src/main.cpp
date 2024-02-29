#include <Arduino.h>
#include <SensorPanel.h>
#include <MotorDriver.h>
#include <PID.h>

SensorPanel frontQtr(const_cast<uint8_t *>((const uint8_t[]) {25, 27, 31, 37, 39, 29, 35, 14}));
SensorPanel rearQtr(const_cast<uint8_t *>((const uint8_t[]) {24, 26, 28, 30, 32, 34, 36, 38}));

MotorDriver driver;

const int leftPins[] = {11, 52, 50};
const int rightPins[] = {12, 46, 48};

void botSetup(){
  Serial.begin(9600);
  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

}

void botLoop(){


}

void setup(){
  botSetup();
  driver.turnLeft(80,80);
  frontQtr.calibrate(2);
  driver.turnRight(80,80);
  rearQtr.calibrate(2);
  driver.stop();

}

void loop(){
  
  rearQtr.read();
  frontQtr.read();
  int correction = pid(rearQtr.error * 1, false);
  driver.applyLinePid(correction, false);

}