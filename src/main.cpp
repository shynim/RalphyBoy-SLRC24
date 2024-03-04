#include <Arduino.h>
#include <SensorPanel.h>
#include <MotorDriver.h>
#include <PID.h>
#include <Tof.h>
#include <Radar.h>
#include <SoftwareSerial.h>

SensorPanel frontQtr(const_cast<uint8_t *>((const uint8_t[]) {25, 27, 31, 37, 39, 29, 35, 14}));
SensorPanel rearQtr(const_cast<uint8_t *>((const uint8_t[]) {24, 26, 28, 30, 32, 34, 36, 38}));

MotorDriver driver;

SoftwareSerial bt(0,1);

const int xshutFront = 15;

Tof frontLox(const_cast<int *>((const int[]) {xshutFront, 1, 1}));

const int leftPins[] = {11, 52, 50};
const int rightPins[] = {12, 46, 48};

const int leftEncoderPin = 19;
const int rightEncoderPin = 18;

unsigned volatile long leftEncoder = 1;
unsigned volatile long rightEncoder = 1;
unsigned long leftCount = 0;
unsigned long rightCount = 0;

void countLeftOut1(){
    leftEncoder += 1;
}
void countRightOut1(){
    rightEncoder += 1;
}

void botSetup(){
  
  Serial.begin(9600);
  bt.begin(9600);

  pinMode(leftEncoderPin, INPUT);
  pinMode(rightEncoderPin, INPUT);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRightOut1, RISING);
  //attachInterrupt(digitalPinToInterrupt(rightEncoderPins[0]), countRightOut1, RISING);
  //attachInterrupt(digitalPinToInterrupt(rightEncoderPins[1]), countRightOut1, RISING);

  initRadar();
  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

  pinMode(xshutFront, OUTPUT);

  frontLox.shut();
  radarLox.shut();
  
}

void botLoop(){
  rearQtr.read();
  frontQtr.read();
  int correction = pid(rearQtr.error * 1, false);
  driver.applyLinePid(correction, false);

}

void setup(){
  botSetup();
  
  // frontQtr.calibrate(1);
  // rearQtr.calibrate(1);
  // driver.stop();

}

void loop(){
  // Serial.print(int(leftEncoder * 0.95));
  // Serial.print("   ");
  // Serial.print(rightEncoder);
  // Serial.println();
  
  // int err = int(leftEncoder * 0.95) - rightEncoder;
  // int correction = encoderPid(err);
  // driver.applyEncoderPid(correction * -1);

  // radarLox.init();
  // centerRadar();
  // delay(1000);

  // for(int i = 78; i < 169; i++){
  //   turnRadar(i, 100);
  //   int count = 0;
  //   for(int i = 0; i < 10; i++){
  //     readRadar();
  //     if(radarLox.reading < 1500 && radarLox.reading > 0) count++;
  //   }
  //   if(count > 2){
  //     break;
  //   }

  // }

  // Serial.print("found");
  // delay(10000000);

  radarLox.init();
  centerRadar();
  while(true){

    readRadar();
    bt.println(radarLox.reading);
    
  }

}