#include <Arduino.h>
#include <SensorPanel.h>
#include <MotorDriver.h>
#include <PID.h>
#include <Tof.h>
#include <Radar.h>
#include <SoftwareSerial.h>
#include <Sonic.h>
#include <MetalDetector.h>

Sonic wallSonic(41, A12, 100);

SensorPanel frontQtr(const_cast<uint8_t *>((const uint8_t[]) {25, 27, 31, 37, 39, 29, 35, 14}));
SensorPanel rearQtr(const_cast<uint8_t *>((const uint8_t[]) {24, A0, 28, 30, 32, 34, 36, 38}));

MotorDriver driver;

SoftwareSerial bt(0,1);

const int xshutFront = 15;

Tof frontLox(const_cast<int *>((const int[]) {xshutFront, 1, 0}));

const int leftPins[] = {11, 52, 50};
const int rightPins[] = {12, 46, 48};

const int leftEncoderPin = 19;
const int rightEncoderPin = 18;

unsigned long leftEncoder = 1;
unsigned long rightEncoder = 1;
unsigned long leftCount = 0;
unsigned long rightCount = 0;

void countLeftOut1(){
  leftEncoder += 1;
}
void countRightOut1(){
  rightEncoder += 1;
}

void initEncoder(){
  leftCount = 0;
  rightCount = 0;
  leftEncoder = 0;
  rightEncoder = 0;
}

void breakForward(){
  while(leftEncoder < 4){
    driver.forward(255,255);
  }
  driver.brake();

}

void breakBackward(){
  while(leftEncoder < 4){
    driver.backward(255,255);
  }
  driver.brake();

}

void autoPosition(int frontGap){

  int setPoint = frontGap; 

  frontLox.init();
  initEncoder();

  frontLox.read();
  int driveErr = setPoint - int(frontLox.reading / 10);

  if(driveErr > 0 ){
    breakBackward();
  }
  else if(driveErr < 0){
    breakForward();
  }

  while(true){
    frontLox.read();
    int driveErr = setPoint - int(frontLox.reading / 10);
    
    int speed = 40 + (2 * abs(driveErr));
    if(driveErr > 0){
      int err = rightEncoder - leftEncoder;
      int correction = encoderPid(err);
      
      driver.applyEncoderPid(correction, speed * -1);
    }else if(driveErr < 0){
      int err = rightEncoder - leftEncoder;
      int correction = encoderPid(err);
      
      driver.applyEncoderPid(correction, speed);
    }else{
      driver.stop(100);
      break;
    }
  }

}

void oneWheelTurn(char dir, int distance){
  initEncoder();
  if(dir == 'r'){
    leftCount += distance;
    while(leftEncoder <= leftCount){
      driver.forward(100,0);
    }
  }else if(dir == 'l'){
    rightCount += distance;
    while(rightEncoder <= rightCount){
      driver.forward(0,100);
    }
  }
  driver.stop(100);
}

void botSetup(){
  Serial.begin(9600);
  //bt.begin(9600);

  pinMode(leftEncoderPin, INPUT);
  pinMode(rightEncoderPin, INPUT);
  // pinMode(20, INPUT_PULLUP);
  // pinMode(21, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRightOut1, RISING);

  initRadar();
  initMetalDetector();
  
  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

  pinMode(xshutFront, OUTPUT);

  frontLox.shut();
  radarLox.shut();

}

void botLoop(){
  rearQtr.read();
  frontQtr.read();

  for(int i = 0;i < 8; i++){
    Serial.print(rearQtr.rawReadings[i]);
    Serial.print("  ");
  }
  Serial.println();
  // int correction = pid(rearQtr.error * 1, false);
  // driver.applyLinePid(correction, false);

}

void botStart(){

  radarLox.init();
  
  while(true){
    frontLox.read();
    if(frontLox.reading < 100 && frontLox.reading != -1){
      frontLox.shut();
      delay(1000);
      break;
    }
  }

}

void wallFollow(int time, int startTime){
  while((millis() - startTime) <= time){
    int err = 6 - wallSonic.readCenti();
    int correction = wallPid(err);
    driver.applyWallPid(correction * -1);
  }
}

void setup(){
  delay(50);
  botSetup();
  
  // frontQtr.calibrate(1);
  // rearQtr.calibrate(1);
  // driver.stop();

}

void loop(){

  Serial.println(detectMetal());

  // while(true){
  //   Serial.print(int(leftEncoder * 0.96));
  //   Serial.print("   ");
  //   Serial.print(rightEncoder);
  //   Serial.println();

  //   int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
  //   int correction = encoderPid(err);
  //   driver.applyEncoderPid(correction, 60);
  // }
  

  // Serial.print("start");
  // centerRadar();
  // delay(100);
  // radarLox.init();
  // breakForward();

  // while(true){
  //   turnRadar(20,0);
  //   int startTime = millis();
  //   wallFollow(75, startTime);
  //   readRadar();
  //   if(radarLox.reading < 200 && radarLox.reading != -1){
  //     driver.stop(100);
  //     break;
  //   }
  //   // Serial.print(radarLox.reading);
  //   // Serial.print("  ");

  //   turnRadar(55,0);
  //   startTime = millis();
  //   wallFollow(75, startTime);
  //   readRadar();
  //   if(radarLox.reading < 150 && radarLox.reading != -1){
  //     driver.stop(100);
  //     break;
  //   }
  //   // Serial.print(radarLox.reading);
  //   // Serial.print("  ");

  //   turnRadar(90,0);
  //   startTime = millis();
  //   wallFollow(75, startTime);
  //   readRadar();
  //   if(radarLox.reading < 150 && radarLox.reading != -1){
  //     driver.stop(100);
  //     break;
  //   }

  // }

  // radarLox.shut();

  // autoPosition(11);
  // frontLox.shut();

  // oneWheelTurn('l', 420);

}