#include <Arduino.h>
#include <SensorPanel.h>
#include <MotorDriver.h>
#include <PID.h>
#include <Tof.h>
#include <Radar.h>
#include <SoftwareSerial.h>
#include <Sonic.h>
#include <MetalDetector.h>
#include <Arm.h>
#include <Lox.h>

Sonic wallSonic(41, A12, 100);
Sonic backSonic(43, 429 , 100);

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
  while(leftEncoder < 5){
    driver.forward(255,255);
  }
  driver.brake();

}

void breakBackward(){
  while(leftEncoder < 5){
    driver.backward(255,255);
  }
  driver.brake();

}

void autoPosition(int frontGap, char lox){

  int setPoint = frontGap; 

  initEncoder();

  int reading;
  reading = lox == 'f'? readFrontLox() : readRadarLox();
  int driveErr = setPoint - int(reading / 10);

  if(driveErr > 0 ){
    breakBackward();
  }
  else if(driveErr < 0){
    breakForward();
  }

  while(true){
    reading = lox == 'f'? readFrontLox() : readRadarLox();
    int driveErr = setPoint - int(reading / 10);
    
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

void goStraight(int distance){
  initEncoder();
  while(leftEncoder <= distance || rightEncoder <= distance){
    int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
    int correction = encoderPid(err);
    driver.applyEncoderPid(correction, 80);
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

  initElbow(35,180);
  initWrist(0,180);
  initGripper(0,180);
  
  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

  pinMode(xshutFront, OUTPUT);

  frontLox.shut();
  radarLox.shut();

  initLoxes();

  frontQtr.calibrate(1000);

}

void botLoop(){
  //rearQtr.read();
  frontQtr.read();

  for(int i = 0;i < 8; i++){
    Serial.print(frontQtr.rawReadings[i]);
    Serial.print("  ");
  }
  Serial.println();
  // int correction = pid(frontQtr.error * -1, true);
  // driver.applyLinePid(correction, true);

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

void placeCube(){
  positionArm();
  delay(2000);

  // armDown(500,true);
  // delay(2000);

  // grabCube();
  // delay(2000);

  // armUp(2000);
  // delay(2000);

  // armDown(1000, false);
  // delay(2000);

  // spreadGripper();
  // delay(2000);

  // breakBackward();
  // initEncoder();

  // while(leftEncoder < 60 || rightEncoder < 60){
  //   driver.backward(60,60);
  //   Serial.print(int(leftEncoder * 0.96));
  //   Serial.print("   ");
  //   Serial.print(rightEncoder);
  //   Serial.println();
  // }
  // driver.stop();
  // delay(2000);

  // writeElbow(120);
  // delay(1000);

  // positionArm();
  // delay(2000);

  armDownClose();
  delay(5000);

  spreadGripper();
  delay(2000);

  grabCube();
  delay(2000);

  writeElbow(50);
  delay(50000);

}

bool checkMiddleRadarPos(){
  turnRadar(65,500);
  int reading = readRadarLox();
  if(reading < 100 && reading != -1){
    return true;
  }else{
    return false;
  }
}

void trashYard(){

  bool wallFound = false;
  int radarState = 0; //1->2->3 ===> 20->55->90

  centerRadar();
  delay(100);

  breakForward();

  while(true){

    int reading = readFrontLox();
    if(reading < 150 && reading != -1){
      driver.stop(100);
      radarState = 0;
      break;
    }

    turnRadar(50,0);
    int startTime = millis();
    wallFollow(60, startTime);
    reading = readRadarLox();
    if(reading < 150 && reading != -1){
      driver.stop(100);
      radarState = 1;
      if(checkMiddleRadarPos()){
        radarState = 2;
      }
      break;
    }
    // Serial.print(radarLox.reading);
    // Serial.print("  ");

    turnRadar(70,0);
    startTime = millis();
    wallFollow(60, startTime);
    reading = readRadarLox();
    if(reading < 150 && reading != -1){
      driver.stop(100);
      radarState = 2;
      break;
    }
    // Serial.print(radarLox.reading);
    // Serial.print("  ");

    turnRadar(90,0);
    startTime = millis();
    wallFollow(60, startTime);
    reading = readRadarLox();
    if(reading < 150 && reading != -1){
      driver.stop(100);
      radarState = 3;
      if(checkMiddleRadarPos()){
        radarState = 2;
      }
      break;
    }

  }
  
  turnRadar(90,75);

  int reading = readRadarLox();

  if(reading < 200 && reading != -1){
    if(readFrontLox() < 150){
      wallFound = true;
    }
  }

  if(wallFound){
    autoPosition(11, 'f');
    oneWheelTurn('l', 420);
    
  }else{
    if(radarState == 0){
      autoPosition(15, 'f');
    }else if(radarState == 1){
      turnRadar(50, 75);
      autoPosition(15, 'r');
    }
    else if(radarState == 2){
      turnRadar(70, 75);
      autoPosition(15, 'r');
    }else{
      turnRadar(90, 75);
      autoPosition(15, 'r');
    }

    oneWheelTurn('l', 420);

    if(radarState == 0){ 
      oneWheelTurn('r', 420);
      delay(100000);
    }else if(radarState == 1){
      breakForward();
      goStraight(50);
      driver.stop();
      oneWheelTurn('r', 420);
      delay(100000);
    
    }else if(radarState == 2){
      breakForward();
      goStraight(100);
      driver.stop();
      oneWheelTurn('r', 420);
      delay(100000);
      
    }else{
      driver.stop();
      delay(10000000);
      
    }    
  }


  
}

void setup(){
  delay(50);
  botSetup();

  positionArm();
  
  // frontQtr.calibrate(1);
  // rearQtr.calibrate(1);
  // driver.stop();

}

void loop(){
  // trashYard();
  //placeCube();
  attachGripper();
  grabCube();

  // Serial.println(detectMetal());

  // while(true){
  //   Serial.print(int(leftEncoder * 0.96));
  //   Serial.print("   ");
  //   Serial.print(rightEncoder);
  //   Serial.println();

  //   int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
  //   int correction = encoderPid(err);
  //   driver.applyEncoderPid(correction, 60);
  // }
  

  // attachGripper();
  // gripper.write(60);


  // breakForward();
  // while(leftEncoder < 20000000 || rightEncoder < 200000000){
  //   botLoop();

  // }
  // driver.stop();
  // delay(10000);
  
  // detachElbow();
  // detachWrist();

  // radarLox.init();
  
  // while(true){
  //   int startTime = millis();
  //   readRadar();
  //   Serial.print(radarLox.reading);
  //   Serial.print("  ");
  //   Serial.println(millis() - startTime);
  // }
  
}