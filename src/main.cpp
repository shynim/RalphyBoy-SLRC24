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
#include <Colour.h>
#include <LED.h>

Sonic wallSonic(41, A12, 100);
Sonic backSonic(43, 42 , 100);

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
unsigned long leftCount = 1;
unsigned long rightCount = 1;

const int pushDistanceForward = 150;
const int pushDistanceBackward = 90;

char wallColour;

int juncToSemicircle = 0;
int juncToCircle = 0;

int turnSpeed = 100;
int baseSpeed = 90;
boolean boxPicked = false;

boolean rightBox = false;
boolean frontBox = false;

void countLeftOut1(){
  leftEncoder += 1;
}

void countRightOut1(){
  rightEncoder += 1;
}

void initEncoder(){
  leftCount = 1;
  rightCount = 1;
  leftEncoder = 1;
  rightEncoder = 1;
}

void breakForward(){
  initEncoder();
  while(leftEncoder < 5){
    driver.forward(255,255);
  }
  driver.brake();

}

void breakBackward(){
  initEncoder();
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

void autoLinePosition(int frontGap, char lox, boolean limitSpeed){
  int setPoint = frontGap; 

  int reading;
  reading = lox == 'f'? readFrontLox() : readBackLox();
  int driveErr = setPoint - int(reading / 10);

  if(driveErr > 0 ){
    if(lox == 'f'){
      breakBackward();
    }else{
      breakForward();
    }
  }
  else if(driveErr < 0){
    if(lox == 'f'){
      breakForward();
    }else{
      breakBackward();
    }
  }

  while(true){
    reading = lox == 'f'? readFrontLox() : readBackLox();
    int driveErr = setPoint - int(reading / 10);
    
    int speed = 60 + (2 * abs(driveErr));
    if(speed > 70 && limitSpeed){speed = 70;}
    if(driveErr > 0){
      if(lox == 'f'){
        rearQtr.read();
        int correction = pid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 50);
      }else{
        rearQtr.read();
        int correction = pid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 50);
      }
      
    }else if(driveErr < 0){
      if(lox == 'f'){
        frontQtr.read();
        int correction = pid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, speed, speed + 50);
      }else{
        rearQtr.read();
        int correction = pid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 50);
      }
      
    }else{
      break;
    }
  }

}

void goStraight(boolean frwrd, int baseSpeed = 80){
  int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
  int correction = encoderPid(err);
  int speed = frwrd == true ? baseSpeed : -baseSpeed;
  driver.applyEncoderPid(correction, speed);

}

void pushForward(int distance, boolean frwrd){
  initEncoder();
  while(leftEncoder <= distance || rightEncoder <= distance){
    int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
    int correction = encoderPid(err);
    int speed = frwrd == true ? 80 : -80;
    driver.applyEncoderPid(correction, speed);
  }
  driver.stop(100);
}

int pushLineForward(int distance, boolean frwrd, boolean metalDetect, int baseSpeed = 60){
  initEncoder();
  int count = 0;
  if(frwrd){
    while(leftEncoder <= distance || rightEncoder <= distance){
      frontQtr.read();
      int correction = pid(frontQtr.error * -1, true);
      driver.applyLinePid(correction, true, baseSpeed, baseSpeed + 50);
      if(metalDetect){count += detectMetal();}
    }
  }else{
    while(leftEncoder <= distance || rightEncoder <= distance){
      rearQtr.read();
      int correction = pid(rearQtr.error, false);
      driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
    }
  }
  driver.stop(100);
  return count;
}

void printEncoder(){
  Serial.print(leftEncoder);
  Serial.print("  ");
  Serial.print(rightEncoder);
  Serial.println();
}

void oneWheelTurn(char dir, int distance){
  initEncoder();
  if(dir == 'r'){
    leftCount += distance;
    while(leftEncoder <= leftCount){
      printEncoder();
      driver.forward(100,0);
    }
  }else if(dir == 'l'){
    rightCount += distance;
    while(rightEncoder <= rightCount){
      printEncoder();
      driver.forward(0,100);
    }
  }
  driver.stop(100);
}

void turnLeft90(){
  initEncoder();
  leftCount += 190;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnLeft(turnSpeed, turnSpeed);
  }
  driver.stop(100);
}

void turnRight90(){
  initEncoder();
  leftCount += 180;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnRight(turnSpeed, turnSpeed);
    
  }
  driver.stop(100);
}

void turnRightTillMiddle(boolean frwrd){
  driver.turnRight(turnSpeed,turnSpeed);
  delay(200);
  if(frwrd){
    frontQtr.read();
    while(frontQtr.panelReading[3] == 1){
      driver.turnRight(turnSpeed, turnSpeed);
      frontQtr.read();
    }
    
    frontQtr.read();
    while(frontQtr.panelReading[3] != 1){
      driver.turnRight(turnSpeed, turnSpeed);
      frontQtr.read();
    }
  }else{
    rearQtr.read();
    while(rearQtr.panelReading[3] == 1){
      driver.turnRight(turnSpeed, turnSpeed);
      rearQtr.read();
    }
    
    rearQtr.read();
    while(rearQtr.panelReading[3] != 1){
      driver.turnRight(turnSpeed, turnSpeed);
      rearQtr.read();
    }
  }
}

void turnLeftTillMiddle(boolean frwrd){
  driver.turnLeft(turnSpeed,turnSpeed);
  delay(200);
  if(frwrd){
    frontQtr.read();
    while(frontQtr.panelReading[4] == 1){
      driver.turnLeft(turnSpeed, turnSpeed);
      frontQtr.read();
    }
    
    frontQtr.read();
    while(frontQtr.panelReading[4] != 1){
      driver.turnLeft(turnSpeed, turnSpeed);
      frontQtr.read();
    }
  }else{
    rearQtr.read();
    while(rearQtr.panelReading[4] == 1){
      driver.turnLeft(turnSpeed, turnSpeed);
      rearQtr.read();
    }
    
    rearQtr.read();
    while(rearQtr.panelReading[4] != 1){
      driver.turnLeft(turnSpeed, turnSpeed);
      rearQtr.read();
    }
  }
}

void turnLeftToCircle(){
  initEncoder();
  leftCount += 170;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnLeft(turnSpeed, turnSpeed);
  }
  driver.stop(100);
}

void turnBack(){
  initEncoder();
  leftCount += 410;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnLeft(turnSpeed, turnSpeed);
  }
  driver.stop();
}

void wallFollow(int time, int startTime){
  while(int(millis() - startTime) <= time){
    int err = 6 - wallSonic.readCenti();
    int correction = wallPid(err);
    driver.applyWallPid(correction * -1);
  }
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
      pushForward(50, true);
      driver.stop();
      oneWheelTurn('r', 420);
      delay(100000);
    
    }else if(radarState == 2){
      breakForward();
      pushForward(100, true);
      driver.stop();
      oneWheelTurn('r', 420);
      delay(100000);
      
    }else{
      driver.stop();
      delay(10000000);
      
    }    
  }

}

void lineFollowForward(char turn = 'n', boolean checkBoxes = false){
  int pushDistance = pushDistanceForward;
  while (true) {

    frontQtr.read();
    if (frontQtr.pattern == 1) {
      int correction = pid(frontQtr.error * -1, true);
      driver.applyLinePid(correction, true, baseSpeed, baseSpeed + 50);

    }else{
      char pattern = frontQtr.pattern;

      if(checkBoxes){
        int reading = wallSonic.readCenti();
        if(reading < 60 && reading != -1){
          light(1, 'g');
          rightBox = true;
        }
      }

      bool left = pattern == 'L';
      bool right = pattern == 'R';
      bool t = pattern == 'T';

      initEncoder();

      leftCount = pushDistance;
      rightCount = pushDistance;

      int tCount = 0;
      while(rightEncoder <= rightCount || leftEncoder <= leftCount){
        
        goStraight(true);

        frontQtr.read();
        
        if (frontQtr.pattern == 'L') {
          left = true; 
        } else if (frontQtr.pattern == 'R') {
          right = true;
        } else if (frontQtr.pattern == 'T') {
          t = true;
          tCount++;
        }

      }

      if (t || (left && right)) {
        pattern = 'T';
      } else if (left) {
        pattern = 'L';
      } else if (right) {
        pattern = 'R';
      } else {
        pattern = 0;
      }
      driver.stop(100);
      
      frontQtr.read();
      char newPattern = frontQtr.pattern;

      if(newPattern == 'T'){
        break;
      }

      if(checkBoxes){
        int reading = readFrontLox();
        if(reading < 500 && reading != -1){
          light(2, 'r');
          frontBox = true;
        }
      }

      if(newPattern == 1 || pattern == 'T'){
        if(turn == 'r'){
          turnRightTillMiddle(true);
        }else if(turn == 'l'){
          turnLeftTillMiddle(true);
        }else{
          if(checkBoxes){
            if(rightBox){
              turnRightTillMiddle(true);
            }
            else if(frontBox){

            }else{
              turnLeftTillMiddle(true);
            }
          }
        }
        break;
      }else{
        if(pattern == 'L'){
          turnLeftTillMiddle(true);
        }else if(pattern == 'R'){
          turnRightTillMiddle(true);
        }else{
          turnBack();
        }
        break;
      }

    }
  }
  driver.stop(100);  

}

void lineFollowBackward(char turn = 'n'){
  int pushDistance = pushDistanceBackward;
  while (true) {

    rearQtr.read();
    if (rearQtr.pattern == 1) {
      int correction = pid(rearQtr.error, false);
      driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);

    }else{
      char pattern = rearQtr.pattern;
      Serial.println(pattern);

      bool left = pattern == 'L';
      bool right = pattern == 'R';
      bool t = pattern == 'T';

      initEncoder();

      leftCount = pushDistance;
      rightCount = pushDistance;

      int tCount = 0;
      while(rightEncoder <= rightCount || leftEncoder <= leftCount){
        
        goStraight(false);

        rearQtr.read();
        
        if (rearQtr.pattern == 'L') {
          left = true; 
        } else if (rearQtr.pattern == 'R') {
          right = true;
        } else if (rearQtr.pattern == 'T') {
          t = true;
          tCount++;
        }

      }

      if (t || (left && right)) {
        pattern = 'T';
      } else if (left) {
        pattern = 'L';
      } else if (right) {
        pattern = 'R';
      } else {
        pattern = 0;
      }
      driver.stop(100);
      
      rearQtr.read();
      char newPattern = rearQtr.pattern;

      if(newPattern == 'T'){
        break;
      }

      if(newPattern == 1 || pattern == 'T'){
        if(turn == 'r'){
          turnRightTillMiddle(false);
        }else if(turn == 'l'){
          turnLeftTillMiddle(false);
        }else if(turn == 'u'){
          turnBack();
        }else{

        }
        break;
      }else{
        if(pattern == 'L'){
          turnLeftTillMiddle(false);
        }else if(pattern == 'R'){
          turnRightTillMiddle(false);
        }else{
          turnBack();
        }
        break;
      }
      driver.stop(100);  

    }
  }
  
}

void wall(){

  lineFollowBackward();

  while(readBackLox() > 150){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false);
  }

  autoLinePosition(4, 'b', false);
  driver.stop();
  delay(500);

  bool green = checkGreenWall();
  wallColour = green == true ? 'g' : 'b';

  light(1, wallColour);
  delay(100);
  turnOffAllLeds();

  lineFollowForward('r');
  lineFollowForward();
  lineFollowForward('l');
}

void reachColourJunction(char colour){
  if(colour == 'w'){
    while(true){
      frontQtr.read();
      if(frontQtr.pattern == 'L' || frontQtr.pattern == 'R' || frontQtr.pattern == 'T'){
        break;
      }
      int correction = pid(frontQtr.error, true);
      driver.applyLinePid(correction * -1, true);
    }
  }else if(colour == 'b' || colour == 'g'){
    initEncoder();
    while(leftEncoder < juncToCircle){
      frontQtr.read();
      int correction = pid(frontQtr.error, true);
      driver.applyLinePid(correction * -1, true);
    }
  }else{
    initEncoder();
    while(leftEncoder < juncToSemicircle + 220){
      rearQtr.read();
      int correction = pid(rearQtr.error, false);
      driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
    }
  }
  
}

char getJuctionColour(){
  if(checkGreenColourJunction()){
    return 'g';
  }else{
    return 'b';
  }
}

void goToCircle(char colour){
  int pushDistance = 180;
  pushForward(pushDistance, true);

  driver.stop(100);

  if(colour == wallColour){
    turnLeftTillMiddle(true);
  }else{
    turnRightTillMiddle(true);
  }
  driver.stop(100);

  initEncoder();
  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 'L' || frontQtr.pattern == 'R' || frontQtr.pattern == 'T'){
      break;
    }
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
  }

  juncToCircle = leftEncoder;
  pushForward(pushDistanceForward, true);
  driver.stop(100);
  if(colour == wallColour){
    turnRightTillMiddle(true);
  }else{
    turnLeftTillMiddle(true);
  }
  driver.stop(100);

}

double checkReadings(uint8_t *readings, int size) {
  int sum = 0;
  for (int i = 0; i < size; ++i) {
      sum += readings[i];
  }
  double mean = sum / size;

  double squaredDiffSum = 0.0;
  for (int i = 0; i < size; ++i) {
      squaredDiffSum += pow(readings[i] - mean, 2);
  }
  double variance = squaredDiffSum / size;
  double stdDeviation = sqrt(variance);

  //double threshold = 0.1 * mean;

  return stdDeviation;
}

void circle(){
  lineFollowForward('l');

  initEncoder();
  int readCount = 0;
  uint8_t readings[200] = {};

  while(leftEncoder < 800){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
    if(leftEncoder > 300 && leftEncoder < 700){
      readings[readCount] = backSonic.readCenti();
      Serial.println(readings[readCount]);
      readCount++;
    }
  } 

  driver.stop(100);

  double std = checkReadings(const_cast<uint8_t *>(readings), readCount);
  int sum = 0;
  for (int i = 0; i < readCount; ++i) {
      sum += readings[i];
  }
  double mean = sum / readCount;

  // Serial.print(mean);
  // Serial.print("  ");
  // Serial.print(std);
  // Serial.println();

  if(mean < 30.0 && std < 2.5){
    light(2, 'g');
  }else{
    light(2, 'b');
  }

  while(true){
    rearQtr.read();
    if(rearQtr.pattern == 'R'){
      break;
    }
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false);
  }
  pushForward(pushDistanceBackward, false);
  driver.stop(100);
  turnLeftTillMiddle(true);
  driver.stop(100);
  lineFollowForward();
  
}

void goToSemicircle(char colour){
  pushForward(pushDistanceForward, true);
  driver.stop(100);

  if(colour == wallColour){
    turnLeftTillMiddle(true);
  }else{
    turnRightTillMiddle(true);
  }
  driver.stop(100);
  
}

int reachBox(){
  
  while(readFrontLox() > 150){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
  }

  autoLinePosition(6, 'f', true);

  int distance = 35;
  int count = 0;
  
  count += pushLineForward(distance, true, false);

  unsigned long startTime = millis();
  while((millis() - startTime) < 1000){
    count += detectMetal();
  }
  startTime = millis();
  bool pushed = false;
  while((millis() - startTime) < 1000){
    if(!pushed){
      breakForward();
      initEncoder();
      while(leftEncoder < 25 || rightEncoder < 25){
        frontQtr.read();
        int correction = pid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true);
      }
      driver.stop(100);
      pushed = true;
    }
    count += detectMetal();
  }
  digitalWrite(13, LOW);
  return count;

}

bool foundMetalBox(int count){

  return count >= 1000;
}

void grabCube(){
  armDownClose();
  delay(250);

  spreadGripper();
  delay(250);

  initEncoder();
  while(leftEncoder < 50 || rightEncoder < 50){
    if(leftEncoder == 20 || rightEncoder == 20){
      gripCube();
    }
    goStraight(true, 100);
  }
  driver.stop(100);

  attachWrist();
  writeWrist(50);
  delay(250);

  attachElbow();
  writeElbow(35);
  delay(250);

  turnSpeed = 130;
  baseSpeed = 90;

}

void exitSemicircle(char orientation){
  if(orientation == 'n'){
    lineFollowBackward();
  }else if(orientation == 'w'){
    lineFollowBackward('r');
  }else if(orientation == 'e'){
    lineFollowBackward('l');
  }

}

void semicircle(){
  char orientation = 'n';
  bool metal = false;
  
  initEncoder();
  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 'L' || frontQtr.pattern == 'R' || frontQtr.pattern == 'T'){
      break;
    }
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
  }

  juncToSemicircle = leftEncoder;
  lineFollowForward('n', true);
  
  char dir;
  if(rightBox){
    orientation = 'e';
    if(frontBox){
      dir = 'l';
    }else{
      dir = 'u';
    }
  }else if(frontBox){
    dir = 'l';
  }else{
    orientation = 'w';
  }
  metal = foundMetalBox(reachBox());
  if(metal){
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation);
    return;
  }

  lineFollowBackward(dir);
  if(dir == 'l' && orientation == 'e'){orientation = 'n';}
  else if(dir == 'l' && orientation == 'n'){orientation = 'w';}
  else if(dir == 'u'){orientation = 'w';}
  if(orientation == 'w'){
    reachBox();
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation);
    return;
  }
  metal = foundMetalBox(reachBox());
  if(metal){
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation);
    return;
    
  }

  dir = 'l';
  lineFollowBackward(dir);
  orientation = 'w';
  reachBox();
  driver.stop(100);
  grabCube();
  exitSemicircle(orientation);
  return;
}

void goToPlace(){
  
  pushForward(100, false);

  initEncoder();
  while(leftEncoder < 100 || rightEncoder < 100){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  }

  
  lineFollowBackward();
  turnRightTillMiddle(false);

  initEncoder();
  while(leftEncoder < 100 || rightEncoder < 100){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, 80, 130);
  }

  driver.stop();

}

void placeCube(){

  autoLinePosition(25, 'f', true);
  driver.stop(500);

  attachWrist();
  attachElbow();

  writeElbow(78);
  delay(250);

  writeWrist(10);
  delay(250);

  spreadGripper();
  delay(250);

  armDownClose();

  turnSpeed = 100;
  baseSpeed = 90;

  attachGripper();
  gripCube();
  delay(100);

  detachGripper();
  detachElbow();
  detachWrist();

  initEncoder();
  while(leftEncoder < 160 || rightEncoder < 160){
    frontQtr.read();
    int correction = boxPid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, 80, 160);
  }
  driver.stop(100);

  spreadGripper();
  delay(500);

  pushLineForward(50, false, false, 100);

  attachWrist();

  writeWrist(65);
  delay(100);

  positionArm();
  delay(100);

  breakForward();
  pushLineForward(130, true, false, 100);

}

void botStart(){
  
  while(true){
    int reading = readBackLox();
    if(reading < 100 && reading != -1){
      delay(1000);
      pushForward(50, false);
      break;
    }
  }

}

void botEnd(){
  lineFollowBackward('r');

  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 'L' || frontQtr.pattern == 'R' || frontQtr.pattern == 'T'){
      break;
    }
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
  }
  
  pushForward(200, true);
  driver.stop(999999999);

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
  initColour();

  initElbow(35,180);
  initWrist(0,180);
  initGripper(0,180);
  initLED();

  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

  pinMode(xshutFront, OUTPUT);

  frontLox.shut();
  radarLox.shut();

  initLoxes();

  frontQtr.calibrate(1000);
  rearQtr.calibrate(1000);
}

void botLoop(){

  botStart();

  wall();

  reachColourJunction('w');
  driver.stop(100);
  char colour = getJuctionColour();

  goToCircle(colour);
  circle();
  
  reachColourJunction(wallColour);

  goToSemicircle(colour);
  semicircle();

  reachColourJunction('r');

  goToPlace();

  placeCube();

  botEnd();

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
  botLoop();

  // armDownClose();
  // delay(1000);

  // gripCube();
  // delay(500);

  // attachElbow();
  // writeElbow(81);
  // delay(500);

  // initEncoder();
  // while(leftEncoder < 190 || rightEncoder < 190){
  //   frontQtr.read();
  //   int correction = pid(frontQtr.error, true);
  //   driver.applyLinePid(correction * -1, true, 80, 130);
  // }
  // driver.stop();

  // spreadGripper();
  // delay(2000);

  // gripCube();
  // delay(3000);

  // attachWrist();

  // writeWrist(45);
  // delay(3000);

  // botLoop();
  
  // lineFollowBackward('l');
  // lineFollowForward();

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