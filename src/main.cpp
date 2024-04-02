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
#include <Gun.h>

Sonic wallSonic(41, A12, 100);
Sonic backSonic(43, 42 , 100);

SensorPanel frontQtr(const_cast<uint8_t *>((const uint8_t[]) {25, 27, 31, 37, 39, 29, 35, 14}));
SensorPanel rearQtr(const_cast<uint8_t *>((const uint8_t[]) {24, 7, A0, 30, 32, 34, 36, 38}));

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
const int pushDistanceBackward = 110;

char wallColour;

int juncToSemicircle = 0;
int juncToCircle = 0;

int turnSpeed = 110;
int baseSpeed = 80;
boolean boxPicked = false;

int rightOneWheelSpeed =440;
int leftOneWheelSpeed = 430;

boolean rightBox = false;
boolean frontBox = false;

int gemsA = 0;
int gemsB = 0;

void printEncoder(){
  Serial.print(leftEncoder);
  Serial.print("  ");
  Serial.print(rightEncoder);
  Serial.println();
}

void beep(){
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

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
    
    int speed = 60 + (2 * abs(driveErr));
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

void autoLinePosition(int frontGap, char lox, boolean limitSpeed, int baseSpeed = 60){
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
    
    int speed = baseSpeed + (2 * abs(driveErr));
    if(speed > baseSpeed + 10 && limitSpeed){speed = baseSpeed + 10;}
    if(driveErr > 0){
      if(lox == 'f'){
        rearQtr.read();
        int correction = pid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 50);
      }else{
        frontQtr.read();
        int correction = pid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, speed, speed + 50);
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

void autoLinePositionBox(int frontGap, char lox, boolean limitSpeed, int baseSpeed = 60){
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
    
    int speed = baseSpeed + (2 * abs(driveErr));
    if(speed > baseSpeed + 10 && limitSpeed){speed = baseSpeed + 10;}
    if(driveErr > 0){
      if(lox == 'f'){
        rearQtr.read();
        int correction = boxPid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 80);
      }else{
        frontQtr.read();
        int correction = boxPid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, speed, speed + 80);
      }
      
    }else if(driveErr < 0){
      if(lox == 'f'){
        frontQtr.read();
        int correction = boxPid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, speed, speed + 80);
      }else{
        rearQtr.read();
        int correction = boxPid(rearQtr.error, false);
        driver.applyLinePid(correction, false, speed, speed + 80);
      }
      
    }else{
      break;
    }
  }

}

void goStraight(boolean frwrd, int baseSpeed = 80){
  
  if(frwrd){
    int err = int(rightEncoder * 0.96) - int(leftEncoder * 1);
    int correction = encoderPid(err);
    int speed = baseSpeed;
    driver.applyEncoderPid(correction, speed);
  }else{
    int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
    int correction = encoderPid(err);
    int speed = -baseSpeed;
    driver.applyEncoderPid(correction, speed);
  }
  

}

void pushForward(int distance, boolean frwrd){
  initEncoder();
  while(leftEncoder <= distance || rightEncoder <= distance){
    if(frwrd){
      int err = int(rightEncoder * 0.96) - int(leftEncoder * 1);
      int correction = encoderPid(err);
      int speed = baseSpeed;
      driver.applyEncoderPid(correction, speed);
    }else{
      int err = int(rightEncoder * 1) - int(leftEncoder * 0.96);
      int correction = encoderPid(err);
      int speed = -baseSpeed;
      driver.applyEncoderPid(correction, speed);
    }
    printEncoder();
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

void oneWheelTurn(char dir, int distance){
  initEncoder();
  if(dir == 'r'){
    leftCount += distance;
    while(leftEncoder <= leftCount){
      printEncoder();
      driver.forward(115,0);
    }
  }else if(dir == 'l'){
    rightCount += distance;
    while(rightEncoder <= rightCount){
      printEncoder();
      driver.forward(0,115);
    }
  }
  driver.stop(100);
}

void turnLeft90(){
  initEncoder();
  leftCount += 90;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnLeft(turnSpeed, turnSpeed);
  }
  driver.stop(100);
}

void turnRight90(){
  initEncoder();
  leftCount += 90;
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
  leftCount += 420;
  rightCount = leftCount;
  while(leftEncoder < leftCount || rightEncoder < rightCount){
    printEncoder();
    driver.turnLeft(turnSpeed, turnSpeed - 20);
  }
  driver.stop();
}

void wallFollow(int time, int startTime, bool frwrd){
  while(int(millis() - startTime) <= time){
    int err = 6 - wallSonic.readCenti();
    int correction = wallPid(err);
    driver.applyWallPid(correction * -1, frwrd);
  }
}

bool checkMiddleRadarPos(){
  turnRadar(70,500);
  int reading = readRadarLox();
  if(reading < 100 && reading != -1){
    return true;
  }else{
    return false;
  }
}

// void trashYard(){
//   attachWrist();
//   writeWrist(45);

//   bool wallFound = false;
//   int radarState = 0; //1->2->3 ===> 20->55->90

//   centerRadar();
//   delay(100);

//   breakForward();

//   while(true){

//     int reading = readFrontLox();
//     if(reading < 150 && reading != -1){
//       driver.stop(100);
//       radarState = 0;
//       break;
//     }

//     turnRadar(50,0);
//     int startTime = millis();
//     wallFollow(60, startTime, true);
//     reading = readRadarLox();
//     if(reading < 150 && reading != -1){
//       driver.stop(100);
//       radarState = 1;
//       if(checkMiddleRadarPos()){
//         radarState = 2;
//       }
//       break;
//     }
//     // Serial.print(radarLox.reading);
//     // Serial.print("  ");

//     turnRadar(70,0);
//     startTime = millis();
//     wallFollow(60, startTime, true);
//     reading = readRadarLox();
//     if(reading < 150 && reading != -1){
//       driver.stop(100);
//       radarState = 2;
//       break;
//     }
//     // Serial.print(radarLox.reading);
//     // Serial.print("  ");

//     turnRadar(90,0);
//     startTime = millis();
//     wallFollow(60, startTime, true);
//     reading = readRadarLox();
//     if(reading < 150 && reading != -1){
//       driver.stop(100);
//       radarState = 3;
//       if(checkMiddleRadarPos()){
//         radarState = 2;
//       }
//       break;
//     }

//   }
  
//   turnRadar(90,75);

//   int reading = readRadarLox();

//   if(reading < 200 && reading != -1){
//     reading = readFrontLox();
//     if(reading < 150 && reading != -1){
//       wallFound = true;
//       light(2, 'r');
//     }
//   }

//   if(wallFound){
//     autoPosition(12, 'f');

//     turnRadar(160, 1000);

//     bool blockingTower = false;
//     initEncoder();
//     while(leftEncoder < 185 || rightEncoder < 185){
//       goStraight(false, 70);

//       int reading = readRadarLox();
//       Serial.println(reading);
//       if(reading < 250 && reading != -1){
//         blockingTower = true;
//         pushForward(380 - leftEncoder, false);
//         oneWheelTurn('l', leftOneWheelSpeed);
//         pushForward(325, true);
//         oneWheelTurn('r', rightOneWheelSpeed);
//         autoPosition(12, 'f');
//         oneWheelTurn('l', leftOneWheelSpeed);

//         break;
//       }
//     }

//     if(!blockingTower){
//       autoPosition(12, 'f');
//       oneWheelTurn('l', leftOneWheelSpeed);

//     }
    
//   }else{
//     if(radarState == 0){
//       autoPosition(15, 'f');
//     }else if(radarState == 1){
//       turnRadar(50, 75);
//       autoPosition(15, 'r');
//     }
//     else if(radarState == 2){
//       turnRadar(70, 75);
//       autoPosition(15, 'r');
//     }else{
//       turnRadar(90, 75);
//       autoPosition(15, 'r');
//     }

//     oneWheelTurn('l', leftOneWheelSpeed);

//     if(radarState == 0){ 
//       oneWheelTurn('r', rightOneWheelSpeed);
//     }else if(radarState == 1){
//       breakForward();
//       pushForward(50, true);
//       driver.stop(100);
//       oneWheelTurn('r', rightOneWheelSpeed);
    
//     }else if(radarState == 2){
//       breakForward();
//       pushForward(100, true);
//       driver.stop(100);
//       oneWheelTurn('r', rightOneWheelSpeed);
      
//     }else{
//       breakForward();
//       pushForward(150, true);
//       driver.stop(100);
//       oneWheelTurn('r', rightOneWheelSpeed);
      
//     }   

//     if(readFrontLox() > 400){
//       pushForward(230, true);
//       driver.stop(100); 

//       if(radarState == 0){ 
//         oneWheelTurn('r', rightOneWheelSpeed);

//       }else if(radarState == 1){
//         oneWheelTurn('r', rightOneWheelSpeed);
//         breakForward();
//         autoPosition(12, 'f');
//         driver.stop(100);
      
//       }else if(radarState == 2){
//         oneWheelTurn('r', rightOneWheelSpeed);
//         breakForward();
//         autoPosition(12, 'f');
//         driver.stop(100);

//       }else{
//         oneWheelTurn('r', rightOneWheelSpeed);
//         breakForward();
//         autoPosition(12, 'f');
//         driver.stop(100);
        
//       }   

//       oneWheelTurn('l', leftOneWheelSpeed);
//     }else{
//       autoPosition(12, 'f');
//       oneWheelTurn('l', leftOneWheelSpeed);

//     }

//   }

// }


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
      while(rightEncoder <= leftCount || leftEncoder <= rightCount){
        
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
      
      frontQtr.read();
      char newPattern = frontQtr.pattern;

      if(newPattern == 'T'){
        break;
      }

      if(checkBoxes){
        int reading = readFrontLox();
        if(reading < 500 && reading != -1){
          frontBox = true;
        }
      }

      if(newPattern == 1 || pattern == 'T'){
        if(turn == 'r'){
          driver.stop(100);
          beep();
          turnRightTillMiddle(true);
          driver.stop(250); 
        }else if(turn == 'l'){
          driver.stop(100);
          beep();
          turnLeftTillMiddle(true);
          driver.stop(250); 
        }else{
          if(checkBoxes){
            if(rightBox){
              driver.stop(100);
              beep();
              turnRightTillMiddle(true);
              driver.stop(250); 
            }
            else if(frontBox){

            }else{
              driver.stop(100);
              beep();
              turnLeftTillMiddle(true);
              driver.stop(250);  

            }
          }
        }
        break;
      }else{
        if(pattern == 'L'){
          driver.stop(100);
          beep();
          turnLeftTillMiddle(true);
          driver.stop(250);  

        }else if(pattern == 'R'){
          driver.stop(100);
          beep();
          turnRightTillMiddle(true);
          driver.stop(250);  

        }else{
          driver.stop(100);
          beep();
          turnBack();
          driver.stop(250);  

        }
        break;
      }

    }
  }

}

void lineFollowBackward(char turn = 'n', bool front = false){
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
      
      rearQtr.read();
      char newPattern = rearQtr.pattern;

      if(newPattern == 'T'){
        break;
      }

      if(newPattern == 1 || pattern == 'T'){
        if(turn == 'r'){
          driver.stop(100);
          beep();
          turnRightTillMiddle(front);
          driver.stop(250);
        }else if(turn == 'l'){
          driver.stop(100);
          beep();
          turnLeftTillMiddle(front);
          driver.stop(250);
        }else if(turn == 'u'){
          driver.stop(100);
          beep();
          turnBack();
          driver.stop(250);
        }else{

        }
        break;
      }else{
        if(pattern == 'L'){
          driver.stop(100);
          beep();
          turnLeftTillMiddle(front);
          driver.stop(250);
        }else if(pattern == 'R'){
          driver.stop(100);
          beep();
          turnRightTillMiddle(front);
          driver.stop(250);
        }else{
          driver.stop(100);
          beep();
          turnBack();
          driver.stop(250);
        }
        break;
      }

    }
  }
    
}

bool checkTower(int dataPast){
  if(abs(readRadarLox() - dataPast) > 80){
    return true;
  }else{
    return false;
  }
}

void trashYard(){
  attachWrist();
  writeWrist(100);
  delay(500);

  int dataPast = 0;
  turnRadar(0, 100);

  for(int i = 0;i < 180;i++){
    turnRadar(i, 10);
    checkTower(dataPast);
    dataPast = readRadarLox();
  }

  pushForward(200, true);
  turnLeft90();

  for(int i = 0;i < 180;i++){
    turnRadar(i, 10);
    if(checkTower(dataPast)){
      gemsB = 10;
    }
    dataPast = readRadarLox();
  }

  railUp(1500);

  turnRadar(0, 100);

  for(int i = 0;i < 180;i++){
    turnRadar(i, 10);
    if(checkTower(dataPast)){
      gemsB = 20;
    }
    dataPast = readRadarLox();
  }

  railUp(2500);

  turnRadar(0, 100);

  for(int i = 0;i < 180;i++){
    turnRadar(i, 10);
    if(checkTower(dataPast)){
      gemsB = 30;
    }
    dataPast = readRadarLox();
  }

  railDown(3000);
  positionArm();

  if(gemB == 10){light(1, 'r');}
  else if(gemB == 20){light(1, 'g');}
  else if(gemB == 30){light(1, 'b');}

  turnRight90();
  pushForward(200, false);

  lineFollowBackward('r', true);


}

void wall(){

  lineFollowBackward();

  while(readBackLox() > 150){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false);
  }

  autoLinePosition(4, 'b', false);
  driver.stop(100);

  bool green = checkGreenWall();
  wallColour = green == true ? 'g' : 'b';

  light(1, wallColour);
  delay(100);
  turnOff1Leds();

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

char getJunctionColour(){
  if(checkGreenColourJunction()){
    return 'g';
  }else{
    return 'b';
  }
}

void goToCircle(char colour){
  int pushDistance = 180;

  breakForward();
  pushForward(pushDistance, true);

  beep();

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
  beep();

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

  int cylCount = 0;

  while(leftEncoder < 800){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
    if(leftEncoder > 400 && leftEncoder < 600){
      readings[readCount] = backSonic.readCenti();
      if(readings[readCount] < 30){cylCount++;}
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

  Serial.print(mean);
  Serial.print("  ");
  Serial.print(cylCount);
  Serial.print("  ");
  Serial.print(std);
  Serial.println();

  if(mean < 30.0 && std < 2.5){
    light(2, 'g');
    gemsA = 10;
  }else{
    light(2, 'b');
    gemsA = 20;
  }

  // if(cylCount > 5){
  //   light(2, 'g');
  // }else{
  //   light(2, 'b');
  // }

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
  beep();

  initEncoder();
  while(leftEncoder < 50 || rightEncoder < 50){
    driver.turnLeft(turnSpeed, turnSpeed);
  }
  turnLeftTillMiddle(true);
  driver.stop(100);
  lineFollowForward();
  
}

void goToSemicircle(char colour){
  pushForward(pushDistanceForward, true);
  driver.stop(100);
  beep();

  if(colour == wallColour){
    turnLeftTillMiddle(true);
  }else{
    turnRightTillMiddle(true);
  }
  driver.stop(100);
  
}

int reachCube(){

  int data = readFrontLox();
  while(data > 150 || data == -1){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
    data = readFrontLox();
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
      //breakForward();
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

  return count >= 1500;
}

void grabCube(){
  
  initEncoder();
  while(leftEncoder < 65 || rightEncoder < 65){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  }
  driver.stop(100);

  armDown(1000, true);
  delay(250);

  breakForward();
  initEncoder();
  while(true){
    goStraight(true);
    printEncoder();
    if(leftEncoder >= 10 || rightEncoder >= 10){
      attachGripper();
      for(int i = 110;i >= 50;i--){
          goStraight(true);
          printEncoder();
          gripper.write(i);
          delay(10);
      }
      break;
    }
  }
  driver.stop(100);
  spreadGripper();
  delay(250);
  pushForward(20, true);
  gripCube();
  driver.stop(250);

  armUp(1000);

  turnSpeed = 115;
  baseSpeed = 95;

  detachWrist();

}

void exitSemicircle(char orientation, bool face = false){
  if(!face){
    if(orientation == 'n'){
      lineFollowBackward();
    }else if(orientation == 'w'){
      lineFollowBackward('r' ,true);
    }else if(orientation == 'e'){
      lineFollowBackward('l', true);
    }
  }else{
    if(orientation == 'n'){
      lineFollowBackward('u');
    }else if(orientation == 'w'){
      lineFollowBackward('l' ,true);
    }else if(orientation == 'e'){
      lineFollowBackward('r', true);
    }
  }

}

void semicircle(bool face = false){
  initMetalDetector();
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
  initMetalDetector();
  metal = foundMetalBox(reachCube());
  if(metal){
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation, face);
    return;
  }else{
    if(orientation == 'w'){
      lineFollowBackward('r', true);
      initEncoder();
      while(leftEncoder < 40 || rightEncoder < 40){
        frontQtr.read();
        int correction = pid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
      }
      orientation = 'n';
      reachCube();
      driver.stop(100);
      grabCube();
      exitSemicircle(orientation, face);
      return;
    }
  }

  lineFollowBackward(dir, true);
  initEncoder();
  while(leftEncoder < 40 || rightEncoder < 40){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
  }
  if(dir == 'l' && orientation == 'e'){orientation = 'n';}
  else if(dir == 'l' && orientation == 'n'){orientation = 'w';}
  else if(dir == 'u'){orientation = 'w';}
  if(orientation == 'w'){
    reachCube();
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation, face);
    return;
  }
  initMetalDetector();
  metal = foundMetalBox(reachCube());
  if(metal){
    driver.stop(100);
    grabCube();
    exitSemicircle(orientation, face);
    return;
  }else{
    if(orientation == 'w'){
      lineFollowBackward('r', true);
      initEncoder();
      while(leftEncoder < 40 || rightEncoder < 40){
        frontQtr.read();
        int correction = pid(frontQtr.error, true);
        driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
      }
      orientation = 'n';
      reachCube();
      driver.stop(100);
      grabCube();
      exitSemicircle(orientation, face);
      return;
    }
  }

  dir = 'l';
  lineFollowBackward(dir, true);
  initEncoder();
  while(leftEncoder < 40 || rightEncoder < 40){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
  }
  orientation = 'w';
  reachCube();
  driver.stop(100);
  grabCube();
  exitSemicircle(orientation, face);
  return;
}

void goToPlace(){
  
  initEncoder();
  while(leftEncoder < 50 || rightEncoder < 50){
    goStraight(false, baseSpeed);
  }

  initEncoder();
  while(leftEncoder < 100 || rightEncoder < 100){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  }

  lineFollowBackward('r', true);

  initEncoder();
  while(leftEncoder < 60 || rightEncoder < 60){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  }

  driver.stop(250);

}

void placeCube(){

  int data = readFrontLox();
  while(data > 300 || data == -1){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
    data = readFrontLox();
  }

  autoLinePositionBox(24, 'f', true, 70);
  driver.stop(250);

  armDown(1000, false);
 
  spreadGripper();
  delay(250);

  turnSpeed = 110;
  baseSpeed = 80;

  // breakBackward();
  // initEncoder();
  // while(leftEncoder < 25 || rightEncoder < 25){
  //   rearQtr.read();
  //   int correction = pid(rearQtr.error, false);
  //   driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  // }
  // driver.stop(100);

  attachElbow();
  writeElbow(100);
  delay(250);

  armDownClose();

  initEncoder();
  while(leftEncoder < 35 || rightEncoder < 35){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
    printEncoder();
  }
  driver.stop(100);
  
  gripCube();
  // positionArm();
  // delay(250);

  unsigned long startTime = millis();
  while(millis() - startTime <= 1500){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, 75, 125);
  }

  driver.stop(100);

  spreadGripper();

  breakBackward();
  initEncoder();
  while(leftEncoder < 25 || rightEncoder < 25){
    rearQtr.read();
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false, baseSpeed, baseSpeed + 50);
  }
  driver.stop(100);

  attachWrist();
  writeWrist(65);
  delay(250);

  positionArm();
  delay(250);

  startTime = millis();
  while(millis() - startTime <= 700){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, 75, 125);
  }

  delay(250);

}

void passSquare(char dir, bool frwrd){
  if(frwrd){
    while(true){
      frontQtr.read();
      if(frontQtr.pattern == 'L' || frontQtr.pattern == 'R' || frontQtr.pattern == 'T'){
        break;
      }
      int correction = pid(frontQtr.error, true);
      driver.applyLinePid(correction * -1, true);
    }

    pushForward(pushDistanceForward + 70, frwrd);
    
    if(dir == 'l'){
      beep();
      turnLeftTillMiddle(frwrd);
    }else if(dir == 'r'){
      beep();
      turnRightTillMiddle(frwrd);
    }else{

    }
    driver.stop(250);
    
  }else{
    while(true){
      rearQtr.read();
      if(rearQtr.pattern == 'L' || rearQtr.pattern == 'R' || rearQtr.pattern == 'T'){
        break;
      }
      int correction = pid(rearQtr.error, false);
      driver.applyLinePid(correction, false);
    }

    pushForward(pushDistanceBackward + 70, frwrd);

    if(dir == 'l'){
      beep();
      turnLeftTillMiddle(!frwrd);
    }else if(dir == 'r'){
      beep();
      turnRightTillMiddle(!frwrd);
    }else{

    }
    driver.stop(250);

  }

}

void goToTrashYard(){
  passSquare('f', true);

  lineFollowForward();

  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 0){
      break;
    }
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
  }

  driver.stop(250);
  beep();

}

void goToBall(){
  int data = readFrontLox();
  while(data > 150 || data == -1){
    frontQtr.read();
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true, baseSpeed, baseSpeed + 50);
    data = readFrontLox();
  }
  driver.stop(100);

  autoLinePositionBox(7, 'f', true, 70);
  driver.stop(250);

  armBall();
}

void ball(){
  gripBall();
  getBall();
  delay(250);

  attachElbow();
  writeElbow(30);
  delay(250);

  attachWrist();
  writeWrist(145);
  delay(250);

  spreadGripper();

  attachWrist();
  writeWrist(100);
  delay(300);

  attachBallHolder();
  writeBallHolder(90);
  positionArm();
}

void goToShoot(int score){
  passSquare('f', false);

  while(true){
    rearQtr.read();
    if(rearQtr.pattern == 'T'){
      break;
    }
    int correction = pid(rearQtr.error, false);
    driver.applyLinePid(correction, false);
  }

  initEncoder();
  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 'T'){
      break;
    }
    goStraight(false);
  }
  driver.stop(100);
  beep();

  pushForward(20, true);

  int count = 0;
  if(score == 20 || score == 30){
    if(score == 20){count = 105;}
    else{count = 45;}
    initEncoder();
    while(leftEncoder < count || rightEncoder < count){
      printEncoder();
      driver.turnLeft(turnSpeed, turnSpeed);
    }
    driver.stop(100);

  }else if(score == 40 || score == 50){
    if(score == 50){count = 110;}
    else{count = 45;}
    initEncoder();
    while(leftEncoder < count || rightEncoder < count){
      printEncoder();
      driver.turnRight(turnSpeed, turnSpeed);
    }
    driver.stop(100);
  }

}

void shootBall(){
  positionGun();

  trig();
  delay(500);

  ballHolderDown();

  shoot();
}

void jumpForward(){
  delay(1000);
  initEncoder();
  while(leftEncoder <= 100 || rightEncoder <= 100){
    int err = int(rightEncoder * 0.96) - int(leftEncoder * 1);
    int correction = encoderPid(err);
    driver.applyEncoderPid(correction, -80);
  }
}

void botStart(){
  
  while(true){
    int readingB = readBackLox();
    int readingF = readFrontLox();
    int readingR = readRadarLox();
    if(readingB < 50 && readingB != -1){
      gemB = 20;
      jumpForward();
      break;
    }else if(readingF < 50 && readingF != -1){
      gemB = 10;
      jumpForward();
      break;
    }else if(readingR < 50 && readingR != -1){
      gemB = 30;
      jumpForward();
      break;
    }
  }

}

void botEnd(){
  lineFollowBackward('r');

  while(true){
    frontQtr.read();
    if(frontQtr.pattern == 'T'){
      break;
    }
    int correction = pid(frontQtr.error, true);
    driver.applyLinePid(correction * -1, true);
  }
  
  pushForward(200, true);
  beep();
  driver.stop(99999);

}

void calibrate(){
  initEncoder();
  while(leftEncoder < 410 || rightEncoder < 410){
    driver.turnLeft(110, 110);
    frontQtr.calibrate(1000);
    rearQtr.calibrate(1000);
  }

  initEncoder();
  while(leftEncoder < 410 || rightEncoder < 410){
    driver.turnRight(110, 110);
    frontQtr.calibrate(1000);
    rearQtr.calibrate(1000);
  }

  driver.stop(5000);

  beep();
  
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

  initElbow(0,180);
  initWrist(0,180);
  initGripper(0,180);
  initLED();

  driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

  initLoxes();

  // frontQtr.calibrate(1000);
  // rearQtr.calibrate(1000);

}

void botLoop(){

  botStart();

  wall();

  reachColourJunction('w');
  driver.stop(100);
  char colour = getJunctionColour();

  goToCircle(colour);
  circle();
  
  reachColourJunction(wallColour);

  goToSemicircle(colour);
  semicircle(false);

  reachColourJunction('r');

  goToPlace();
  placeCube();
  lineFollowBackward('r', true);

  goToTrashYard();
  trashYard();

  semicircle(true);

  lineFollowForward();
  placeCube();
  lineFollowBackward('l', true);

  passSquare('l', true);

  goToBall();
  ball();

  goToShoot(gemsA + gemB);
  shootBall();

  delay(1000);
  beep();
  delay(200);
  beep();
  delay(200);
  beep();

  delay(99999999);

}

void setup(){
  delay(50);
  botSetup();

  positionArm();
  positionGun();

  calibrate();

  // frontQtr.calibrate(1000);
  // rearQtr.calibrate(1000);

}

void loop(){
  botLoop();

}