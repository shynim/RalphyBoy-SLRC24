#include <Arduino.h>

int S0 = 51;
int S1 = 45;
int S2 = 53;
int S3 = 49;
int sensorOut = 47;

int redMin = 15; // Red minimum value
int redMax = 130; // Red maximum value
int greenMin = 15; // Green minimum value
int greenMax = 130; // Green maximum value
int blueMin = 15; // Blue minimum value
int blueMax = 110; // Blue maximum value
 
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
 
int redValue;
int greenValue;
int blueValue;

void initColour(){
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(A13, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);


}

int getRedPW() {

  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);

  redValue = map(PW, redMin,redMax,255,0);
  delay(100);
  //return PW;
  return redValue;

}

int getGreenPW() {
 
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);
  
  greenValue = map(PW, greenMin,greenMax,255,0);
  delay(100);
  //return PW;
  return greenValue;

}

int getBluePW() {
 
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);

  blueValue = map(PW, blueMin,blueMax,255,0);
  delay(100);
  //return PW;
  return blueValue;

 
}

char getColour(){
  
  getRedPW();
  getGreenPW();
  getBluePW();

  if(redValue > 150 && greenValue < 150 && blueValue < 150){
    return 'R';
  }else{
    return 'B';
  }
}

bool checkGreenWall(){
  getGreenPW();
  getBluePW();

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  if(greenValue > blueValue){
    return true;
  }else{
    return false;
  }

}

bool checkGreenColourJunction(){
  digitalWrite(S2, HIGH);
  delay(5);
  int blue = 0;
  int green = 0;
  for(int i = 0; i < 10; i++){
    blue += analogRead(A13);
  }
  blue /= 10;
  digitalWrite(S2, LOW);
  delay(5);
  digitalWrite(S3, HIGH);
  delay(5);
  for(int i = 0; i < 10; i++){
    green += analogRead(A13);
  }
  green /= 10;
  digitalWrite(S3, LOW);
  delay(5);

  if(green - blue > 80){
    return true;
  }else{
    return false;
  }

}