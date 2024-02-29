#include <MotorDriver.h>
#include <Arduino.h>

const int baseSpeed = 80;
const int maxSpeed = 110;

void MotorDriver::init(int* leftPins,int* rightPins){
    leftPWM = leftPins[0];
    leftDirection[0] = leftPins[1];
    leftDirection[1] = leftPins[2];

    rightPWM = rightPins[0];
    rightDirection[0] = rightPins[1];
    rightDirection[1] = rightPins[2];

    pinMode(leftPWM, OUTPUT);
    pinMode(rightPWM, OUTPUT);
    pinMode(leftDirection[0], OUTPUT);
    pinMode(leftDirection[1], OUTPUT);
    pinMode(rightDirection[0], OUTPUT);
    pinMode(rightDirection[1],OUTPUT);
}

void MotorDriver::forward(int speed) {
    digitalWrite(leftDirection[0], HIGH);
    digitalWrite(leftDirection[1], LOW);
    digitalWrite(rightDirection[0], HIGH);
    digitalWrite(rightDirection[1], LOW);

    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

void MotorDriver::stop() {
    digitalWrite(leftDirection[0], HIGH);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], HIGH);
    digitalWrite(rightDirection[1], HIGH);

    analogWrite(leftPWM, 255);
    analogWrite(rightPWM, 255);
    delay(1000);
}

void MotorDriver::backward(int speed) {
    digitalWrite(leftDirection[0], LOW);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], LOW);
    digitalWrite(rightDirection[1], HIGH);

    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

void MotorDriver::turnLeft(int leftSpeed, int rightSpeed) {
    digitalWrite(leftDirection[0], LOW);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], HIGH);
    digitalWrite(rightDirection[1], LOW);

    analogWrite(leftPWM, leftSpeed);
    analogWrite(rightPWM, rightSpeed);
}

void MotorDriver::turnRight(int leftSpeed, int rightSpeed) {
    digitalWrite(leftDirection[0], HIGH);
    digitalWrite(leftDirection[1], LOW);
    digitalWrite(rightDirection[0], LOW);
    digitalWrite(rightDirection[1], HIGH);

    analogWrite(leftPWM, leftSpeed);
    analogWrite(rightPWM, rightSpeed);
}
void MotorDriver::reverseRight(int speed) {
    digitalWrite(leftDirection[0], LOW);
    digitalWrite(leftDirection[1], LOW);
    digitalWrite(rightDirection[0], LOW);
    digitalWrite(rightDirection[1], HIGH);

    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

void MotorDriver::reverseLeft(int speed) {
    digitalWrite(leftDirection[0], LOW);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], LOW);
    digitalWrite(rightDirection[1], LOW);

    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

void MotorDriver::forward(int leftSpeed, int rightSpeed) {
    digitalWrite(leftDirection[0], HIGH);
    digitalWrite(leftDirection[1], LOW);
    digitalWrite(rightDirection[0], HIGH);
    digitalWrite(rightDirection[1], LOW);

    analogWrite(leftPWM, leftSpeed);
    analogWrite(rightPWM, rightSpeed);
}

void MotorDriver::backward(int leftSpeed, int rightSpeed) {
    digitalWrite(leftDirection[0], LOW);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], LOW);
    digitalWrite(rightDirection[1], HIGH);

    analogWrite(leftPWM, leftSpeed);
    analogWrite(rightPWM, rightSpeed);
}

void MotorDriver::brake(){
    digitalWrite(leftDirection[0], HIGH);
    digitalWrite(leftDirection[1], HIGH);
    digitalWrite(rightDirection[0], HIGH);
    digitalWrite(rightDirection[1], HIGH);

    delay(100);
}

void MotorDriver::applyLinePid(int correction, bool frwrd) {
    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    if (leftSpeed < 0) {
        leftSpeed = 0;
    }

    if (rightSpeed < 0) {
        rightSpeed = 0;
    }

    if (leftSpeed >= maxSpeed) {
        leftSpeed = maxSpeed;
    }

    if (rightSpeed >= maxSpeed) {
        rightSpeed = maxSpeed;
    }

    if(frwrd) forward(leftSpeed, rightSpeed);
    else backward(leftSpeed, rightSpeed);
}