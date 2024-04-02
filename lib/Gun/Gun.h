#include <Servo.h>
#include <Arduino.h>

Servo ballHolder;
Servo trigger;
Servo triggerSwitch;

const int ballHolderPin = 10;
const int triggerPin = 9;
const int triggerSwitchPin = 8;

void attachBallHolder(){
    ballHolder.attach(ballHolderPin);
}

void attachTrigger(){
    trigger.attach(triggerPin);
}

void attachTriggerSwitch(){
    triggerSwitch.attach(triggerSwitchPin);
}

void detachBallHolder(){
    ballHolder.detach();
}

void detachTrigger(){
    trigger.detach();
}

void detachTriggerSwitch(){
    triggerSwitch.detach();
}

void writeTrigger(int ang){
    trigger.write(ang);
}

void writeTriggerSwitch(int ang){
    triggerSwitch.write(ang);
}

void writeBallHolder(int ang){
    ballHolder.write(ang);
}

void getBall(){
    attachBallHolder();
    writeBallHolder(150);
}

void trig(){
    attachTrigger();
    writeTrigger(80);
    delay(1000);

}

void shoot(){
    attachTriggerSwitch();
    writeTriggerSwitch(130);
    delay(500);
    writeTriggerSwitch(170);

}

void ballHolderDown(){
    attachBallHolder();
    writeBallHolder(65);
    delay(500);
}

void ballHolderUp(){
    attachBallHolder();
    writeBallHolder(100);
    delay(500);
}

void positionGun(){
    attachTriggerSwitch();
    attachTrigger();

    writeTriggerSwitch(170);
    writeTrigger(150);
    ballHolderDown();

    detachBallHolder();
    detachTrigger();
    detachTriggerSwitch();

}