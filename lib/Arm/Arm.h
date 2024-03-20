#include <Servo.h>
#include <Arduino.h>

Servo elbow;
Servo wrist;
Servo gripper;

const int elbowPin = 2;
const int wristPin = 3;
const int gripperPin = 4;

int elbowRange[2];
int wristRange[2];
int gripperRange[2];

void initElbow(int minAng, int maxAng){
    elbowRange[0] = minAng;
    elbowRange[1] = maxAng;
}

void initWrist(int minAng, int maxAng){
    wristRange[0] = minAng;
    wristRange[1] = maxAng;
}

void initGripper(int minAng, int maxAng){
    gripperRange[0] = minAng;
    gripperRange[1] = maxAng;
}

void attachElbow(){
    elbow.attach(elbowPin);
}

void attachWrist(){
    wrist.attach(wristPin);
}

void attachGripper(){
    gripper.attach(gripperPin);
}

void detachElbow(){
    elbow.detach();
}

void detachWrist(){
    wrist.detach();
}

void detachGripper(){
    gripper.detach();
}

void writeElbow(int ang){
    if(ang > elbowRange[1]) ang = elbowRange[1];
    if(ang < elbowRange[0]) ang = elbowRange[0];

    elbow.write(ang);
}

void writeWrist(int ang){
    if(ang > wristRange[1]) ang = wristRange[1];
    if(ang < wristRange[0]) ang = wristRange[0];

    wrist.write(ang);
}

void writeGripper(int ang){
    if(ang > gripperRange[1]) ang = gripperRange[1];
    if(ang < gripperRange[0]) ang = gripperRange[0];

    gripper.write(ang);
}

void gripCube(){
    attachGripper();
    gripper.write(4);
    delay(100);
}

void spreadGripper(){
    attachGripper();
    gripper.write(55);
    delay(80);
    detachGripper();

}

int elbowStartPos = 40;
int wristStartPos = 5;

int elbowDownPos = 155;
int wristStraightPos = 115;

void positionArm(){
    attachGripper();
    writeGripper(0);
    delay(500);
    detachGripper();

    attachElbow();
    writeElbow(elbowStartPos);
    delay(500);
    detachElbow();

    attachWrist();
    writeWrist(wristStartPos);
    delay(500);
    detachWrist();

}

void armDown(int time, bool spread){
    attachElbow();
    attachWrist();
    attachGripper();

    int elbowDelay = int(time / (elbowDownPos - elbowStartPos));
    int wristDelay = int(time / (wristStraightPos - wristStartPos));

    int elbowAng = elbowStartPos;
    int wristAng = wristStartPos;

    int startTimeElbow = millis();
    int startTimeWrist = millis();

    while(elbowAng < elbowDownPos || wristAng < wristStraightPos){
        if((millis() - startTimeElbow) >= elbowDelay && elbowAng < elbowDownPos){
            writeElbow(++elbowAng);
            startTimeElbow = millis();
        }
        if((millis() - startTimeWrist) >= wristDelay && wristAng < wristStraightPos){
            writeWrist(++wristAng);
            startTimeWrist = millis();
        }
        if(elbowAng == 120 && spread){
            spreadGripper();
        }

    }

}

void armUp(int time){
    attachElbow();
    attachWrist();
    attachGripper();

    int elbowDelay = int(time / (elbowDownPos - elbowStartPos));
    int wristDelay = int(time / (wristStraightPos - wristStartPos));

    int elbowAng = elbowDownPos;
    int wristAng = wristStraightPos;

    int startTimeElbow = millis();
    int startTimeWrist = millis();

    while(elbowAng > elbowStartPos || wristAng > wristStartPos){
        if((millis() - startTimeElbow) >= elbowDelay && elbowAng > elbowStartPos){
            writeElbow(--elbowAng);
            startTimeElbow = millis();
        }
        if((millis() - startTimeWrist) >= wristDelay && wristAng > wristStartPos){
            writeWrist(--wristAng);
            startTimeWrist = millis();
        }

    }

}

void armDownClose(){
    attachElbow();
    attachWrist();
    attachGripper();

    writeWrist(45);
    delay(500);
    writeElbow(83);
    delay(500);
    spreadGripper();
    delay(500);
    writeWrist(5);
    delay(500);

}
