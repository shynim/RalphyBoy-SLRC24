#include <Arduino.h>

void turnOffAllLeds(){
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
    digitalWrite(A14, HIGH);
    digitalWrite(17, HIGH);

}

void initLED(){
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A14, OUTPUT);
    pinMode(17, OUTPUT);

    turnOffAllLeds();
}

void light(int led, char colour){
    turnOffAllLeds();
    if(led == 1){
        if(colour == 'r'){
            digitalWrite(A1, LOW);
        }else if(colour == 'g'){
            digitalWrite(A14, LOW);

        }else if(colour == 'b'){
            digitalWrite(17, LOW);

        }

    }else if(led == 2){
        if(colour == 'r'){
            digitalWrite(A3, LOW);

        }else if(colour == 'g'){
            digitalWrite(A4, LOW);

        }else if(colour == 'b'){
            digitalWrite(A2, LOW);

        }

    }
}