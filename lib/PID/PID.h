#define SLRC24_PID_H
#ifndef SLRC24_PID_H

#endif //SLRC24_PID_H

const double fP = 0.1;
const double fI = 0;
const double fD = 0.6;

const double bP = 0.05;
const double bI = 0;
const double bD = 0.4;

int totalError = 0;
int prevError = 0;

int pid(int error, bool frwrd = true) {
    totalError += error;

    double p;
    double i;
    double d;

    if(frwrd){
        p = error * fP;
        i = totalError * fI;
        d = (error - prevError) * fD;
    }else{
        p = error * bP;
        i = totalError * bI;
        d = (error - prevError) * bD;
    }

    prevError = error;

    int correction = (int)(p + i + d);

    return correction;
}

const double eP = 5;
const double eD = 2;

int encoderPid(int error){
    double p = error * eP;
    double d = (error - prevError) *eD;

    prevError = error;

    int correction = (int)(p + d);

    return correction;
}
