#define SLRC24_PID_H
#ifndef SLRC24_PID_H

#endif //SLRC24_PID_H

const double fP = 0.015;
const double fI = 0;
const double fD = 0.052;

const double bP = 0.035;
const double bI = 0;
const double bD = 0.5;

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

const double fbP = 0.025;
const double fbI = 0;
const double fbD = 0.05;

const double bbP = 0.035;
const double bbI = 0;
const double bbD = 0.25;

int boxPid(int error, bool frwrd = true) {
    totalError += error;

    double p;
    double i;
    double d;

    if(frwrd){
        p = error * fbP;
        i = totalError * fbI;
        d = (error - prevError) * fbD;
    }else{
        p = error * bbP;
        i = totalError * bbI;
        d = (error - prevError) * bbD;
    }

    prevError = error;

    int correction = (int)(p + i + d);

    return correction;
}

const double eP = 0.8;
const double eD = 2;

int encoderPid(int error){
    double p = error * eP;
    double d = (error - prevError) *eD;

    prevError = error;

    int correction = (int)(p + d);

    return correction;
}

const double wP = 8.5;
const double wD = 30;
const double wI = 0.008;

int wallPid(int error){
    totalError += error;

    double p = error * wP;
    double i = totalError * wI;
    double d = (error - prevError) *wD;

    prevError = error;

    int correction = (int)(p + i + d);

    return correction;
}
