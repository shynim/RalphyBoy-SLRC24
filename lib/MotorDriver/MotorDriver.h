#ifndef SLRC24_MOTORDRIVER_H
#define SLRC24_MOTORDRIVER_H

class MotorDriver{
    public:
        void init(int* leftPins,int *rightPins);

        void forward(int speed);
        
        void backward(int speed);

        void turnLeft(int leftSpeed, int rightSpeed);
        
        void turnRight(int leftSpeed, int rightSpeed);

        void reverseRight(int speed);

        void reverseLeft(int speed);

        void forward(int leftSpeed, int rightSpeed);

        void backward(int leftSpeed, int rightSpeed);

        void stop(int del = 1000);

        void brake();

        void applyLinePid(int correction, bool frwrd, int base = 90, int max = 140);

        void applyEncoderPid(int correction, int base = 80);

        void applyWallPid(int correction);

        int applyDrivePid(int correction);

        
    private:
        int leftPWM;
        int rightPWM;
        int leftDirection[2];
        int rightDirection[2];

};

#endif //SLRC24_MOTORDRIVER_H