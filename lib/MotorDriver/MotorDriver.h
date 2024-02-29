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

        void stop();

        void brake();

        void applyLinePid(int correction, bool frwrd);

        
    private:
        int leftPWM;
        int rightPWM;
        int leftDirection[2];
        int rightDirection[2];

};

#endif //SLRC24_MOTORDRIVER_H