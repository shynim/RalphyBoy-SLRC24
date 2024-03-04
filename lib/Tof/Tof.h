#include <VL53L0X.h>

#ifndef SLRC24_TOF_H
#define SLRC24_TOF_H

class Tof{

public:
    int reading = -1;

    Tof(int *settings);

    void read();
    void init();
    void shut();

private:
    VL53L0X sensor;

    int xshut;
    bool longRange = false;
    bool highAccuracy = false;

};
#endif //SLRC24_TOF_H


