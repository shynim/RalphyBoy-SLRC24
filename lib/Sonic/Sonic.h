#include <NewPing.h>

#ifndef SLRC24_SONIC_H
#define SLRC24_SONIC_H

class Sonic{
    public:
        Sonic(int trig, int echo, int maxDistance) : sonic(trig, echo, maxDistance){
            t = trig;
            e = echo;
        };

        int readMili();

        int readCenti();

    private:
        NewPing sonic;
        int t;
        int e;
        
};
  
#endif //SLRC24_SONIC_H


