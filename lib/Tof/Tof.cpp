#include <Tof.h>
#include <Wire.h>

Tof::Tof(int *settings){
    xshut = settings[0];
    longRange = settings[1];
    highAccuracy = settings[2];

}
void Tof::init(){
    digitalWrite(xshut, HIGH);

    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init()){
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {}
    }

    if(longRange == 1){
        sensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    }
    if(highAccuracy == 1){
        // increase timing budget to 200 ms
        sensor.setMeasurementTimingBudget(200000);
    }else{
        // reduce timing budget to 20 ms (default is about 33 ms)
        sensor.setMeasurementTimingBudget(20000);
    }

}

void Tof::shut(){
    digitalWrite(xshut, LOW);
}

void Tof::read(){
    int data = sensor.readRangeSingleMillimeters();

    if(data < 2200){
        //Serial.print(data);
        reading = data;
    }
    else{
        //Serial.print("Out of range");
        reading = -1;
        
    }

    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    
    //Serial.println();

}