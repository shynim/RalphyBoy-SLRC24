#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 44
#define SHT_LOX2 15
#define SHT_LOX3 16

// objects for the vl53l0x
Adafruit_VL53L0X rLox = Adafruit_VL53L0X();
Adafruit_VL53L0X fLox = Adafruit_VL53L0X();
Adafruit_VL53L0X bLox = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if(!rLox.begin(LOX1_ADDRESS)) {
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!fLox.begin(LOX2_ADDRESS)) {
    while(1);
  }

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!bLox.begin(LOX3_ADDRESS)) {
    while(1);
  }
}

int readFrontLox(){
  fLox.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  if(measure2.RangeStatus != 4) {
    int data = measure2.RangeMilliMeter;
    if(data < 500){
      return data;
    }else{
      return -1;
    }
  } else {
    return -1;
  }
  
}

int readRadarLox(){
  rLox.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!

  if(measure1.RangeStatus != 4) {
    int data = measure1.RangeMilliMeter;
    if(data < 500){
      return data;
    }else{
      return -1;
    }
  } else {
    return -1;
  }

}

int readBackLox(){
  bLox.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

  if(measure3.RangeStatus != 4) {
    int data = measure3.RangeMilliMeter;
    if(data < 500){
      return data;
    }else{
      return -1;
    }
  } else {
    return -1;
  }
}

void read_dual_sensors() {
  
  rLox.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  fLox.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print("  ");
}

void initLoxes() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW); 
  digitalWrite(SHT_LOX3, LOW); 
  
  setID();

  rLox.setMeasurementTimingBudgetMicroSeconds(20000);
  fLox.setMeasurementTimingBudgetMicroSeconds(20000);
  bLox.setMeasurementTimingBudgetMicroSeconds(20000);
 
}
