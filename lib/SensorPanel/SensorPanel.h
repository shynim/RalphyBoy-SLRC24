#include <QTRSensors.h>

#ifndef SLRC24_SENSORPANEL_H
#define SLRC24_SENSORPANEL_H

class SensorPanel {
public:
    char pattern = 0;
    uint16_t position = 0;
    int error = 0;
    uint16_t panelReading[8];
    uint16_t rawReadings[8];
    bool isMiddle;
    bool isEnd;
    bool junc;
    bool left;
    bool right;

    SensorPanel(uint8_t *sensorPins);

    bool calibrate(int seconds);

    void read();

private:
    QTRSensors qtr;
    const uint8_t SensorCount = 8;

    uint16_t readLine(uint16_t *sensorValues);
    void updatePattern();

};

#endif //SLRC24_SENSORPANEL_H

