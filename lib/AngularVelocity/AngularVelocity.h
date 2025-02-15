#pragma once

#include <IMU.h>

class AngularVelocity
{
public:
    AngularVelocity(){};
    void init(int numReadings, int updateInterval, IMU* imu);
    double getAverage();
    void checkUpdate();
    void reset();
private:
    IMU* imu;
    int maxReadings;
    double *readings = nullptr;
    unsigned long lastUpdate = 0;
    int total = 0;
    int current = 0;
    int updateInterval = 0;
};