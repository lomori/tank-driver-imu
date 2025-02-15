#include "AngularVelocity.h"

#include <Constants.h>

void AngularVelocity::init(int numReadings, int updateInterval, IMU *imu)
{
    this->imu = imu;
    maxReadings = numReadings;
    if (readings!=nullptr)
        delete readings;
    readings = (double *)malloc(sizeof(double)*numReadings);

    this-> updateInterval = updateInterval;
}

void AngularVelocity::reset()
{
    lastUpdate = 0;
    total = 0;
    current = 0;
}

double AngularVelocity::getAverage()
{
    int n = min(total, maxReadings);

    double sum = 0.0;
    for (int i=0;i<n;i++)
        sum+=readings[i];

    return sum / n;
}

void AngularVelocity::checkUpdate()
{
    unsigned long now = millis();

    if ((now - lastUpdate)>=updateInterval)
    {
        lastUpdate = now;
        readings[current++] = imu->readYaw();
        total++;
        if(current >= maxReadings)
            current = 0;
    }
}