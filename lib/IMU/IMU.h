#pragma once

#include <string>

#include <Arduino.h>

class IMU
{
public:
    union IMU_READINGS
    {
        float data[3];
        byte raw[sizeof(float)*3];
    };
    struct IMU_VALUES
    {
        IMU_READINGS gyro;
        IMU_READINGS angles;
        IMU_READINGS accelerations;
        IMU_READINGS velocities;
    };
    union IMU_VALUES_RAW
    {
        IMU_VALUES imu_values;
        float values[4*3];
    };
    IMU(){};
    void init(SemaphoreHandle_t i2cSemaphore);
    double readYaw();
    void getPublishData(IMU_VALUES &values);
    bool isEnabled() { return enabled; };

private:
    static constexpr const char *tag = "IMU";
    SemaphoreHandle_t i2cSemaphore;
    bool enabled = false;
};