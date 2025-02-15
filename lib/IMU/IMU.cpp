#include "IMU.h"

#include <JY901DRV.h>

#include <Constants.h>

void IMU::init(SemaphoreHandle_t i2cSemaphore)
{
    this->i2cSemaphore = i2cSemaphore;
    if (enabled)
    {
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
        {
            // Calibration
            JY901.set9AxisAlgo();
            JY901.calibrateAll();
            JY901.turnLED(0);
            JY901.saveConf(0);
            xSemaphoreGive(i2cSemaphore);
        }
        else
            esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");
    }
    else
    {
        esp_log_write(ESP_LOG_WARN, tag, "IMU disabled");
    }
}

double IMU::readYaw()
{
    double yaw = 0.0;
    if (enabled)
    {
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
        {
            JY901.getGyro();
            yaw = JY901.data.gyro.z * Constants::RAD_S;
            xSemaphoreGive(i2cSemaphore);
        }
        else
            esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");
    }
    return yaw;
}

void IMU::getPublishData(IMU_VALUES &values)
{
    if (enabled)
    {
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
        {
            JY901.getGyro();
            JY901.getAngles();
            JY901.getAcc();
            values.gyro.data[0] = JY901.data.gyro.x * Constants::RAD_S;
            values.gyro.data[1] = JY901.data.gyro.y * Constants::RAD_S;
            values.gyro.data[2] = JY901.data.gyro.z * Constants::RAD_S;
            values.angles.data[0] = JY901.data.angle.roll * Constants::RAD;
            values.angles.data[1] = JY901.data.angle.pitch * Constants::RAD;
            values.angles.data[2] = JY901.data.angle.yaw * Constants::RAD;
            values.accelerations.data[0] = JY901.data.acc.x * Constants::Gs;
            values.accelerations.data[1] = JY901.data.acc.y * Constants::Gs;
            values.accelerations.data[2] = JY901.data.acc.z * Constants::Gs;
            xSemaphoreGive(i2cSemaphore);
        }
        else
            esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");
    }
}
