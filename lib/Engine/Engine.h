#pragma once

#include <Arduino.h>

#include <ESP32Encoder.h>

#include <atomic>

#include <PID.h>

class Engine
{
public:
    struct Calibration
    {
        const double *forward;
        const double *backward;
    };
    Engine(){};
    bool setPPS(double pps);
    double getPPS();
    double getActualPPS();
    const char *getTag() { return localTag; }
    void init(const char *tag, uint8_t pwmPin, uint8_t pwmChannel, uint8_t pin1, uint8_t pin2,
              uint8_t counterPinA, uint8_t counterPinB, Calibration *calibration, double bias);
    void controller();
    void stop();
    bool correctionEnabled = true;

private:
    //static constexpr const char *tag="Engine";
    const char *localTag;
    static const int FREQ = 120; // HZ
    static const int RESOLUTION = 8;
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pwmChannel;
    double bias;
    Calibration *calibration;
    PID *pid = nullptr; // PID algoritm handler
    PID::CtrlConfig pidConfig; // PID parameters
    volatile unsigned long lastUpdateMS = 0;
    volatile unsigned long previousCounter = 0;
    ESP32Encoder encoder;
    double pps;
    double actualPPS;
    unsigned int pps2pwm(double pps);
    bool accelerating;
};
