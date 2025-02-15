#pragma once

#include <Arduino.h>

class Constants
{
public:
    static const int NUM_CALIBRATIONS = 10;
    static constexpr double __yValues[NUM_CALIBRATIONS] = {35, 40, 45, 50, 55, 60, 65, 245, 250, 255};

    static constexpr double xValuesLF[NUM_CALIBRATIONS] = {0.0000,5.5000,45.0000,69.0000,78.0000,87.5000,
                                                            100.5000,518.0000,530.0000,543.5000};


    static constexpr double xValuesLB[NUM_CALIBRATIONS] = {0.0000,52.0000,60.0000,73.0000,90.0000,
                                                            100.0000,112.5000,531.5000,544.0000,559.5000};


    static constexpr double xValuesRF[NUM_CALIBRATIONS] = {0.0000,7.9920,47.4525,72.4276,77.4226,87.8682,
                                                            101.3986,527.9720,538.9610,554.9451};

    static constexpr double xValuesRB[NUM_CALIBRATIONS] = {0.0000,48.9266,60.9391,67.4326,80.9191,92.3154,
                                                            104.8951,512.4875,522.9770,538.4615};

    static const unsigned int LINEAR_UPDATE_MS = 100;
    static const unsigned int ANGULAR_UPDATE_MS = 500;

    static constexpr double EPSLON = 0.0001;

    // static constexpr double M_PI = 3.14159265358979323846;
    static constexpr double DEG_S = 2000.0 / 32768.0; // °/s
    static constexpr double RAD_S = DEG_S * PI / 180.0;
    static constexpr double DEG = 180.0 / 32768.0; // °
    static constexpr double RAD = PI / 32768.0;

    static constexpr double Gs = 16.0 / 32768.0; // Gravity

    // Left engine
    static const int L_PIN1 = 25;
    static const int L_PIN2 = 33;
    static const int L_PWM = 32;
    static const int L_PWM_CHANNEL = 0;
    static const int L_COUNTER_PINA = 15;
    static const int L_COUNTER_PINB = 23;
    static constexpr double L_BIAS = 0.0;

    // Right engine
    static const int R_PIN1 = 27;
    static const int R_PIN2 = 14;
    static const int R_PWM = 12;
    static const int R_PWM_CHANNEL = 1;
    static const int R_COUNTER_PINA = 18;
    static const int R_COUNTER_PINB = 19;
    static constexpr double R_BIAS = 0.0;

    static const int ENABLE_ENGINES = 26;

    static constexpr double MM_PULSE = 0.567523243455062;
    static constexpr double Y_WHEELS_DISTANCE_M = 0.26;
    static constexpr double MAX_PPS = 500.0;
    // static constexpr double Y_WHEELS_DISTANCE_M_COMPUTED = 1.33017516317257;
    //static constexpr double MAX_RPM = 205.0;
    static constexpr double WHEEL_DIAMETER = 0.045;
    static constexpr unsigned long MAX_ULONG = 4294967295UL;

    static const unsigned int MAX_PWM = 255;
    static const unsigned int MIN_PWM = 40;

    static constexpr double PULSES_RAD = 35.7094489195534;
};