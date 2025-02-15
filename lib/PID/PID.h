#pragma once

class PID
{
public:
    struct Result
    {
        double output;
        double integral;
        double derivative;
        double proportional;
        double integralError;
    };
    struct CtrlConfig
    {
        double kp;               // PID Kp parameter
        double ki;               // PID Ki parameter
        double kd;               // PID Kd parameter
        double initialOutput;   // output, if any, already set
        double maxOutput;       // PID maximum output limitation
        double minOutput;       // PID minimum output limitation
        double maxIntegral;     // PID maximum integral value limitation
        double minIntegral;     // PID minimum integral value limitation
    };
    PID(const CtrlConfig &config);
    void updateParameters(const CtrlConfig &params);
    void compute(double inputError, Result &output);

private:
    struct CtrlBlock
    {
        double Kp;               // PID Kp value
        double Ki;               // PID Ki value
        double Kd;               // PID Kd value
        double previousErr1;    // e(k-1)
        double previousErr2;    // e(k-2)
        double integralErr;     // Sum of error
        double lastOutput;      // PID output in last control period
        double delta;            // output adjustment
        double maxOutput;       // PID maximum output limitation
        double minOutput;       // PID minimum output limitation
        double maxIntegral;     // PID maximum integral value limitation
        double minIntegral;     // PID minimum integral value limitation
    } pid;
    static double calcIncPos(CtrlBlock &pid, double error);
};
