#include <sys/param.h>
#include <esp32-hal-log.h>

#include "PID.h"

void PID::compute(double error, Result &result)
{
    /* Add current error to the integral error */
    pid.integralErr += error;
    /* If the integral error is out of the range, it will be limited */
    pid.integralErr = MIN(pid.integralErr, pid.maxIntegral);
    pid.integralErr = MAX(pid.integralErr, pid.minIntegral);

    result.integralError = pid.integralErr;

    /* Calculate the pid control value by location formula */
    /* u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki */

    result.proportional = error * pid.Kp;
    result.derivative = (error - pid.previousErr1) * pid.Kd;
    result.integral = pid.integralErr * pid.Ki;
    pid.delta = result.proportional + result.derivative + result.integral;

    /* compute full output */
    result.output = pid.lastOutput + pid.delta;

    /* If the output is out of the range, it will be limited */
    result.output = MIN(result.output, pid.maxOutput);
    result.output = MAX(result.output, pid.minOutput);

    /* Update last output */
    pid.lastOutput = result.output;

    /* Update previous error */
    pid.previousErr1 = error;
}

PID::PID(const CtrlConfig &config)
{
    pid.previousErr1 = 0.0;
    pid.previousErr2 = 0.0;
    pid.integralErr = 0.0;
    updateParameters(config);
}

void PID::updateParameters(const CtrlConfig &params)
{
    pid.Kp = params.kp;
    pid.Ki = params.ki;
    pid.Kd = params.kd;
    pid.lastOutput = params.initialOutput;
    pid.maxOutput = params.maxOutput;
    pid.minOutput = params.minOutput;
    pid.maxIntegral = params.maxIntegral;
    pid.minIntegral = params.minIntegral;
}
