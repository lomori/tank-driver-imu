#include "Engine.h"

#include <InterpolationLib.h>

#include <Constants.h>

void Engine::stop()
{
    encoder.pauseCount();
    ledcWrite(pwmChannel, 0);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    previousCounter = 0;
    lastUpdateMS = millis();

    if (pid)
    {
        delete pid;
        pid = nullptr;
    }
    pps = 0.0;
    actualPPS = 0.0;
    encoder.clearCount();
}

unsigned int Engine::pps2pwm(double pps)
{
    unsigned int pwm = 0;
    if (pps > 0)
        pwm = round(Interpolation::Linear(
            calibration->forward, Constants::__yValues,
            Constants::NUM_CALIBRATIONS,
            pps, true));
    else
        pwm = round(Interpolation::Linear(
            calibration->backward, Constants::__yValues,
            Constants::NUM_CALIBRATIONS,
            -pps, true));
    return min(max(pwm, Constants::MIN_PWM), Constants::MAX_PWM);
}

double Engine::getPPS()
{
    return pps;
}

double Engine::getActualPPS()
{
    return actualPPS;
}

const double MAX_PULSES_PER_MS = 700.0 / 1000.0;

void Engine::controller()
{
    if (correctionEnabled && abs(pps) > Constants::EPSLON)
    {
        unsigned long now = millis();
        unsigned long elapsed = now - lastUpdateMS;
        lastUpdateMS = now;

        double expected = abs(pps * elapsed / 1000.0);
        volatile unsigned long currentCounter = abs(encoder.getCount());
        unsigned long deltaCounter = currentCounter - previousCounter;
        actualPPS = deltaCounter * 1000 / elapsed;
        previousCounter = currentCounter;

        // sanity check: sometimes are are getting very weird values.
        double maxPPS = MAX_PULSES_PER_MS * elapsed;
        if (deltaCounter > maxPPS)
        {
            esp_log_write(ESP_LOG_ERROR, localTag,
                          "%s: Invalid delta counter (%lu)\n", localTag, deltaCounter);
            deltaCounter = round(expected);
        }

        double error = expected - deltaCounter;

        if (accelerating)
        { // disregard first cycle as robot is accelerating.
            accelerating = false;
            esp_log_write(ESP_LOG_INFO, localTag, "%s: accelerating\n", localTag);
        }
        else
        {
            if (abs(bias) > 0)
            {
                error *= (1.0 + bias);
            }

            PID::Result result;
            pid->compute(error, result);
            uint32_t output = round(result.output);
            ledcWrite(pwmChannel, output);

            esp_log_write(ESP_LOG_INFO, localTag,
                          "%s:%d/%d,%6.2f,%3lu,%6.2f,%3d,%6.2f,%6.2f,%6.2f,%6.2f\n",
                          localTag,
                          xPortInIsrContext(), xPortGetCoreID(),
                          expected,
                          deltaCounter,
                          error,
                          output,
                          result.integralError,
                          result.proportional,
                          result.integral,
                          result.derivative
                          );
        }
    }
}

bool Engine::setPPS(double pps)
{
    bool changed = false;

    if (abs(pps - this->pps) > Constants::EPSLON)
    {
        esp_log_write(ESP_LOG_INFO, localTag, "%s:%d/%d PPS: %f\n", localTag,
              xPortInIsrContext(), xPortGetCoreID(), pps);

        // if changing directions or stopping, disconnect everything
        if (abs(pps) < Constants::EPSLON || pps * this->pps < 0)
            stop();

        if (abs(pps) > Constants::EPSLON)
        {
            if (pps > 0)
            {
                digitalWrite(pin1, LOW);
                digitalWrite(pin2, HIGH);
            }
            else
            {
                digitalWrite(pin1, HIGH);
                digitalWrite(pin2, LOW);
            }

            this->pps = pps;
            actualPPS = pps;

            uint32_t pwm = pps2pwm(pps);
            esp_log_write(ESP_LOG_INFO, localTag, "%s %lu Initial PWM: %d\n", localTag, millis(), pwm);

            pidConfig.initialOutput = pwm;

            if (pid) // if we are already running
            {
                // just update parameters but continue processing
                pid->updateParameters(pidConfig);
            }
            else
            {
                changed = true;
                accelerating = true;
                lastUpdateMS = millis();
                // else set brand new controller.
                pid = new PID(pidConfig);
                encoder.clearCount();
                encoder.resumeCount();
            }
            ledcWrite(pwmChannel, pwm);
        }
    }
    return changed;
}

void Engine::init(const char *tag, uint8_t pwmPin, uint8_t pwmChannel,
                  uint8_t pin1, uint8_t pin2, uint8_t counterPinA, uint8_t counterPinB,
                  Calibration *calibration, double bias)
{
    this->localTag = tag;
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->calibration = calibration;
    this->bias = bias;
    this->pwmChannel = pwmChannel;

    // ESP32Encoder::useInternalWeakPullResistors=DOWN;
    //  Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachSingleEdge(counterPinA, counterPinB);
    encoder.pauseCount();
    encoder.clearCount();

    pidConfig.kp = 6.0;
    pidConfig.kd = 3.0;
    pidConfig.ki = 1.0;
    pidConfig.maxOutput = 255;
    pidConfig.minOutput = 30; // do not allow direction change here
    pidConfig.maxIntegral = 100;
    pidConfig.minIntegral = -100;

    pps = 0.0;
    actualPPS = 0.0;
    previousCounter = 0;
    lastUpdateMS = millis();

    // sets the pins as outputs:
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);

    // attach the channel to the GPIO to be controlled
    pinMode(pwmPin, OUTPUT);
    digitalWrite(pwmPin, LOW);
    // configure PWM functionalitites
    ledcSetup(pwmChannel, FREQ, RESOLUTION);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(pwmPin, pwmChannel);
    ledcWrite(pwmChannel, 0);
}
