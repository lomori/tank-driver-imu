

#include "Controller.h"

void Controller::correctAngularVelocity(double averageAngularVelocity)
{
    double pps1 = engineL.getPPS();
    double pps2 = engineR.getPPS();

    if (abs(pps1) > Constants::EPSLON || abs(pps2) > Constants::EPSLON)
    {
        lastAngularUpdate = millis();
        double error = vel.angular - averageAngularVelocity;
        PID::Result result;
        pid->compute(error, result);
        esp_log_write(ESP_LOG_INFO, tag, "AV: expect %7.3f avg %7.3f err %7.3f out %7.3f T Err %7.3f P %7.3f I %7.3f D %7.3f\n",
                      vel.angular, averageAngularVelocity, error,
                      result.output,
                      result.integralError,
                      result.proportional,
                      result.integral,
                      result.derivative);
        DifferentialDrive::pps pps;
        DifferentialDrive::velocities newVel;
        newVel.angular = vel.angular + result.output;
        newVel.linear = vel.linear;
        kinematics.calculatePPS(newVel, pps);
        engineL.setPPS(pps.motor1);
        engineR.setPPS(pps.motor2);
    }
}

double Controller::getActualLinearVelocity()
{
    double ppsL = engineL.getActualPPS();
    double ppsR = engineR.getActualPPS();

    double vl = ppsL * Constants::MM_PULSE / 1000.0;
    double vr = ppsR * Constants::MM_PULSE / 1000.0;

    return (vl + vr) / 2.0;
}

void Controller::init(IMU *imu)
{
    leftCalibration.forward = Constants::xValuesLF;
    leftCalibration.backward = Constants::xValuesLB;
    rightCalibration.forward = Constants::xValuesRF;
    rightCalibration.backward = Constants::xValuesRB;

    vel.angular = 0.0;
    vel.linear = 0.0;

    pidConfig.kp = 0.6;
    pidConfig.kd = 0.4;
    pidConfig.ki = 0.1;
    pidConfig.maxOutput = 255;
    pidConfig.minOutput = -255;
    pidConfig.maxIntegral = 100;
    pidConfig.minIntegral = -100;
    pidConfig.initialOutput = 0.0;

    engineL.init("Left", Constants::L_PWM, Constants::L_PWM_CHANNEL,
                 Constants::L_PIN1, Constants::L_PIN2,
                 Constants::L_COUNTER_PINA, Constants::L_COUNTER_PINB,
                 &leftCalibration, Constants::L_BIAS);
    engineR.init("Right", Constants::R_PWM, Constants::R_PWM_CHANNEL,
                 Constants::R_PIN1, Constants::R_PIN2,
                 Constants::R_COUNTER_PINA, Constants::R_COUNTER_PINB,
                 &rightCalibration, Constants::R_BIAS);

    pinMode(Constants::ENABLE_ENGINES, OUTPUT);
    digitalWrite(Constants::ENABLE_ENGINES, HIGH);

    av.init(5, 100, imu);
    lastAngularUpdate = millis();
    lastLinearUpdate = millis();
}

void Controller::stopMotor(Engine &engine, bool force)
{
    if (force || abs(engine.getPPS()) > Constants::EPSLON)
    {
        esp_log_write(ESP_LOG_INFO, tag, "%s engine stopped.\n", engine.getTag());
        engine.setPPS(0.0);
    }
}

void Controller::stop()
{
    vel.linear = 0;
    vel.angular = 0;
    stopMotor(engineL, true);
    stopMotor(engineR, true);
    engineTimeout = Constants::MAX_ULONG;
    if (pid)
    {
        delete pid;
        pid = nullptr;
    }
 }

void Controller::setDynamics(double linearX, double angularZ, unsigned int timeoutMS)
{
    vel.linear = linearX;
    vel.angular = angularZ;

    DifferentialDrive::pps pps;

    kinematics.calculatePPS(vel, pps);
    bool moving = abs(pps.motor1) > Constants::EPSLON ||
                  abs(pps.motor2) > Constants::EPSLON;

    bool changedL = engineL.setPPS(pps.motor1);
    bool changedR = engineR.setPPS(pps.motor2);

    if (timeoutMS > 0 && moving)
        engineTimeout = millis() + timeoutMS;
    else
        engineTimeout = Constants::MAX_ULONG;

    if (changedL || changedR)
    {
        if (pid)
        {
            delete pid;
            pid = nullptr;
        }
        if (moving)
        {
            pid = new PID(pidConfig);
            lastLinearUpdate = millis();
            lastAngularUpdate = millis();
            av.reset();
        }
        else
        {
            lastLinearUpdate = Constants::MAX_ULONG;
            lastAngularUpdate = Constants::MAX_ULONG;
        }
    }
}

void Controller::stopMotors(bool force)
{
    stopMotor(engineL, force);
    stopMotor(engineR, force);
}

bool firstCall = true;

void Controller::controlLoop()
{
    unsigned long now = millis();

    if (firstCall)
    {
        esp_log_write(ESP_LOG_INFO, tag, "CL: %d/%d\n",
                      xPortInIsrContext(), xPortGetCoreID());
        firstCall = false;
    }

    if (engineTimeout <= now)
    {
        stopMotors(true);
        engineTimeout = Constants::MAX_ULONG;
    }

    if (correctionEnabled)
    {
        av.checkUpdate();

        if ((now - lastAngularUpdate) >= Constants::ANGULAR_UPDATE_MS)
        {
            correctAngularVelocity(av.getAverage());
            lastAngularUpdate = now;
        }
    }

    if ((now - lastLinearUpdate) >= Constants::LINEAR_UPDATE_MS)
    {
        engineR.controller();
        engineL.controller();
        lastLinearUpdate = now;
    }
}
