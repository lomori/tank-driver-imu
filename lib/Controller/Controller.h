#pragma once

#include <DifferentialDrive.h>

#include <Engine.h>

#include <PID.h>

#include <Constants.h>

#include <IMU.h>

#include <AngularVelocity.h>

class Controller
{
public:
    Controller() : kinematics(Constants::WHEEL_DIAMETER,
        Constants::Y_WHEELS_DISTANCE_M, Constants::MAX_PPS){};
    void init(IMU* imu);
    void controlLoop();
    void stopMotors(bool force);
    void stop();
    double getActualLinearVelocity();
    void setDynamics(double linearX, double angularZ, unsigned int timeoutMS);
    bool correctionEnabled = false;

private:
    static constexpr const char *tag="Controller";
    unsigned long lastLinearUpdate;
    unsigned long lastAngularUpdate;
    unsigned long engineTimeout = Constants::MAX_ULONG;

    AngularVelocity av;
    DifferentialDrive kinematics;
    //DifferentialDrive::pps pps;
    DifferentialDrive::velocities vel;
    Engine::Calibration leftCalibration;
    Engine::Calibration rightCalibration;
    Engine engineL;
    Engine engineR;
    PID *pid = nullptr;        // PID algoritm handler
    PID::CtrlConfig pidConfig; // PID parameters

    void stopMotor(Engine &engine, bool force);
    void correctAngularVelocity(double averageAngularV);
};