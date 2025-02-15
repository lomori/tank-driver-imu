#include "DifferentialDrive.h"

//#include <stdio.h>

//#include <stdlib.h>

//#include <algorithm>

#include <Constants.h>

// https://blog.hadabot.com/ros2-unicycle-to-differential-drive-kinematics.html

DifferentialDrive::DifferentialDrive(double wheelDiameter, double wheelsDistance, double maxPPS)
    : wheelDiameter_(wheelDiameter), wheelsDistance_(wheelsDistance), maxPPS_(maxPPS)
{
}

void DifferentialDrive::calculatePPS(velocities &v,
                                     DifferentialDrive::pps &pps)
{
    //printf("Velocities input: %7.4f   %7.4f\n", v.linear, v.angular);

    double vL = ((2.0 * v.linear) - (v.angular * wheelsDistance_)) / wheelDiameter_;
    double vR = ((2.0 * v.linear) + (v.angular * wheelsDistance_)) / wheelDiameter_;

    //printf("Wheel angular velocities (rad/s): %7.4f   %7.4f\n", vL, vR);

    double ppsL = vL * Constants::PULSES_RAD;
    double ppsR = vR * Constants::PULSES_RAD;

    double maxPPS = std::max(std::abs(ppsL), std::abs(ppsR));

    if (maxPPS > maxPPS_)
    {
        double scale = maxPPS_ / maxPPS;
        //printf("Scale: %7.4f", scale);
        ppsL *= scale;
        ppsR *= scale;
    }

    pps.motor1 = ppsL;
    pps.motor2 = ppsR;
}

void DifferentialDrive::getVelocities(pps &p, velocities &v)
{
    //printf("PPS Input: %7.4f    %7.4f\n", p.motor1, p.motor2);

    double vL = p.motor1 / Constants::PULSES_RAD;
    double vR = p.motor2 / Constants::PULSES_RAD;

    //printf("Wheel angular velocities (rad/s): %7.4f   %7.4f\n", vL, vR);

    v.angular = ((vR-vL) * wheelDiameter_ / 2.0) / wheelsDistance_;
    v.linear = ((vL * wheelDiameter_) + (v.angular * wheelsDistance_)) / 2.0;
}
