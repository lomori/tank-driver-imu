#pragma once

class DifferentialDrive
{
    public:
        struct pps
        {
            double motor1;
            double motor2;
        };        
        struct velocities
        {
            double linear;
            double angular;
        };
        DifferentialDrive(double wheelDiameter, double wheelsDistance, double maxPPS);
        void calculatePPS(velocities &v, pps &p);
        void getVelocities(pps &p, velocities &v);
    private:
        double wheelDiameter_;
        double wheelsDistance_;
        double maxPPS_;
};

