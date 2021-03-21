#include "drivetrain.hpp"
// #include <cmath>

// get robot orientation from rio / navX
double Drivetrain::get_angle()
{
    return pi;
}

// drive in specified direction
void Drivetrain::drive(Twist_I const &twist)
{
    Twist_R const twist_r { twist, get_angle() };
    for(auto &wheel : wheels)
        wheel->drive(twist_r);
}
