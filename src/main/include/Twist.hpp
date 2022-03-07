#pragma once

#include <cmath>
#include <iostream>

// cartesian and rotational velocities in a space relative to the feild
struct Twist_I
{
    double dx, dy, dtheta;
};

// Twist_I rotated into a space relative to the robot
struct Twist_R
{
    double dx, dy, dtheta;
    Twist_R(Twist_I const &twist, double robot_angle)
        : dx { twist.dx * std::cos(robot_angle) + twist.dy * std::sin(robot_angle) }
        , dy { twist.dy * std::cos(robot_angle) - twist.dx * std::sin(robot_angle) }
        , dtheta { twist.dtheta }
    {
    }
    

    friend std::ostream& operator<< (std::ostream& out, Twist_R const& tw)
    {
        return out << "[" << tw.dx << ',' << tw.dy << ',' << tw.dtheta <<']';
    }
};