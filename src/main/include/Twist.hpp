#ifndef __TWIST_H__
#define __TWIST_H__

#include "Constants.hpp"
#include <cmath>

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
};

#endif // __TWIST_H__