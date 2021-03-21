#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Twist.hpp"
#include "Wheel.hpp"
#include <array>
#include <memory>

class Drivetrain
{
    std::array<std::unique_ptr<Wheel>,4> wheels {
        std::make_unique<Wheel>(WHEELS::WHEEL_1),
        std::make_unique<Wheel>(WHEELS::WHEEL_2),
        std::make_unique<Wheel>(WHEELS::WHEEL_3),
        std::make_unique<Wheel>(WHEELS::WHEEL_4)
    };

public:
    double get_angle(); // pull from rio
    void   drive(Twist_I const& twist);
};

#endif // __DRIVETRAIN_H__