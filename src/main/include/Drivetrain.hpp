#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Twist.hpp"
#include "Wheel.hpp"
#include <array>
#include <AHRS.h>
#include <memory>
#include <frc/SPI.h>
#include <iostream>

class Drivetrain
{
    std::array<std::unique_ptr<Wheel>,4> wheels {
        std::make_unique<Wheel>(WHEELS::WHEEL_1),
        std::make_unique<Wheel>(WHEELS::WHEEL_2),
        std::make_unique<Wheel>(WHEELS::WHEEL_3),
        std::make_unique<Wheel>(WHEELS::WHEEL_4)
    };
    std::unique_ptr<AHRS> navx { std::make_unique<AHRS>(frc::SPI::Port::kMXP)};

public:
    Drivetrain();
    double get_angle(); // pull from rio
    void print();
    void   drive(Twist_I const& twist);
    void gotoZero();
    void goto180();
};

#endif // __DRIVETRAIN_H__