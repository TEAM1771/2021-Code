#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "Constants.hpp"
#include <complex>
#include "Twist.hpp"
#include <thread>
#include <iostream>

#include <ctre\Phoenix.h>

#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/SwerveModuleState.h>

using namespace std::complex_literals; // makes '1i' work

// steering ratio 12.8:1
class Wheel
{
    inline static int id_setter = 0;
    int const         id        = id_setter++;
    using float_t               = double;

    TalonFX  driver, turner;
    CANCoder direction;
    can_adr  cancoder_adr;

    frc::Translation2d const wheel_pos;

    units::meter_t radius;

public:
    Wheel(WHEELS::WheelInfo const& wheel_info);
    Wheel(Wheel const&) = delete;
    Wheel(Wheel&&)      = delete;

    constexpr operator frc::Translation2d() const
    {
        return wheel_pos;
    }

    // void        zeroEncoder();
    void        printAngle();
    float_t     get_angle();
    std::thread drive(frc::SwerveModuleState const& state);
};
#endif // __WHEEL_H__