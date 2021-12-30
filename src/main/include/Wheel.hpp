#pragma once

#include <complex>
#include "Twist.hpp"


#include <ctre\Phoenix.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <thread>
#include <iostream>


using namespace std::complex_literals; // makes '1i' work
using can_adr = int;
// steering ratio 12.8:1

namespace WHEELS
{
    struct WheelInfo
    {
        // stored in radians and inches
        can_adr const            driver, turner, cancoder;
        frc::Translation2d const wheel_pos;
        units::inch_t const      radius;
        units::degree_t const    offset;

        // inputs in degrees and inches
        constexpr WheelInfo(can_adr d, can_adr t, can_adr c, frc::Translation2d wheel_position, units::inch_t diameter_, units::degree_t offset_)
            : driver { d }
            , turner { t }
            , cancoder { c }
            , wheel_pos { wheel_position }
            , radius { diameter_ / 2 }
            , offset { offset_ }
        {}
    };
    WheelInfo const WHEEL_1 { 30, 31, 11, { 11_in, -11_in }, 4_in, 0_deg };  // 360-275
    WheelInfo const WHEEL_2 { 40, 41, 12, { 11_in, 11_in }, 4_in, 0_deg };   // 15
    WheelInfo const WHEEL_3 { 50, 51, 13, { -11_in, 11_in }, 4_in, 0_deg };  // 360-83.9
    WheelInfo const WHEEL_4 { 60, 61, 14, { -11_in, -11_in }, 4_in, 0_deg }; // 75

    constexpr double speed_mult = 1; // hacky way to deal with joysticks
} // namespace WHEELS

class Wheel
{
    inline static int id_setter = 0;
    int const         id        = id_setter++;
    using float_t               = double;

    TalonFX  driver, turner;
    CANCoder direction;
    can_adr  cancoder_adr;

    frc::Translation2d const wheel_pos;

    units::inch_t   radius;
    units::degree_t offset;

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
    float_t     getAngle();
    std::thread drive(frc::SwerveModuleState const& state);
    void        stop();
};