#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "Constants.hpp"
#include <complex>
#include "Twist.hpp"
#include <iostream>
#include "drivetrain.hpp"
#include <thread>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <ctre\Phoenix.h>

using namespace std::complex_literals; // makes '1i' work

// steering ratio 12.8:1
class Wheel
{
    inline static int wheelid_ = 0;
    int wheelid = wheelid_++;
    using float_t = double;
    TalonFX driver, turner;
    CANCoder direction;
    can_adr cancoder_adr;
public:
    Wheel(WHEELS::WheelInfo const &wheel_info);
    Wheel(Wheel const&) = delete;
    Wheel(Wheel &&) = delete;
    void printAngle();
    float_t get_angle();
    void drive(frc::SwerveModuleState const &twist);
};
#endif // __WHEEL_H__