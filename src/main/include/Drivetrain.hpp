#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Twist.hpp"
#include "Wheel.hpp"
#include <AHRS.h>
#include <array>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <iostream>
#include <memory>

class Drivetrain
{
    std::array<std::unique_ptr<Wheel>, 4> wheels {
        std::make_unique<Wheel>(WHEELS::WHEEL_1),
        std::make_unique<Wheel>(WHEELS::WHEEL_2),
        std::make_unique<Wheel>(WHEELS::WHEEL_3),
        std::make_unique<Wheel>(WHEELS::WHEEL_4)
    };

    frc::SwerveDriveKinematics<4> const m_kinematics {
        *wheels[0],
        *wheels[1],
        *wheels[2],
        *wheels[3]
    };

    std::unique_ptr<AHRS> navx { std::make_unique<AHRS>(frc::SPI::Port::kMXP) };

public:
    Drivetrain();
    double get_angle(); // pull from rio
    void   print();
    void   drive(frc::ChassisSpeeds const& feild_speeds);
    void   drive(wpi::array<frc::SwerveModuleState, 4> const& states);
    void   gotoZero();
    void   goto180();
};

#endif // __DRIVETRAIN_H__